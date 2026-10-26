"""Anchor fit: correct the camera registration with seated insertions.

The part is rigid, but taped down: it creeps ~1 mm when a seat pushes on it. So anchors
are grouped in epochs (a new epoch starts when an anchor lands > MOVE_TOL off its
prediction), and the fit has

    shared:     pitch scales s = (sx, sy)              (the part does not stretch)
    per epoch:  in-plane offset d_e (base, m) + yaw_e   (the part slid / turned)

with soft priors: epoch 0 near the registration, epoch e near epoch e-1 (a creep, not a
teleport). Only the error across each insertion axis counts (depth is the force
controller's job), which leaves exactly the in-plane DOF: x / z on the switch face,
x / y on the breadboard.

    1 anchor  -> translation of its epoch (yaw / pitch from the prior or older epochs)
    2 anchors far apart (port 1 + port 8, a1 + j30) -> + yaw and the pitch along them
    3+ (board, not collinear) -> + both pitches

Anchors and predictions are in the same tool coordinates (plug/lead tip TCP), so the TCP
model error is absorbed as long as the grip is unchanged.
"""
import numpy as np
from scipy.optimize import least_squares


SIGMA_MEAS = 0.3e-3       # a constrained-tip position across the axis (compliant engage)
SIGMA_YAW = np.radians(1.0)
SIGMA_SCALE = 0.015
SIGMA_CREEP_T = 3e-3      # epoch-to-epoch slide of a taped part
SIGMA_CREEP_YAW = np.radians(1.5)


def _T(T0, d, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    T = np.eye(4)
    T[:3, :3] = Rz @ T0[:3, :3]
    T[:3, 3] = T0[:3, 3] + d
    return T


def _lateral(T, f, v):
    a = T[:3, :3] @ f.axis
    return v - a * (v @ a)


class Fit:
    """Solved fit; .T / .scale are the latest epoch's pose."""

    def __init__(self, model, prior, anchors):
        self.model = model
        self.T0 = np.array(prior['T'])
        self.s0 = np.array(prior.get('scale', [1.0, 1.0]), float)
        sig_t = prior.get('sigma_t', 2e-3)
        sig_yaw = prior.get('sigma_yaw', SIGMA_YAW)
        sig_s = np.array(prior.get('sigma_scale', [SIGMA_SCALE, SIGMA_SCALE]), float)
        if model.kind == 'switch':
            sig_s[1] = 1e-6               # no features off the row: y pitch is meaningless
        self.n_ep = 1 + max([a.get('epoch', 0) for a in anchors], default=0)
        feats = [model.feature(a['feature']) for a in anchors]
        meas = [np.array(a['base']) for a in anchors]
        eps = [a.get('epoch', 0) for a in anchors]

        def res(x):
            s = self.s0 * (1 + x[:2])
            r = [x[:2] / sig_s]
            for e in range(self.n_ep):
                d, yaw = x[2 + 4 * e:5 + 4 * e], x[5 + 4 * e]
                if e == 0:
                    r += [d / sig_t, [yaw / sig_yaw]]
                else:
                    dp, yp = x[2 + 4 * (e - 1):5 + 4 * (e - 1)], x[5 + 4 * (e - 1)]
                    r += [(d - dp) / SIGMA_CREEP_T, [(yaw - yp) / SIGMA_CREEP_YAW]]
            for f, m, e in zip(feats, meas, eps):
                T = _T(self.T0, x[2 + 4 * e:5 + 4 * e], x[5 + 4 * e])
                r.append(_lateral(T, f, model.place(T, s, f.p) - m) / SIGMA_MEAS)
            return np.concatenate(r)

        sol = least_squares(res, np.zeros(2 + 4 * self.n_ep), method='lm')
        self.x = sol.x
        self.cov = np.linalg.pinv(sol.jac.T @ sol.jac)
        self.scale = self.s0 * (1 + self.x[:2])
        self.T = self.pose(self.n_ep - 1)
        self.resid = [float(np.linalg.norm(_lateral(self.pose(e), f, model.place(self.pose(e), self.scale, f.p) - m)))
                      for f, m, e in zip(feats, meas, eps)]

    def pose(self, e):
        return _T(self.T0, self.x[2 + 4 * e:5 + 4 * e], self.x[5 + 4 * e])

    def sigma(self, fid):
        """1-sigma position uncertainty of a feature in the latest epoch (m): (lateral, along)."""
        f = self.model.feature(fid)
        e = self.n_ep - 1

        def pos(xx):
            return self.model.place(_T(self.T0, xx[2 + 4 * e:5 + 4 * e], xx[5 + 4 * e]), self.s0 * (1 + xx[:2]), f.p)

        J = np.zeros((3, len(self.x)))
        for k in range(len(self.x)):
            dx = np.zeros(len(self.x))
            dx[k] = 1e-6
            J[:, k] = (pos(self.x + dx) - pos(self.x - dx)) / 2e-6
        C = J @ self.cov @ J.T
        a = self.T[:3, :3] @ f.axis
        P = np.eye(3) - np.outer(a, a)
        return float(np.sqrt(max(np.linalg.eigvalsh(P @ C @ P).max(), 0))), float(np.sqrt(max(a @ C @ a, 0)))
