"""Synthetic end-to-end check of the calibration solver.

Run:  .venv/bin/python -m pytest tests -q      (or just run this file)
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

from dashboard import calibration as cal
from dashboard import kinematics as kin


def _setup(seed=0, n=25, noise_px=0.2, dh_err=True):
    rng = np.random.default_rng(seed)
    board = cal.Board(8, 7, 0.015)
    K = np.array([[640.0, 0, 640], [0, 640, 360], [0, 0, 1]])
    dist = np.zeros(5)
    size = (1280, 720)
    # truth: board on the table under the camera at q0, camera looking along the flange z
    Y = kin.pose_to_T([-0.19, 0.46, 0.0], R.from_euler('x', 180, degrees=True).as_rotvec())
    X = kin.pose_to_T([0.0, -0.06, 0.03], R.from_euler('z', 3, degrees=True).as_rotvec())
    dh_true = kin.UR3E_NOMINAL_DH.copy()
    if dh_err:
        dh_true[1:5, 0] += rng.normal(0, 0.002, 4)      # joint zero offsets
        dh_true[1:3, 1] += rng.normal(0, 0.001, 2)      # link lengths
    q0 = np.array([-1.57, -1.3, 1.6, -1.9, -1.57, 0.0])
    Kf = np.array([[610.0, 0, 320], [0, 610, 240], [0, 0, 1]])
    F = kin.pose_to_T([0.0, -0.9, 0.8], R.from_euler('x', -140, degrees=True).as_rotvec())

    samples = []
    centre = Y[:3, :3] @ np.r_[(board.cols - 1) * board.square / 2, (board.rows - 1) * board.square / 2, 0] + Y[:3, 3]
    for _ in range(20000):
        if len(samples) == n:
            break
        q = q0 + rng.normal(0, [0.25, 0.15, 0.15, 0.25, 0.25, 0.6])
        Tc = kin.fk(q, dh_true) @ X
        P = kin.inv(Tc) @ np.c_[(Y[:3, :3] @ board.obj.T).T + Y[:3, 3], np.ones(len(board.obj))].T
        if np.any(P[2] < 0.1):
            continue
        uv = cal._project(P[:3].T, K, dist)
        if uv.min() < 10 or uv[:, 0].max() > size[0] - 10 or uv[:, 1].max() > size[1] - 10:
            continue
        if np.linalg.norm(Tc[:3, 3] - centre) > 0.45:
            continue
        Pf = kin.inv(F) @ np.c_[(Y[:3, :3] @ board.obj.T).T + Y[:3, 3], np.ones(len(board.obj))].T
        uvf = cal._project(Pf[:3].T, Kf, dist)
        samples.append({'id': len(samples), 'q': np.round(q, 4).tolist(),
                        'gripper_corners': (uv + rng.normal(0, noise_px, uv.shape)).tolist(),
                        'fixed_corners': (uvf + rng.normal(0, noise_px, uvf.shape)).tolist()})
    gcam = {'K': K, 'dist': dist, 'size': size}
    fcam = {'K': Kf, 'dist': dist, 'size': (640, 480)}
    return board, samples, gcam, fcam, dh_true, X, Y, F


def test_recovers_extrinsics_and_dh():
    board, samples, gcam, fcam, dh_true, X, Y, F = _setup()
    res = cal.solve(samples, board, gcam, fcam, kin.UR3E_NOMINAL_DH, {'intrinsics': False, 'dh': 'full'})
    Xs, Ys, Fs = (np.array(res[k]) for k in ('T_flange_gcam', 'T_base_board', 'T_base_fcam'))
    assert kin.pose_error(X, Xs)[0] < 0.002
    assert kin.pose_error(Y, Ys)[0] < 0.002
    assert kin.pose_error(F, Fs)[0] < 0.005
    # refining the DH table must explain the images better than the (wrong) prior table
    assert res['fits']['refined']['gripper_rms_px'] < 0.8 * res['fits']['controller']['gripper_rms_px']
    assert res['fits']['refined']['gripper_rms_px'] < 0.5
    # and move the flange towards the truth
    dh = np.array(res['dh_refined'])
    q = np.array(samples[0]['q'])
    assert kin.pose_error(kin.fk(q, dh_true), kin.fk(q, dh))[0] < kin.pose_error(kin.fk(q, dh_true), kin.fk(q, kin.UR3E_NOMINAL_DH))[0]


def test_intrinsics_from_samples():
    board, samples, gcam, fcam, *_ = _setup(seed=1, dh_err=False)
    guess = dict(gcam, K=np.array([[600.0, 0, 630], [0, 600, 350], [0, 0, 1]]))
    res = cal.solve(samples, board, guess, None, kin.UR3E_NOMINAL_DH, {'intrinsics': True, 'dh': 'none'})
    K = np.array(res['gripper_intrinsics']['K'])
    assert abs(K[0, 0] - 640) < 5 and abs(K[0, 2] - 640) < 5
    assert res['fits']['controller']['gripper_rms_px'] < 0.5


if __name__ == '__main__':
    test_recovers_extrinsics_and_dh()
    test_intrinsics_from_samples()
    print('ok')
