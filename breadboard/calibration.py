"""Where the breadboard is in base: the CAD hole table placed by manually taught corner inserts.

Calibration (configs/breadboard/calibration.yaml, written by `python -m breadboard.teach`):
  corners    a centre-gripped part seated by hand at known holes near the board corners; each
             record keeps the part's holes, the flange pose and the joints at that moment.
  workspace  named flange poses taught by hand (pick area, safe pose above the board, ...).
  fit        the board frame in base (T_base_board) solved from the corners, with residuals.

The taught point of a corner is the tool TCP (the corner's own `tcp` if it was grasp-calibrated
when taught, else (configs/breadboard/gripper.yaml `tcp_pins`: the
midpoint of the part's pin exits = the casing bottom, on the board top once seated) and its board
point is the midpoint of the part's holes on the CAD top face. 3+ corners that are not
collinear give T_base_board (Kabsch, rigid). Teach every corner and run every insertion at the
same tool orientation: an error in the TCP's x/y then shifts all taught points alike and
cancels in the fit for every part inserted at that orientation (a part seated in two holes of
one column sets it). Parts turned 90 deg (across rows) see the x/y error rotated: the grasp
calibration has to remove it. The z of the TCP has to be right in any case.

The cancellation needs the same TCP yaw at teach and run time: targets() keeps the part's x
closest to the taught one where the part allows it (flip_for). A per-grasp TCP correction must
be measured the same way at teach time (store it in the corner's `tcp`), or it re-adds the
teach-time grasp offset as a bias.

TODO(calibration, 2026-10-27): the manual corner inserts and workspace poses are taught on the
robot tomorrow (2026-10-27); until then there is no fit and `to_base` raises. After the first
successful grasps, the grasp calibration (breadboard/grasp_calib.py) corrects the TCP per grasp.
TODO(board model): fix the CAD to the real 63 rows and the measured e-f channel (board.py)
BEFORE teaching: the fit maps CAD hole positions to taught flange poses, so a corner taught
against the wrong hole table bakes the error into T_base_board. Teach corners spread over the
whole board (rows 1-3 and 61-63, both halves a-e and f-j), and re-fit after any CAD change.
"""
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

from breadboard.board import Board

ROOT = Path(__file__).resolve().parent.parent
CALIB = ROOT / 'configs' / 'breadboard' / 'calibration.yaml'
GRIPPER = ROOT / 'configs' / 'breadboard' / 'gripper.yaml'
RESIDUAL_WARN_MM = 0.3      # above this the board model (rows, channel) or a taught corner is off
MIN_SPREAD_M = 0.008        # RMS spread of the corners across their main axis (noise gain ~ 1/spread)


def load(path=CALIB):
    d = yaml.safe_load(Path(path).read_text()) if Path(path).exists() else None
    d = d or {}
    d.setdefault('corners', [])
    d.setdefault('workspace', {})
    return d


def save(calib, path=CALIB):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(yaml.safe_dump(calib, sort_keys=False))


def load_gripper(path=GRIPPER):
    return yaml.safe_load(Path(path).read_text())


def T_of(pose):
    """[x, y, z, rx, ry, rz] (m, rotvec) -> 4x4."""
    T = np.eye(4)
    T[:3, :3] = R.from_rotvec(pose[3:6]).as_matrix()
    T[:3, 3] = pose[:3]
    return T


def pose_of(T):
    return list(map(float, T[:3, 3])) + list(map(float, R.from_matrix(T[:3, :3]).as_rotvec()))


def kabsch(A, B):
    """Rigid T with B ~ T A (A, B: (n, 3), n >= 3 and not collinear)."""
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    Rm = Vt.T @ D @ U.T
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Rm, cb - Rm @ ca
    return T


def fit(calib, tcp, board=None):
    """T_base_board from the taught corners and the tool TCP (flange frame). Returns
    (T, residuals_mm per corner)."""
    board = board or Board()
    if len(calib['corners']) < 3:
        raise ValueError(f"{len(calib['corners'])} corner inserts taught, the fit needs 3 or more")
    A = np.array([board.part(c['holes'])[0] / 1000.0 for c in calib['corners']])
    B = np.array([(T_of(c['flange']) @ T_of(c.get('tcp', tcp)))[:3, 3] for c in calib['corners']])
    sv = np.linalg.svd(A - A.mean(0), compute_uv=False)
    if sv[1] / np.sqrt(len(A)) < MIN_SPREAD_M:
        raise ValueError('the corner inserts are (nearly) collinear: teach corners on both the '
                         'a-e and f-j sides and at both ends')
    T = kabsch(A, B)
    res = np.linalg.norm((A @ T[:3, :3].T + T[:3, 3]) - B, axis=1) * 1000
    return T, res


class Placement:
    """Hole ids -> base targets for the tool TCP, from a fitted calibration."""

    def __init__(self, calib=None, gripper=None, board=None):
        self.calib = calib if calib is not None else load()
        self.gripper = gripper or load_gripper()
        self.board = board or Board()
        if 'fit' not in self.calib:
            raise ValueError('breadboard not calibrated: python -m breadboard.teach fit')
        self.T = np.array(self.calib['fit']['T_base_board'])
        tcp = self.gripper['tcp_pins']
        if 'tcp_pins' in self.calib['fit'] and not np.allclose(self.calib['fit']['tcp_pins'], tcp, atol=1e-6):
            raise ValueError('configs/breadboard/gripper.yaml tcp_pins changed since the fit: '
                             'python -m breadboard.teach fit')
        if T_of(tcp)[2, 2] < 0.9:
            raise ValueError('tcp_pins z must point into the board like the flange z (see gripper.yaml)')
        # the part direction (TCP x) the corners were taught with, base frame
        self.x_taught = None
        if self.calib.get('corners'):
            xs = [(T_of(c['flange']) @ T_of(c.get('tcp', tcp)))[:3, 0] for c in self.calib['corners']]
            self.x_taught = np.mean(xs, axis=0)

    def to_base(self, holes):
        """Base position (m) of the part's TCP when seated at these holes, and the part
        direction in base (unit, first hole -> last)."""
        c, d = self.board.part(holes)
        return self.T[:3, :3] @ (c / 1000.0) + self.T[:3, 3], self.T[:3, :3] @ d

    def tcp_target(self, holes, flip=False):
        """Pose of the TCP seated at `holes`: x along the part (first -> last hole, reversed
        with flip for symmetric parts), z into the board (= the flange z of a downward tool),
        so insertion is along +z of the TCP."""
        p, d = self.to_base(holes)
        z = -self.T[:3, 2]
        x = -d if flip else d
        x = x - z * (x @ z)
        x /= np.linalg.norm(x)
        Tt = np.eye(4)
        Tt[:3, :3] = np.column_stack([x, np.cross(z, x), z])
        Tt[:3, 3] = p
        return Tt

    def flip_for(self, holes):
        """For symmetric parts: the flip that keeps the TCP x closest to the taught one (the
        TCP x/y error cancels only at the taught yaw; see the module docstring)."""
        if self.x_taught is None:
            return False
        return bool(self.tcp_target(holes)[:3, 0] @ self.x_taught < 0)

    def flange_target(self, holes, flip=False, above=0.0, tcp=None):
        """Flange pose (base) that puts the TCP `above` m over the seated target. `tcp`: a
        per-grasp TCP (grasp_calib.corrected_tcp) instead of gripper.yaml's."""
        Tt = self.tcp_target(holes, flip)
        Tt[:3, 3] -= Tt[:3, 2] * above
        return pose_of(Tt @ np.linalg.inv(T_of(tcp or self.gripper['tcp_pins'])))
