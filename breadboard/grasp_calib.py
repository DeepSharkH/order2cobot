"""Grasp calibration: after every successful grasp, measure where the part sits in the fingers.

Planned (user decision 2026-10-27, implement after the first successful grasps): the fingers
never hold a part exactly where gripper.yaml assumes, so before moving to the board the
executor measures the offset of the part's pin exits from the nominal `tcp_pins` and corrects
the TCP for this grasp only. The corrected TCP then drives the move above the target holes.

TODO(grasp calibration): choose and implement the measurement, e.g.
  - the gripper camera looking at the held part's bottom (pin exits) against the fingertips,
  - touching a reference (a calibrated hole / edge in the workspace) with the part, or
  - a fixed fixture that centres the part before the final grasp.
Return the TCP correction in the flange frame and log it (urcore.runlog note observed).
Teach the calibration corners with the same measurement (corner `tcp` in calibration.yaml),
otherwise the teach-time grasp offset stays in the fit as a bias (breadboard/calibration.py).
"""


def measure(ex, gripper, part):
    """Offset of the held part's pin-exit midpoint from gripper['tcp_pins'] (flange frame, m),
    measured right after the grasp. `ex` is the urcore.urexec.URExec of the run."""
    raise NotImplementedError('TODO(grasp calibration): measure the part in the fingers after the grasp')


def corrected_tcp(gripper, offset):
    """gripper['tcp_pins'] shifted by the measured offset (flange frame)."""
    t = list(gripper['tcp_pins'])
    return [t[0] + offset[0], t[1] + offset[1], t[2] + offset[2]] + t[3:]
