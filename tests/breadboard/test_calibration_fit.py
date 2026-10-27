"""breadboard.calibration: corner inserts -> board pose, hole -> base target (synthetic robot)."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from breadboard import calibration as cal
from breadboard.board import Board


def taught(board, T_base_board, tcp, holes_list):
    """Flange poses of a part seated by hand at each holes pair (the holes set its yaw)."""
    pl = cal.Placement({'fit': {'T_base_board': T_base_board.tolist()}}, {'tcp_pins': tcp}, board)
    return [{'holes': h, 'flange': pl.flange_target(h)} for h in holes_list]


def test_fit_recovers_board_pose_and_targets():
    board = Board()
    T = np.eye(4)
    T[:3, :3] = R.from_euler('z', 91.3, degrees=True).as_matrix()
    T[:3, 3] = [-0.45, -0.05, 0.012]
    tcp = [0.0, 0.0, 0.2645, 0, 0, np.pi / 2]
    corners = taught(board, T, tcp, [['A1', 'A3'], ['J1', 'J3'], ['A58', 'A60'], ['J58', 'J60']])   # in the 60- and 63-row CAD
    Tf, res = cal.fit({'corners': corners}, tcp, board)
    assert np.allclose(Tf, T, atol=1e-6) and res.max() < 1e-3
    # a TCP x/y error taught and used at the same orientation cancels
    tcp_err = [0.001, -0.0007, 0.2645, 0, 0, np.pi / 2]
    Tf2, res2 = cal.fit({'corners': corners}, tcp_err, board)
    pl = cal.Placement({'corners': corners, 'fit': {'T_base_board': Tf2.tolist()}}, {'tcp_pins': tcp_err}, board)
    want = taught(board, T, tcp, [['E30', 'E34']])[0]['flange']
    got = pl.flange_target(['E30', 'E34'])
    assert np.allclose(got, want, atol=1e-6)
    # flange above the seated pose, tool pointing down
    up = pl.flange_target(['E30', 'E34'], above=0.03)
    assert np.isclose(up[2] - got[2], 0.03, atol=1e-6) and cal.T_of(up)[2, 2] < -0.99


def test_flip_keeps_taught_yaw_and_tcp_checks():
    board = Board()
    tcp = [0.0, 0.0, 0.2645, 0, 0, np.pi / 2]
    corners = taught(board, np.eye(4), tcp, [['A1', 'A3'], ['J1', 'J3'], ['A58', 'A60']])
    pl = cal.Placement({'corners': corners, 'fit': {'T_base_board': np.eye(4).tolist(), 'tcp_pins': tcp}},
                       {'tcp_pins': tcp}, board)
    assert not pl.flip_for(['C10', 'C12']) and pl.flip_for(['C12', 'C10'])
    with pytest.raises(ValueError, match='changed since the fit'):
        cal.Placement({'corners': corners, 'fit': {'T_base_board': np.eye(4).tolist(), 'tcp_pins': tcp}},
                      {'tcp_pins': [0, 0, 0.25, 0, 0, np.pi / 2]}, board)
    with pytest.raises(ValueError, match='into the board'):
        cal.Placement({'corners': [], 'fit': {'T_base_board': np.eye(4).tolist()}},
                      {'tcp_pins': [0, 0, 0.25, np.pi, 0, 0]}, board)


def test_fit_needs_three_non_collinear_corners():
    board = Board()
    c = taught(board, np.eye(4), [0, 0, 0.26, 0, 0, 0], [['A1', 'A3'], ['B10', 'B12'], ['A20', 'A22']])
    with pytest.raises(ValueError, match='collinear'):
        cal.fit({'corners': c}, [0, 0, 0.26, 0, 0, 0], board)
    with pytest.raises(ValueError, match='needs 3'):
        cal.fit({'corners': c[:2]}, [0, 0, 0.26, 0, 0, 0], board)
