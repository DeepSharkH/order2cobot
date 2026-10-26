"""3-DoF ICP of a CAD prism against synthetic clouds with a known pose (agents/vision/icp.py)."""

import numpy as np

from agents.vision.icp import (
    boundary_closest,
    convex_hull,
    icp_fit,
    icp_fit_either_way,
    object_footprint,
    prism_closest,
    rectangle,
)
from agents.vision.locate import cloud_fit

L, W, H = 0.16602, 0.05608, 0.00951
TRUE = np.array([0.031, 0.047, np.radians(92.0)])


def world(uvh: np.ndarray, pose: np.ndarray) -> np.ndarray:
    x, y, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)
    return np.c_[x + c * uvh[:, 0] - s * uvh[:, 1], y + s * uvh[:, 0] + c * uvh[:, 1], uvh[:, 2]]


def cloud(
    poly=None,
    pose=TRUE,
    view=(-1.0, 0.0),
    walls=True,
    hide_far=True,
    seed=0,
    noise=0.0008,
    step=0.0025,
):
    """Plate (h = 0) and a prism on it, as seen from direction `view` (object frame u, v):
    walls facing it are sampled, the plate behind the object (in the view's shadow) is not.
    view None = straight down (no walls, nothing hidden). Returns world xyz with h as z."""
    poly = rectangle(L, W) if poly is None else poly
    rng = np.random.default_rng(seed)
    # a world-aligned sampling grid (as a camera's pixels are), not aligned with the object
    g = np.arange(-0.15, 0.15, step) + rng.uniform(0, step)
    X, Y = np.meshgrid(g, g)
    d = np.c_[X.ravel(), Y.ravel()]
    c, s = np.cos(pose[2]), np.sin(pose[2])
    uv = np.c_[c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]]  # object frame
    _, d, inside = boundary_closest(uv, poly)
    pts = [np.c_[uv[inside], np.full(inside.sum(), H)]]  # top
    plate = uv[~inside]
    if view is not None and hide_far:  # shadow: plate just behind the object, seen from `view`
        vd = np.asarray(view) / np.linalg.norm(view)
        back = plate - vd * 0.02  # step 2 cm toward the object along the view: inside = shadowed
        _, _, shadow = boundary_closest(back, poly)
        plate = plate[~shadow]
    pts.append(np.c_[plate, np.zeros(len(plate))])
    if view is not None and walls:  # walls whose outward normal faces the viewer
        vd = np.asarray(view) / np.linalg.norm(view)
        a, b = poly, np.roll(poly, -1, 0)
        for p0, p1 in zip(a, b, strict=True):
            e = p1 - p0
            n = np.array([e[1], -e[0]]) / np.linalg.norm(e)  # outward for a CCW polygon
            if n @ vd <= 0.2:
                continue
            t = np.arange(0, 1, step / np.linalg.norm(e))
            z = np.arange(0.0005, H, step / 2)
            T, Z = np.meshgrid(t, z)
            pts.append(np.c_[p0 + T.ravel()[:, None] * e, Z.ravel()])
    P = world(np.concatenate(pts), pose)
    P[:, 2] += rng.normal(0, noise, len(P))
    return P


def err(pose):
    d = (pose[:2] - TRUE[:2]) * 1000
    dyaw = np.degrees((pose[2] - TRUE[2] + np.pi) % (2 * np.pi) - np.pi)
    return np.abs(d).max(), abs(dyaw)


START = TRUE + [0.003, -0.002, np.radians(1.0)]  # as the coarse search leaves it


def test_prism_closest_points() -> None:
    poly = rectangle(0.1, 0.05)
    q = np.array(
        [
            [0.01, 0.0, 0.02],  # above the top: straight down onto it
            [0.07, 0.0, 0.009],  # beside the +u wall, below the top: onto the wall
            [0.07, 0.0, 0.03],  # off the top edge, above: onto the top edge
            [0.0, 0.0, 0.004],  # inside, low: nearest is the top (5 mm) not a wall (25 mm)
        ]
    )
    c = prism_closest(q, poly, 0.009)
    assert np.allclose(c[0], [0.01, 0.0, 0.009])
    assert np.allclose(c[1], [0.05, 0.0, 0.009])
    assert np.allclose(c[2], [0.05, 0.0, 0.009])
    assert np.allclose(c[3], [0.0, 0.0, 0.009])
    _, d, inside = boundary_closest(np.array([[0.0, 0.0], [0.06, 0.0]]), poly)
    assert inside.tolist() == [True, False] and np.allclose(d, [0.025, 0.01])


def test_convex_hull_and_footprint_of_copies() -> None:
    sq = np.array([[0, 0], [1, 0], [1, 1], [0, 1], [0.5, 0.5]], float)
    assert len(convex_hull(sq)) == 4
    fp = object_footprint((0.083, 0.056, 0.0095), (2, 1))
    assert np.allclose(np.ptp(fp, 0), [0.166, 0.056])


def test_oblique_view_with_the_far_end_hidden() -> None:
    xyz = cloud(view=(-1.0, -0.6))  # the fixed camera: sees the -u end and -v side
    r = icp_fit(xyz, xyz[:, 2], START, rectangle(L, W), H)
    e_mm, e_deg = err(r["pose"])
    assert e_mm < 0.5 and e_deg < 0.2
    assert r["inliers"] > 0.9 and r["residual_mm"]["median"] < 1.0


def test_top_down_view_without_walls_is_still_constrained() -> None:
    # plain point-to-plane ICP has nothing to hold x, y, yaw here (all normals vertical)
    xyz = cloud(view=None)
    r = icp_fit(xyz, xyz[:, 2], START, rectangle(L, W), H)
    e_mm, e_deg = err(r["pose"])
    assert e_mm < 0.7 and e_deg < 0.3


def test_asymmetric_cad_footprint_from_the_wrong_end() -> None:
    poly = convex_hull(
        np.array(
            [[-0.083, -0.028], [0.083, -0.028], [0.083, 0.0167], [0.07, 0.028], [-0.083, 0.028]]
        )
    )  # one corner cut 13 x 11 mm
    xyz = cloud(poly=poly, view=(-0.3, -1.0), hide_far=False)
    flipped = START + [0, 0, np.pi]
    r = icp_fit_either_way(xyz, xyz[:, 2], flipped, poly, H)
    e_mm, e_deg = err(r["pose"])
    assert e_mm < 0.5 and e_deg < 0.2  # ends the right way round


def test_clutter_does_not_pull_the_fit() -> None:
    xyz = cloud(view=(-1.0, -0.6))
    rng = np.random.default_rng(3)
    wires = world(
        np.c_[
            rng.uniform(-0.06, 0.06, 300),
            rng.uniform(-0.02, 0.02, 300),
            rng.uniform(H + 0.001, H + 0.004, 300),
        ],
        TRUE,
    )  # on the top, low
    near = np.c_[
        TRUE[0] + rng.uniform(0.03, 0.05, 400),
        TRUE[1] + rng.uniform(-0.05, 0.05, 400),
        rng.uniform(0.003, 0.02, 400),
    ]  # another object beside the board
    xyz = np.concatenate([xyz, wires, near])
    r = icp_fit(xyz, xyz[:, 2], START, rectangle(L, W), H)
    e_mm, e_deg = err(r["pose"])
    assert e_mm < 1.0 and e_deg < 0.3


def test_icp_beats_the_edge_fit_when_walls_are_seen() -> None:
    """Oblique view (the fixed camera): the walls pin the board; the edge fit ignores them."""
    errs = {"icp": [], "edges": []}
    for seed in range(4):
        xyz = cloud(view=(-1.0, -0.6), seed=seed)
        h = xyz[:, 2]
        errs["icp"].append(err(icp_fit(xyz, h, START, rectangle(L, W), H, n_boot=0)["pose"])[0])
        errs["edges"].append(err(cloud_fit(xyz, h, START, L, W, H, n_boot=0)["pose"])[0])
    assert np.mean(errs["icp"]) < 0.5 * np.mean(errs["edges"])


def test_top_down_without_walls_both_fits_are_sub_millimetre() -> None:
    for seed in range(3):
        xyz = cloud(view=None, seed=seed)
        r = icp_fit(xyz, xyz[:, 2], START, rectangle(L, W), H, n_boot=0)
        e_mm, e_deg = err(r["pose"])
        assert e_mm < 0.4 and e_deg < 0.3
