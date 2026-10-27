"""Object location on synthetic scenes with a known pose (agents/vision/locate.py)."""

import json
from pathlib import Path

import numpy as np
import pytest

from agents.vision import (
    CameraView,
    ObjectNotFound,
    ObjectSpec,
    box_corners,
    load_objects,
    locate_object,
)
from agents.vision.locate import (
    cloud_fit,
    coarse_search,
    fit_plane,
    image_fit,
    project,
    read_ply,
    write_ply,
)
from agents.vision.scene import load_scene

HALF = (0.08301, 0.05608, 0.00951)  # half breadboard: two make the protoboard
PLATE = -0.0045
TRUE = np.array([0.031, 0.047, np.radians(92.0)])  # x, y, yaw of the whole board


def spec(**kw) -> ObjectSpec:
    base = dict(
        name="protoboard", label="Protoboard", size=HALF, instances=2, plate_z=PLATE, images="never"
    )
    return ObjectSpec(**{**base, **kw})


def scene_cloud(
    pose=TRUE, size=(2 * HALF[0], HALF[1], HALF[2]), seed=0, hide_far_end=True, noise=0.0008
):
    """Plate (z = PLATE) with a box on it, sampled every 2.5 mm, with depth noise.

    hide_far_end: drop the plate behind the box's +u end, as the oblique fixed camera
    sees it (the box shadows it), so only three sides are fixed by the cloud.
    """
    rng = np.random.default_rng(seed)
    g = np.arange(-0.25, 0.25, 0.0025)
    X, Y = np.meshgrid(g, g)
    P = np.c_[X.ravel(), Y.ravel()]
    L, W, H = size
    x, y, yaw = pose
    u, v = np.array([np.cos(yaw), np.sin(yaw)]), np.array([-np.sin(yaw), np.cos(yaw)])
    d = P - [x, y]
    du, dv = d @ u, d @ v
    inside = (np.abs(du) <= L / 2) & (np.abs(dv) <= W / 2)
    z = np.where(inside, PLATE + H, PLATE)
    keep = ~(
        hide_far_end & ~inside & (du > L / 2) & (du < L / 2 + 0.03) & (np.abs(dv) < W / 2 + 0.01)
    )
    xyz = np.c_[P, z][keep]
    xyz[:, 2] += rng.normal(0, noise, len(xyz))
    return xyz


def err(r):
    p = r["box"]["pose"]
    d = np.array([p["x"], p["y"]]) - TRUE[:2]
    dyaw = (
        np.radians(p["yaw_deg"]) - TRUE[2] + np.pi / 2
    ) % np.pi - np.pi / 2  # a box is symmetric
    return np.abs(d).max() * 1000, abs(np.degrees(dyaw))


def test_fit_plane_finds_the_plate_under_clutter() -> None:
    xyz = scene_cloud()
    n, c = fit_plane(xyz)
    assert n[2] > 0.999 and abs(c[2] - PLATE) < 0.001


def test_finds_the_protoboard_from_a_partial_view() -> None:
    r = locate_object(spec(), scene_cloud())
    e_mm, e_deg = err(r)
    assert r["box"]["instances"] == [2, 1] and r["box"]["size_mm"] == [166.02, 56.08, 9.51]
    assert e_mm < 1.0 and e_deg < 0.5
    assert r["coarse"]["score"] > 0.6
    assert r["cloud_fit"]["coverage"]["+u end"] < 0.2 or r["cloud_fit"]["coverage"]["-u end"] < 0.2
    assert r["images_needed"]  # a side unseen: the image fit is asked for
    assert r["quality"]["ok"]  # images "never": nothing was promised


def test_needed_image_fit_without_a_trusted_camera_is_a_warning() -> None:
    xyz = scene_cloud()
    untrusted = CameraView(
        "cam",
        np.eye(3),
        np.zeros(5),
        (10, 10),
        np.zeros((10, 10, 3), np.uint8),
        np.eye(4),
        trusted=False,
    )
    r = locate_object(spec(images="auto"), xyz, [untrusted])
    assert r["method"] == "cloud" and not untrusted.used
    assert not r["quality"]["ok"] and "no camera pose is trusted" in r["quality"]["warnings"][0]


def test_instances_are_detected_when_not_given() -> None:
    r = locate_object(spec(instances=None), scene_cloud(hide_far_end=False))
    assert r["box"]["instances"] == [2, 1]
    assert err(r)[0] < 1.0


def test_empty_plate_is_not_found() -> None:
    xyz = scene_cloud(size=(0.0, 0.0, 0.0))
    with pytest.raises(ObjectNotFound):
        locate_object(spec(), xyz)


def test_a_taller_box_is_not_a_protoboard() -> None:
    # the switch next to it: 158 x 99 x 25 mm
    xyz = scene_cloud(size=(0.158, 0.099, 0.025), hide_far_end=False)
    with pytest.raises(ObjectNotFound):
        locate_object(spec(), xyz)


def test_cloud_cross_check_flags_inconsistent_cameras() -> None:
    xyz = scene_cloud(hide_far_end=False)
    moved = scene_cloud(pose=TRUE + [0.0, 0.012, 0.0], hide_far_end=False, seed=1)  # pose 12 mm off
    r = locate_object(spec(images="never"), xyz, check_clouds={"other cam": moved})
    d = r["checks"]["other cam cloud"]["delta_mm_mm_deg"]
    assert abs(d[1] - 12.0) < 1.5
    assert any("other cam cloud disagrees" in w for w in r["quality"]["warnings"])
    agree = locate_object(
        spec(images="never"),
        xyz,
        check_clouds={"other cam": scene_cloud(hide_far_end=False, seed=2)},
    )
    assert agree["quality"]["ok"]


# ── image fit on a rendered view ─────────────────────────────────────────────


def look_at(eye, target):
    """T_world_cam of an optical-frame camera (z forward, y down) at `eye` looking at `target`."""
    z = np.asarray(target, float) - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, :3] = np.stack([x, y, z], 1)
    T[:3, 3] = eye
    return T


def render(T, K, size, pose, L, W, H):
    """A white box on a grey plate, drawn by projecting its faces."""
    import cv2

    w, h = size
    img = np.full((h, w, 3), 90, np.uint8)
    cam = CameraView("render", K, np.zeros(5), size, img, T)
    c = box_corners(pose[:2], pose[2], L, W, PLATE, H)
    faces = [[4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
    shade = [235, 170, 190, 170, 190]
    # far faces first (painter's algorithm)
    depth = [project(cam, c[f])[1].mean() for f in faces]
    for i in np.argsort(depth)[::-1]:
        uv, _ = project(cam, c[faces[i]])
        cv2.fillConvexPoly(img, uv.round().astype(np.int32), (shade[i],) * 3, cv2.LINE_AA)
    return img


def test_image_fit_recovers_the_pose_from_an_oblique_view() -> None:
    K = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]])
    T = look_at(np.array([0.45, -0.35, 0.40]), [TRUE[0], TRUE[1], 0])
    L, W, H = 2 * HALF[0], HALF[1], HALF[2]
    img = render(T, K, (640, 480), TRUE, L, W, H)
    cam = CameraView("render", K, np.zeros(5), (640, 480), img, T)
    start = TRUE + [0.004, -0.003, np.radians(0.8)]
    r = image_fit([cam], start, L, W, PLATE, H, span_mm=8.0, span_deg=1.0, restarts=3)
    d = (r["pose"] - TRUE) * [1000, 1000, np.degrees(1)]
    assert np.abs(d[:2]).max() < 1.0 and abs(d[2]) < 0.5


# ── saved scenes and config ──────────────────────────────────────────────────


def test_objects_config_has_the_protoboard() -> None:
    specs = load_objects(Path(__file__).resolve().parents[2] / "configs/wire/objects.yaml")
    p = specs["protoboard"]
    assert p.size == pytest.approx((0.08301, 0.05608, 0.00951)) and p.instances == 2
    assert p.plate_z == pytest.approx(-0.0045)


def test_ply_round_trip(tmp_path: Path) -> None:
    xyz = np.random.default_rng(0).normal(size=(50, 3))
    rgb = np.random.default_rng(1).integers(0, 255, (50, 3)).astype(np.uint8)
    write_ply(tmp_path / "c.ply", xyz, rgb)
    x2, r2 = read_ply(tmp_path / "c.ply")
    assert np.allclose(x2, xyz, atol=1e-6) and (r2 == rgb).all()


def test_saved_scene_round_trip(tmp_path: Path) -> None:
    """A depth frame of a flat plate at 0.5 m below a downward camera comes back at PLATE."""
    import cv2

    K = np.array([[300.0, 0, 80], [0, 300.0, 60], [0, 0, 1]])
    T = look_at(np.array([0.0, 0.0, PLATE + 0.5]), [0.0, 0.0001, PLATE])
    depth = np.full((120, 160), 0.5, np.float32)
    np.save(tmp_path / "cam0_depth_0.npy", depth)
    cv2.imwrite(str(tmp_path / "cam0_color_0.png"), np.full((120, 160, 3), 128, np.uint8))
    cam = {
        "name": "Top",
        "K": K.tolist(),
        "dist": [0] * 5,
        "size": [160, 120],
        "T_world_cam": T.tolist(),
        "trusted": True,
        "why": "test",
        "color": ["cam0_color_0.png"],
        "depth": ["cam0_depth_0.npy"],
    }
    (tmp_path / "scene.json").write_text(json.dumps({"cams": [cam]}))
    s = load_scene(tmp_path, outlier_std=0)
    assert len(s.clouds) == 1 and len(s.views) == 1 and s.views[0].trusted
    assert np.allclose(s.cloud()[:, 2], PLATE, atol=1e-4)


def test_coarse_search_and_cloud_fit_alone() -> None:
    xyz = scene_cloud(hide_far_end=False)
    L, W, H = HALF
    r = coarse_search(xyz, L, W, H, instances=2)
    assert r["obj"] is not None and len(r["boards"]) >= 1
    h = (xyz - r["c"]) @ r["n"]
    cf = cloud_fit(
        xyz, h, np.array([*r["obj"]["centre_xy"], r["obj"]["yaw"]]), 2 * L, W, H, n_boot=5
    )
    assert np.abs(cf["pose"][:2] - TRUE[:2]).max() < 0.001
    assert min(cf["coverage"].values()) > 0.5  # all four sides seen
