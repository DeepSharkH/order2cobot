"""The protoboard's holes from its CAD: labels, connected nets, 3D positions, and the discrete
placements a casing's pins can take.

The model (configs/objects.yaml `protoboard.model`) is one half breadboard; `instances` copies
end to end along u make the board, as in agents/vision/icp.object_footprint. Each hole is a
square funnel: 1.7 mm wide at the top (z = 9.5 mm), 0.7 mm from z = 9.0 mm down to its floor at
z = 2.5 mm. Its centre is the mean of its four corners at z = 9.0 mm.

Board frame: centred on the box (as `locate_object` returns it), u along the long side, v
across, z up from the box base, metres. Geometry does not depend on the labels; the labels do
depend on the board's orientation, which the box cannot tell (it is symmetric up to the CAD's
0.67 mm hole offset):

  flip_u    number 1 (and half H1) at the +u end instead of the -u end
  flip_v    row A on the +v side instead of the -v side
  rail_plus which rail row of each side is "+" (red line): "outer" or "inner"

Labels: `H<half>:<row><number>` for terminal holes (rows A-E | F-J, numbers 1-30 per half, as
printed on each half) and `H<half>:<+|-><L|R><k>` for rail holes (L = the A side, k = 1-25 from
the number-1 end). Nets: the 5 holes A-E (or F-J) of one number, or one rail row of one half.
"""

import itertools
from dataclasses import dataclass
from functools import lru_cache
from typing import Any

import numpy as np
import numpy.typing as npt
from scipy.cluster.hierarchy import fcluster, linkage
from scipy.spatial import cKDTree

from agents.vision.icp import model_axes

Array = npt.NDArray[Any]
PITCH = 0.00254
TOP_Z = 0.0095  # board top: hole entry (CAD)
FLOOR_Z = 0.0025  # hole floor (CAD)
ENTRY = 0.0017  # hole width at the top (funnel); 0.7 mm from 0.5 mm down
ROWS = "ABCDEFGHIJ"


@dataclass(frozen=True)
class Hole:
    name: str  # "H1:A12", "H2:+L7"
    half: int
    kind: str  # "terminal" | "rail"
    row: str  # "A".."J" | "+L" "-L" "+R" "-R"
    num: int  # printed number 1-30; rail: 1-25 from the number-1 end
    net: str  # holes connected to each other share it
    uv: tuple[float, float]  # board frame, metres


@lru_cache(maxsize=4)
def cad_holes(step: str) -> Array:
    """Hole centres (u, v) of one model instance in model axes (metres), from the STEP."""
    from agents.vision.locate import read_step

    V = model_axes(read_step(step))
    P = V[np.abs(V[:, 2] - 0.009) < 1e-5, :2]
    if len(P) < 4:
        raise ValueError(f"{step}: no hole corners at z = 9.0 mm")
    lab = fcluster(linkage(P, "single"), 0.0012, "distance")
    return np.array([P[lab == k].mean(0) for k in np.unique(lab)])


def _levels(x: Array, tol: float = 1e-5) -> list[float]:
    out: list[float] = []
    for value in np.sort(x):
        if not out or value - out[-1] > tol:
            out.append(float(value))
    return out


class BoardGrid:
    """All holes of the board, labelled for one orientation."""

    def __init__(
        self,
        step: str,
        length: float,
        instances: int = 2,
        flip_u: bool = False,
        flip_v: bool = False,
        rail_plus: str = "outer",
    ) -> None:
        if rail_plus not in ("outer", "inner"):
            raise ValueError("rail_plus must be 'outer' or 'inner'")
        self.orientation = {"flip_u": flip_u, "flip_v": flip_v, "rail_plus": rail_plus}
        self._args = (step, length, instances)
        one = cad_holes(step)
        vs = _levels(one[:, 1])
        mid = np.mean(vs)
        rail_v = sorted(vs, key=lambda v: -abs(v - mid))[:4]  # the 4 outermost rows
        term_v = [v for v in vs if v not in rail_v]
        if len(term_v) != 10:
            raise ValueError(f"expected 10 terminal rows, found {len(term_v)}")
        su = -1.0 if flip_u else 1.0
        sv = -1.0 if flip_v else 1.0
        holes: list[Hole] = []
        for i in range(instances):
            off = (i - (instances - 1) / 2) * length
            half = instances - i if flip_u else i + 1
            uv = one + [off, 0.0]
            # terminal rows in label order: A first = the -v side unless flipped
            for r, v in enumerate(sorted(term_v, key=lambda v: sv * v)):
                pts = uv[np.abs(uv[:, 1] - v) < 1e-5]
                pts = pts[np.argsort(su * pts[:, 0])]
                for c, p in enumerate(pts):
                    row = ROWS[r]
                    strip = "A-E" if r < 5 else "F-J"
                    holes.append(
                        Hole(
                            f"H{half}:{row}{c + 1}",
                            half,
                            "terminal",
                            row,
                            c + 1,
                            f"H{half}:{c + 1}{strip}",
                            (float(p[0]), float(p[1])),
                        )
                    )
            for v in rail_v:
                side = "L" if sv * (v - mid) < 0 else "R"  # L = the A side
                outer = all(
                    abs(v - mid) >= abs(w - mid) for w in rail_v if (w - mid) * (v - mid) > 0
                )
                sign = "+" if outer == (rail_plus == "outer") else "-"
                pts = uv[np.abs(uv[:, 1] - v) < 1e-5]
                pts = pts[np.argsort(su * pts[:, 0])]
                for k, p in enumerate(pts):
                    row = f"{sign}{side}"
                    holes.append(
                        Hole(
                            f"H{half}:{row}{k + 1}",
                            half,
                            "rail",
                            row,
                            k + 1,
                            f"H{half}:{row}",
                            (float(p[0]), float(p[1])),
                        )
                    )
        self.holes = holes
        self.uv = np.array([h.uv for h in holes])
        self.index = {h.name: k for k, h in enumerate(holes)}
        self._tree = cKDTree(self.uv)

    @classmethod
    def from_spec(cls, spec: Any, instances: int | None = None, **orientation: Any) -> "BoardGrid":
        """From an `ObjectSpec` (configs/objects.yaml) whose `model` is the half-board STEP."""
        if not spec.model:
            raise ValueError(f"{spec.name}: no CAD model, so no holes")
        return cls(spec.model, spec.size[0], instances or spec.instances or 1, **orientation)

    @classmethod
    def from_holes_json(
        cls, path: str, flip_u: bool = False, flip_v: bool = False, numbers: int | None = None
    ) -> "BoardGrid":
        """A one-piece board from a hole table (assets/models/cad/breadboard.py output, e.g.
        dashboard/web/models/cad/breadboard.json: 'a1'..'j60', 'L+1'..'R-50', mm, x across
        a -> j, y towards number 1). One "half" (H1), numbers as printed, centred on its holes.
        `numbers` re-lays the terminal strips for a board with another count of numbers (the
        830-point MB-102 has 63) from the table's rows a-j and pitch; its rails are left out
        (their grouping depends on the board)."""
        import json

        raw = json.loads(open(path).read())["holes"]
        if numbers is not None:
            a1 = np.array(raw["a1"])
            step = np.array(raw["a2"]) - a1
            raw = {
                f"{r}{k}": (np.array(raw[f"{r}1"]) + (k - 1) * step).tolist()
                for r in "abcdefghij"
                for k in range(1, numbers + 1)
            }
        su = -1.0 if flip_u else 1.0
        sv = -1.0 if flip_v else 1.0
        xy = np.array(list(raw.values()))[:, :2] / 1000
        mid = (xy.min(0) + xy.max(0)) / 2
        holes: list[Hole] = []
        for name, (x, y, _) in raw.items():
            uv = (su * -(y / 1000 - mid[1]), sv * (x / 1000 - mid[0]))
            if name[0] in "LR":
                row, k = name[1] + name[0], int(name[2:])
                holes.append(Hole(f"H1:{row}{k}", 1, "rail", row, k, f"H1:{row}", uv))
            else:
                row, k = name[0].upper(), int(name[1:])
                strip = "A-E" if row <= "E" else "F-J"
                holes.append(Hole(f"H1:{row}{k}", 1, "terminal", row, k, f"H1:{k}{strip}", uv))
        g = cls.__new__(cls)
        g.orientation = {"flip_u": flip_u, "flip_v": flip_v, "rail_plus": "table"}
        g._args = ("json", path, numbers)
        g.holes = holes
        g.uv = np.array([h.uv for h in holes])
        g.index = {h.name: k for k, h in enumerate(holes)}
        g._tree = cKDTree(g.uv)
        return g

    def __getitem__(self, name: str) -> Hole:
        return self.holes[self.index[name]]

    def relabel(self, **orientation: Any) -> "BoardGrid":
        """The same holes labelled for another orientation (geometry unchanged)."""
        if self._args[0] == "json":
            o = {**self.orientation, **orientation}
            return BoardGrid.from_holes_json(self._args[1], o["flip_u"], o["flip_v"], self._args[2])
        return BoardGrid(*self._args, **{**self.orientation, **orientation})

    # ── geometry ─────────────────────────────────────────────────────────────

    def snap(self, uv: Array, max_d: float = PITCH / 2) -> list[tuple[Hole | None, float]]:
        """Nearest hole of each board-frame point and its distance; None beyond `max_d`."""
        d, k = self._tree.query(np.atleast_2d(uv))
        return [
            (self.holes[j] if dj <= max_d else None, float(dj)) for dj, j in zip(d, k, strict=True)
        ]

    def placements(
        self,
        exit_x: Any,
        tol: float = 0.0003,
        occupied: Any = (),
        same_net: bool = False,
        ordered: bool = False,
    ) -> list[dict[str, Any]]:
        """Every discrete way a casing with exits at `exit_x` (metres along its axis, as in
        casings.json) fits the holes: each exit within `tol` of a hole, the casing axis along
        +-u or +-v. Placements on an `occupied` hole name are left out, and so are those whose
        pins share a net (they short the part) unless `same_net`. `ordered` keeps both pin
        orders of a symmetric part (needed for polarised parts)."""
        ex = np.asarray(exit_x, float)
        ex = ex - ex.mean()
        busy = set(occupied)
        out, seen = [], set()
        for k0 in range(len(self.holes)):
            for axis in ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)):
                a = np.array(axis)
                pts = self.uv[k0] + np.outer(ex - ex[0], a)
                d, idx = self._tree.query(pts)
                if d.max() > tol:
                    continue
                hs = [self.holes[j] for j in idx]
                names = tuple(h.name for h in hs)
                if len(set(names)) < len(names) or busy & set(names):
                    continue
                nets = [h.net for h in hs]
                if not same_net and len(set(nets)) < len(nets):
                    continue
                key = names if ordered else tuple(sorted(names))
                if key in seen:
                    continue
                seen.add(key)
                out.append(
                    {
                        "holes": list(names),
                        "axis_uv": [float(a[0]), float(a[1])],
                        "centre_uv": (self.uv[idx].mean(0)).tolist(),
                        "residual": float(d.max()),
                        "nets": nets,
                    }
                )
        return out

    def place(self, origin: Any, u_axis: Any, normal: Any, z: float = TOP_Z) -> Array:
        """3D points of all holes at height `z` over the box base, for a board whose base centre
        is `origin`, long axis `u_axis` and normal `normal` (any frame: world or base)."""
        n = np.asarray(normal, float)
        n /= np.linalg.norm(n)
        u = np.asarray(u_axis, float) - n * (np.asarray(u_axis, float) @ n)
        u /= np.linalg.norm(u)
        v = np.cross(n, u)
        return (
            np.asarray(origin, float)
            + np.outer(self.uv[:, 0], u)
            + np.outer(self.uv[:, 1], v)
            + z * n
        )

    # ── fit to detected holes ────────────────────────────────────────────────

    def fit(
        self,
        uv_seen: Any,
        max_d: float = 0.0009,
        shifts_u: int = 3,
        shifts_v: int = 2,
        iters: int = 8,
        turns: tuple[float, ...] = (0.0, np.pi),
    ) -> dict[str, Any]:
        """Rigid 2D correction (du, dv, dyaw) of the grid onto hole centres detected in the board
        frame (e.g. from a gripper-cam close-up, placed with the box pose). The initial pose is
        only good to a few mm, so whole-pitch shifts and the 180° turn are tried; the grid's
        gaps (centre channel, rail groups, board ends, the half-board joint) tell them apart.

        The hole pattern is symmetric under a half turn (about a point 0.67 mm off the box
        centre per CAD), so a turned fit puts every hole at the same place: positions are fixed,
        labels are not. `turned` says the labels of the fit are those of the box turned half a
        turn (`relabel(flip_u=not, flip_v=not)`); only a printed number can decide. Once it has,
        `turns=(0.0,)` keeps the labels of this grid's orientation (the turned twin, 0.67 mm
        away, would otherwise count as the runner-up).
        Returns the best hypothesis, its rms, matched share, and the best score of a hypothesis
        that puts the holes elsewhere: close to the best means the index is ambiguous (another
        view, or more of the board's gaps in it)."""
        P = np.atleast_2d(np.asarray(uv_seen, float))
        hyps = []
        for turn, su, sv in itertools.product(
            turns, range(-shifts_u, shifts_u + 1), range(-shifts_v, shifts_v + 1)
        ):
            T = np.array([su * PITCH, sv * PITCH, turn])
            for _ in range(iters):
                Q = _apply(P, T)  # seen points in the grid's frame
                d, k = self._tree.query(Q)
                m = d < max_d
                if m.sum() < 3:
                    break
                T = _compose(_kabsch(Q[m], self.uv[k[m]]), T)
            Q = _apply(P, T)
            d, _ = self._tree.query(Q)
            m = d < max_d
            rms = float(np.sqrt(np.mean(d[m] ** 2))) if m.any() else np.inf
            hyps.append((float(m.mean()) - rms / max_d * 0.1, T, rms, float(m.mean())))
        hyps.sort(key=lambda h: -h[0])
        best = hyps[0]
        placed = cKDTree(_apply(self.uv, _inverse(best[1])))  # the best grid in the seen frame
        second = next(
            (
                h
                for h in hyps[1:]
                if placed.query(_apply(self.uv, _inverse(h[1])))[0].max() > PITCH / 4
            ),
            None,
        )
        T = best[1]
        yaw = float((T[2] + np.pi) % (2 * np.pi) - np.pi)
        return {
            # maps seen points onto the grid; the grid sits at its inverse in the seen frame
            "seen_to_grid": {"du": float(T[0]), "dv": float(T[1]), "dyaw": yaw},
            "turned": abs(yaw) > np.pi / 2,
            "rms": best[2],
            "matched": best[3],
            "score": best[0],
            "runner_up_score": second[0] if second else None,
        }

    def place_fitted(self, fit: dict[str, Any]) -> Array:
        """Board-frame uv of every hole after a `fit` (in the frame the seen points were in)."""
        t = fit["seen_to_grid"]
        return _apply(self.uv, _inverse(np.array([t["du"], t["dv"], t["dyaw"]])))


def _rot(a: float) -> Array:
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def _apply(P: Array, T: Array) -> Array:
    return P @ _rot(T[2]).T + T[:2]


def _compose(A: Array, B: Array) -> Array:
    """T = A after B."""
    t = _rot(A[2]) @ B[:2] + A[:2]
    return np.array([t[0], t[1], A[2] + B[2]])


def _kabsch(Q: Array, G: Array) -> Array:
    """Rigid 2D transform taking Q onto G (least squares)."""
    qc, gc = Q.mean(0), G.mean(0)
    H = (Q - qc).T @ (G - gc)
    a = np.arctan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    t = gc - _rot(a) @ qc
    return np.array([t[0], t[1], a])


def _inverse(T: Array) -> Array:
    t = -_rot(-T[2]) @ T[:2]
    return np.array([t[0], t[1], -T[2]])


def box_frame(located: dict[str, Any]) -> tuple[Array, Array, Array]:
    """(base centre, u axis, normal) of a `locate_object` result's box, world frame."""
    c = np.array(located["box"]["corners"])  # bottom: -u-v, +u-v, +u+v, -u+v, then top
    u = c[1] - c[0]
    return c[:4].mean(0), u / np.linalg.norm(u), np.array([0.0, 0.0, 1.0])


__all__ = ["ENTRY", "FLOOR_Z", "PITCH", "TOP_Z", "BoardGrid", "Hole", "box_frame", "cad_holes"]
