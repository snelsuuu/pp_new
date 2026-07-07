"""Shared geometry: frame transforms, sector-FOV filter, constrained edge graph."""
import math
from typing import Dict, List, Tuple

import numpy as np

try:
    from scipy.spatial import Delaunay
    _HAS_SCIPY = True
except ImportError:  # pragma: no cover
    Delaunay = None
    _HAS_SCIPY = False


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def fov_filter(car, cones_by_class: Dict[str, np.ndarray],
               vertex_offset: float = -5.0, R: float = 30.0,
               fov_deg: float = 60.0) -> Tuple[List[Tuple[float, float]], List[str]]:
    """Select cones inside a circular sector anchored behind the car.

    car: (x, y, yaw). Returns (points, class_names) in world frame.
    """
    cx, cy, yaw = car
    half = math.radians(fov_deg / 2)
    c, s = math.cos(yaw), math.sin(yaw)
    pts, cls = [], []
    for name, arr in cones_by_class.items():
        for px, py in arr:
            dx, dy = px - cx, py - cy
            lx, ly = c * dx + s * dy, -s * dx + c * dy
            vx, vy = lx - vertex_offset, ly
            r = math.hypot(vx, vy)
            if r > R or r < 1e-3:
                continue
            if abs(math.atan2(vy, vx)) <= half:
                pts.append((px, py))
                cls.append(name)
    return pts, cls


def build_edges(pts, cls, max_len: float = 6.0):
    """Delaunay (or trivial) edges with class/length constraints.

    Drops: edges >= max_len, blue-blue, yellow-yellow, and any
    orange-like <-> blue/yellow edge.
    """
    N = len(pts)
    if N < 2:
        return []
    P = np.asarray(pts)
    es = set()
    if _HAS_SCIPY and N >= 3:
        for simplex in Delaunay(P).simplices:
            i, j, k = map(int, simplex)
            for a, b in ((i, j), (j, k), (k, i)):
                es.add((min(a, b), max(a, b)))
    else:
        k = min(3, N - 1)
        for i in range(N):
            d2 = np.sum((P - P[i]) ** 2, axis=1)
            d2[i] = np.inf
            for j in np.argsort(d2)[:k]:
                es.add((min(i, int(j)), max(i, int(j))))

    orange_like, bw = {"orange", "big"}, {"blue", "yellow"}
    out = []
    for i, j in es:
        if np.sum((P[i] - P[j]) ** 2) >= max_len ** 2:
            continue
        ci, cj = cls[i], cls[j]
        if ci == cj and ci in bw:
            continue
        if (ci in orange_like) != (cj in orange_like):
            continue
        out.append((i, j))
    return out
