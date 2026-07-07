"""Baseline planner: direct port of the ROS2 sector-FOV midpoint planner."""
import math
from typing import List, Tuple

import numpy as np

from .geometry import fov_filter, build_edges


def baseline_plan(car, cones_by_class, min_spacing: float = 1.0,
                  min_car_dist: float = 2.0, max_hop: float = 7.0):
    """Returns (path, fov_points, fov_classes, kept_candidates)."""
    pts, cls = fov_filter(car, cones_by_class)
    if len(pts) < 2:
        return [], pts, cls, []
    edges = build_edges(pts, cls)

    cands: List[Tuple[float, float]] = []
    for i, j in edges:
        if cls[i] == "big" and cls[j] == "big":
            continue
        cands.append(((pts[i][0] + pts[j][0]) / 2, (pts[i][1] + pts[j][1]) / 2))

    bigs = [pts[i] for i, c in enumerate(cls) if c == "big"]
    if len(bigs) >= 2:
        cands.append(tuple(np.asarray(bigs).mean(axis=0)))

    # farthest-first spacing filter
    car_p = np.array(car[:2])
    kept: List[np.ndarray] = []
    if cands:
        C = np.asarray(cands)
        for idx in np.argsort(-np.sum((C - car_p) ** 2, axis=1)):
            p = C[idx]
            if np.linalg.norm(p - car_p) < min_car_dist:
                continue
            if kept and min(np.linalg.norm(p - k) for k in kept) < min_spacing:
                continue
            kept.append(p)

    # greedy nearest-neighbour chain
    path, cur, used = [], car_p, [False] * len(kept)
    K = np.asarray(kept) if kept else np.zeros((0, 2))
    for _ in range(len(kept)):
        d = np.linalg.norm(K - cur, axis=1)
        d[used] = np.inf
        d[d > max_hop] = np.inf
        i = int(np.argmin(d))
        if not np.isfinite(d[i]):
            break
        used[i] = True
        path.append(tuple(K[i]))
        cur = K[i]
    return path, pts, cls, kept
