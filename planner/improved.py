"""Improved planners.

v1: sparse-map robustness
    - adaptive edge length (relaxed retry when the 6 m graph yields nothing)
    - single-side virtual candidates when the opposite boundary is missing
    - relaxed-hop fallback for the greedy chain

v2: v1 candidates + heading-aware cost chain
    - hop cost = distance + w_turn * |heading change|
    - hops turning more than max_turn_deg are rejected outright
"""
import math
from typing import List, Tuple

import numpy as np

from .geometry import fov_filter, build_edges


def _virtual_candidates(pts, cls, car, half_width: float = 1.75,
                        partner_radius: float = 5.0):
    """For boundary cones with no opposite-colour partner nearby, emit a
    candidate offset half a track width toward the track interior."""
    P = np.asarray(pts)
    out: List[Tuple[float, float]] = []
    car_p = np.array(car[:2])
    for i, ci in enumerate(cls):
        if ci not in ("blue", "yellow"):
            continue
        opp = "yellow" if ci == "blue" else "blue"
        has_partner = any(
            cls[j] == opp and np.linalg.norm(P[i] - P[j]) < partner_radius
            for j in range(len(cls)))
        if has_partner:
            continue
        same = [j for j in range(len(cls)) if cls[j] == ci and j != i]
        if not same:
            continue
        j = min(same, key=lambda j: np.linalg.norm(P[i] - P[j]))
        t = P[j] - P[i]
        n = np.linalg.norm(t)
        if n < 1e-6:
            continue
        t /= n
        normal = np.array([-t[1], t[0]])
        if np.dot(normal, car_p - P[i]) < 0:
            normal = -normal
        out.append(tuple(P[i] + half_width * normal))
    return out


def improved_plan(car, cones_by_class, min_spacing: float = 1.0,
                  min_car_dist: float = 2.0, max_hop: float = 7.0,
                  base_max_len: float = 6.0, relaxed_max_len: float = 12.0,
                  half_width: float = 1.75):
    """v1. Returns (path, fov_points, fov_classes, kept_candidates)."""
    pts, cls = fov_filter(car, cones_by_class)
    if len(pts) < 1:
        return [], pts, cls, []

    def edge_candidates(max_len):
        edges = build_edges(pts, cls, max_len=max_len)
        cands = []
        for i, j in edges:
            if cls[i] == "big" and cls[j] == "big":
                continue
            cands.append(((pts[i][0] + pts[j][0]) / 2,
                          (pts[i][1] + pts[j][1]) / 2))
        return cands

    cands = edge_candidates(base_max_len)
    if not cands:
        cands = edge_candidates(relaxed_max_len)

    cands += _virtual_candidates(pts, cls, car, half_width=half_width)

    bigs = [pts[i] for i, c in enumerate(cls) if c == "big"]
    if len(bigs) >= 2:
        cands.append(tuple(np.asarray(bigs).mean(axis=0)))

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

    def chain(max_hop_):
        path, cur, used = [], car_p, [False] * len(kept)
        K = np.asarray(kept) if kept else np.zeros((0, 2))
        for _ in range(len(kept)):
            d = np.linalg.norm(K - cur, axis=1)
            d[used] = np.inf
            d[d > max_hop_] = np.inf
            i = int(np.argmin(d)) if len(d) else 0
            if not len(d) or not np.isfinite(d[i]):
                break
            used[i] = True
            path.append(tuple(K[i]))
            cur = K[i]
        return path

    path = chain(max_hop)
    if not path:
        path = chain(max_hop * 1.8)
    return path, pts, cls, kept


def improved_plan_v2(car, cones_by_class, min_spacing: float = 1.0,
                     min_car_dist: float = 2.0, max_hop: float = 7.0,
                     base_max_len: float = 6.0, relaxed_max_len: float = 12.0,
                     half_width: float = 1.75, w_turn: float = 3.0,
                     max_turn_deg: float = 75.0):
    """v2: v1 candidate generation + heading-aware cost chain."""
    _, pts, cls, kept = improved_plan(
        car, cones_by_class, min_spacing=min_spacing,
        min_car_dist=min_car_dist, max_hop=max_hop,
        base_max_len=base_max_len, relaxed_max_len=relaxed_max_len,
        half_width=half_width)
    car_p = np.array(car[:2])
    max_turn = math.radians(max_turn_deg)

    def chain(max_hop_, heading0):
        path, cur, heading = [], car_p.copy(), heading0
        used = [False] * len(kept)
        K = np.asarray(kept) if kept else np.zeros((0, 2))
        for _ in range(len(kept)):
            best, best_cost = -1, np.inf
            for i in range(len(kept)):
                if used[i]:
                    continue
                v = K[i] - cur
                d = np.linalg.norm(v)
                if d > max_hop_ or d < 1e-6:
                    continue
                ang = math.atan2(v[1], v[0])
                dturn = abs(math.atan2(math.sin(ang - heading),
                                       math.cos(ang - heading)))
                if dturn > max_turn:
                    continue
                cost = d + w_turn * dturn
                if cost < best_cost:
                    best_cost, best = cost, i
            if best < 0:
                break
            v = K[best] - cur
            heading = math.atan2(v[1], v[0])
            used[best] = True
            path.append(tuple(K[best]))
            cur = K[best]
        return path

    path = chain(max_hop, car[2])
    if not path:
        path = chain(max_hop * 1.8, car[2])
    return path, pts, cls, kept
