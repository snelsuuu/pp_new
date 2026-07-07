"""Path-quality metrics and closed-loop lap simulation."""
import math

import numpy as np
from scipy.spatial import cKDTree


def path_metrics(path_xy, center, name: str = ""):
    """path_xy: (N,2) planned or driven points. center: (M,2) true centerline."""
    P = np.asarray(path_xy)
    if len(P) < 3:
        return dict(name=name, n=len(P), cte_mean=np.nan, cte_max=np.nan,
                    heading_rough=np.nan, zigzag=np.nan)
    tree = cKDTree(center)
    d, _ = tree.query(P)
    seg = np.diff(P, axis=0)
    head = np.arctan2(seg[:, 1], seg[:, 0])
    dh = np.abs(np.arctan2(np.sin(np.diff(head)), np.cos(np.diff(head))))
    return dict(name=name, n=len(P),
                cte_mean=float(d.mean()), cte_max=float(d.max()),
                heading_rough=float(np.degrees(dh.mean())),
                zigzag=float(np.degrees(dh.max())))


def simulate_lap(track, planner, step: float = 1.0, lookahead: float = 2.5,
                 max_steps: int = 600):
    """Kinematic point-follower closed loop. Returns (trajectory, status)."""
    cones = {"blue": track['blue'], "yellow": track['yellow'],
             "orange": np.zeros((0, 2)), "big": track['big']}
    x, y, yaw = track['start_pose']
    start = np.array([x, y])
    traj = [(x, y)]
    left_start = False
    tree_c = cKDTree(track['center'])

    for t in range(max_steps):
        pp, *_ = planner((x, y, yaw), cones)
        if not pp:
            return np.asarray(traj), f"STUCK@{t} (no path)"
        target = None
        for p in pp:
            if np.hypot(p[0] - x, p[1] - y) >= lookahead:
                target = p
                break
        if target is None:
            target = pp[-1]

        des = math.atan2(target[1] - y, target[0] - x)
        dyaw = math.atan2(math.sin(des - yaw), math.cos(des - yaw))
        yaw += np.clip(dyaw, -0.35, 0.35)
        x += step * math.cos(yaw)
        y += step * math.sin(yaw)
        traj.append((x, y))

        d_off, _ = tree_c.query([x, y])
        if d_off > 2.5:
            return np.asarray(traj), f"OFF-TRACK@{t} (cte={d_off:.1f}m)"

        d_start = np.hypot(x - start[0], y - start[1])
        if d_start > 15:
            left_start = True
        if left_start and d_start < 4:
            return np.asarray(traj), f"LAP DONE in {t} steps"
    return np.asarray(traj), "TIMEOUT"
