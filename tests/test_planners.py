"""Smoke tests: planners produce sane output on clean and degraded maps.

Run from repo root:  python -m pytest tests/ -q
"""
import numpy as np

from planner import baseline_plan, improved_plan, improved_plan_v2
from sim.track_gen import make_track, degrade
from sim.metrics import simulate_lap, path_metrics

PLANNERS = [baseline_plan, improved_plan, improved_plan_v2]


def _cones(tk):
    return {"blue": tk['blue'], "yellow": tk['yellow'],
            "orange": np.zeros((0, 2)), "big": tk['big']}


def test_clean_snapshot_all_planners():
    tk = make_track(seed=42)
    for pl in PLANNERS:
        path, pts, cls, kept = pl(tk['start_pose'], _cones(tk))
        assert len(path) > 0, f"{pl.__name__} produced empty path on clean map"
        m = path_metrics(np.vstack([tk['start_pose'][:2], path]), tk['center'])
        assert m['cte_max'] < 1.5


def test_clean_lap_v2():
    tk = make_track(seed=0)
    traj, status = simulate_lap(tk, improved_plan_v2)
    assert 'DONE' in status, f"v2 failed clean lap: {status}"


def test_v2_survives_known_baseline_stuck_frame():
    tk = make_track(seed=1)
    dtk = degrade(tk, seed=101, drop_rate=0.15)
    base_path, *_ = baseline_plan(dtk['start_pose'], _cones(dtk))
    v2_path, *_ = improved_plan_v2(dtk['start_pose'], _cones(dtk))
    assert len(base_path) == 0, "expected baseline to be stuck here"
    assert len(v2_path) > 0, "v2 should recover the stuck frame"


def test_degrade_empty_edge_cases():
    tk = make_track(seed=2)
    # extreme rates must not crash (regression test for _safe_vstack)
    dtk = degrade(tk, seed=5, drop_rate=0.99, color_flip=0.99,
                  phantom_rate=0.5, pos_noise=1.0)
    assert dtk['blue'].shape[1] == 2
    assert dtk['yellow'].shape[1] == 2
