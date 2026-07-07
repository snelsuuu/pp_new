"""Reproduce the degradation stress benchmark.

Usage:  python -m sim.benchmark  (from repo root)
Writes benchmark_results.csv and prints summary tables.
"""
import time

import numpy as np
import pandas as pd

from planner import baseline_plan, improved_plan, improved_plan_v2
from sim.track_gen import make_track, degrade
from sim.metrics import simulate_lap, path_metrics

SCENARIOS = {
    "clean":      dict(),
    "noise_0.2":  dict(pos_noise=0.2),
    "noise_0.4":  dict(pos_noise=0.4),
    "drop_15":    dict(drop_rate=0.15),
    "drop_30":    dict(drop_rate=0.30),
    "flip_5":     dict(color_flip=0.05),
    "flip_10":    dict(color_flip=0.10),
    "phantom_10": dict(phantom_rate=0.10),
    "combo_mild": dict(pos_noise=0.2, drop_rate=0.10, color_flip=0.03),
    "combo_hard": dict(pos_noise=0.35, drop_rate=0.20, color_flip=0.07,
                       phantom_rate=0.10),
}

PLANNERS = [("baseline", baseline_plan),
            ("v1", improved_plan),
            ("v2", improved_plan_v2)]


def timing_bench(n_iter: int = 200):
    tk = make_track(seed=3)
    dtk = degrade(tk, seed=103, pos_noise=0.2, drop_rate=0.10, color_flip=0.03)
    cones = {"blue": dtk['blue'], "yellow": dtk['yellow'],
             "orange": np.zeros((0, 2)), "big": dtk['big']}
    for pname, pl in PLANNERS:
        ts = []
        for _ in range(n_iter):
            t0 = time.perf_counter()
            pl(dtk['start_pose'], cones)
            ts.append(time.perf_counter() - t0)
        ts = np.asarray(ts) * 1000
        print(f"{pname:>9}: {ts.mean():.2f} ms mean | "
              f"{np.percentile(ts, 95):.2f} ms p95")


def stress_bench(n_seeds: int = 15, out_csv: str = "benchmark_results.csv"):
    rows = []
    for sc_name, kw in SCENARIOS.items():
        for seed in range(n_seeds):
            tk = make_track(seed=seed)
            dtk = degrade(tk, seed=100 + seed, **kw)
            for pname, pl in PLANNERS:
                traj, status = simulate_lap(dtk, pl)
                m = path_metrics(traj, dtk['center'])
                rows.append(dict(scenario=sc_name, planner=pname, seed=seed,
                                 ok=('DONE' in status), cte_max=m['cte_max'],
                                 status=status))
    df = pd.DataFrame(rows)
    print("\nSUCCESS RATE")
    print(df.pivot_table(index='scenario', columns='planner', values='ok',
                         aggfunc='mean').round(2)
          [[p for p, _ in PLANNERS]].to_string())
    fails = df[~df.ok].copy()
    if len(fails):
        fails['mode'] = np.where(fails.status.str.contains('OFF'),
                                 'off-track', 'stuck')
        print("\nFAILURE MODES")
        print(fails.pivot_table(index='planner', columns='mode', values='seed',
                                aggfunc='count', fill_value=0).to_string())
    print("\nOVERALL:", df.groupby('planner').ok.mean().round(3).to_dict())
    df.to_csv(out_csv, index=False)
    print(f"\nsaved {out_csv}")


if __name__ == "__main__":
    timing_bench()
    stress_bench()
