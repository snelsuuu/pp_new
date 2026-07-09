# FS Path Planning — robustness under degraded SLAM maps

Improving a Delaunay-midpoint path planner for a Formula Student
driverless car. The original ROS2 node works well on clean cone maps but
fails under realistic SLAM degradation. This repo prototypes and
benchmarks improvements in pure Python, then ports the winner back to
ROS2.

## Method

1. **Baseline port** — the ROS2 planner (sector FOV → Delaunay edges →
   class/length constraints → edge midpoints → spacing filter → greedy
   nearest-neighbour chain) extracted into pure Python
   (`planner/baseline.py`).
2. **Simulation** — synthetic closed-loop tracks (`sim/track_gen.py`)
   with a SLAM degradation model: cone position noise, dropout, colour
   misclassification, phantom cones.
3. **Closed-loop benchmark** — a kinematic point-follower replans every
   1 m and drives full laps (`sim/metrics.py`). Success = lap completed
   without leaving the track.
4. **Improvements**, each validated against the benchmark:
   - **v1** (`improved_plan`): adaptive edge length with relaxed retry;
     single-side *virtual candidates* when the opposite boundary is
     missing; relaxed-hop chain fallback.
   - **v2** (`improved_plan_v2`): v1 candidates + heading-aware cost
     chain (cost = distance + 3·|Δheading|, hops turning > 75° rejected).

## Results (15 seeds per scenario, closed-loop laps)

Lap success rate:

| scenario    | baseline | v1   | v2   |
|-------------|----------|------|------|
| clean       | 1.00     | 1.00 | 1.00 |
| noise σ=0.2 | 1.00     | 1.00 | 1.00 |
| noise σ=0.4 | 0.93     | 0.93 | 1.00 |
| drop 15%    | 0.07     | 0.53 | 0.87 |
| drop 30%    | 0.00     | 0.13 | 0.40 |
| flip 5%     | 0.73     | 0.93 | 0.87 |
| flip 10%    | 0.60     | 0.73 | 0.87 |
| phantom 10% | 0.87     | 1.00 | 1.00 |
| combo mild  | 0.20     | 0.67 | 0.87 |
| combo hard  | 0.00     | 0.13 | 0.60 |
| **overall** | **0.54** | **0.71** | **0.85** |

Failure modes (counts over all 450 runs):

| planner  | stuck (fail-safe) | off-track (dangerous) |
|----------|-------------------|-----------------------|
| baseline | 68                | 1                     |
| v1       | 28                | 16                    |
| v2       | 12                | 11                    |

Planning time on a degraded frame (200 iterations):

| planner  | mean    | p95     |
|----------|---------|---------|
| baseline | 1.23 ms | 1.75 ms |
| v1       | 1.52 ms | 2.07 ms |
| v2       | 1.80 ms | 2.40 ms |

All planners comfortably fit a 20 Hz replanning budget.

### Key findings

- On clean maps the baseline is already fine (10/10 laps, mean
  cross-track error ≈ 0.20 m); improvements only matter under
  degradation.
- **Cone dropout is the dominant failure mode**, not colour flips:
  spacing 4 m with a hard 6 m edge limit means two adjacent dropped
  cones split the Delaunay graph and the planner returns an empty path.
- The baseline fails *safe* (stuck) almost exclusively; v1's extra
  candidates traded some stuck failures for dangerous off-track ones;
  v2's heading-aware chain recovered most of that safety while raising
  success further.

### Negative results (v3 experiments)

Two candidate-filtering ideas were benchmarked and rejected:

- **Neighbour-vote colour fix** (flip a cone whose two nearest cones are
  both the opposite colour): catastrophic under degradation
  (overall 0.85 → 0.59, clean laps broken). The geometric margin between
  nearest-opposite (median 3.50 m) and nearest-same (3.69 m) cone
  distances is only 0.19 m, so position noise makes the vote unreliable;
  a min-distance-to-cone veto additionally deleted true centerline
  candidates near phantom cones.
- **Conservative corridor check** (reject candidates whose nearest blue
  and yellow cones lie clearly on the same side): exactly zero effect —
  identical success rate, identical failure counts over 300 laps. The
  heading-aware chain already refuses the candidates this filter
  removes. Dropped to avoid dead code.

Likely next direction for the remaining off-track failures: temporal
smoothing (penalise frame-to-frame path jumps), since a single-frame
planner cannot detect that its path suddenly veered relative to the
previous frame.

## Repo layout

```
planner/     pure-python planners (no ROS imports)
sim/         track generator, degradation model, metrics, benchmark
notebooks/   exploration notebooks
tests/       pytest smoke tests
ros2_node/   ROS2 port of the winning planner (in progress)
```

## Reproduce

```bash
pip install numpy scipy pandas matplotlib pytest
python -m pytest tests/ -q          # smoke tests
python -m sim.benchmark             # full stress matrix (~10-20 min)
```

## Roadmap

- [x] Baseline port + closed-loop benchmark
- [x] v1: sparse-map robustness
- [x] v2: heading-aware cost chain  ← **shipped planner**
- [x] v3 experiments: colour fix + corridor filter (rejected, see negative results)
- [x] ROS2 node wrapping v2 (`ros2_node/slam_path_planner.py`)
- [ ] Temporal smoothing (v4 candidate)
- [ ] On-car / simulator validation

