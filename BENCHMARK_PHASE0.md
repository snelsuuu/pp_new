# Phase 0 — Baseline Benchmark Checklist

Goal: capture hard numbers on the CURRENT planner before changing anything, so every
later improvement has before/after evidence. Target: complete within 2 weeks.

## Step 1 — Fix measurement-corrupting bug first
- [ ] Branch off (e.g. `fix/tick-race`) and remove the second `with self.data_lock:`
      write-back block in `_tick`; pass cone list copies into `_compute_fov_points`
      as arguments instead of reading `self.*_global` inside it.
- [ ] Wrap `Delaunay(pts)` in try/except (QhullError) falling back to the k-NN path,
      so benchmark runs don't crash on collinear cones.
- [ ] Merge; this fixed version IS the baseline.

## Step 2 — Choose test tracks (3–4 total)
- [ ] Track A: hairpin-heavy (this is where greedy NN fails — must be included)
- [ ] Track B: slalom / chicane section
- [ ] Track C: mixed layout (closest to competition track style)
- [ ] (Optional) Track D: wide-track sections (stresses the 6 m edge cap)
- [ ] Record track names/seeds in PROJECT.md §6.

## Step 3 — Metrics to log per run
For each run, record:
- [ ] **Completion:** did the car finish the lap without leaving track / hitting cones? (y/n)
- [ ] **Lap time** (when completed)
- [ ] **Doubling-back events:** count of consecutive path hops with heading change > 90°
- [ ] **Min path-to-cone distance** over the lap (proxy for how close the path skirts boundaries)
- [ ] **Path jitter:** mean lateral displacement of the path between consecutive ticks,
      measured 5 m ahead of the car (proxy for how unstable the controller input is)
- [ ] **Planner tick compute time** (ms, mean + max) — cheap to log, useful later

## Step 4 — Tooling to build (small scripts, keep in repo under `tools/bench/`)
- [ ] `record_run.sh` — launches sim + stack on a given track, records
      `ros2 bag` of /slam/odom, /slam/map_cones, /path_points, plus ground-truth topics
      the sim exposes.
- [ ] `analyze_bag.py` — reads a bag, computes the Step 3 metrics, appends a row to
      `results.csv` (columns: date, git hash, track, planner_variant, run_idx, metrics...).
- [ ] Add `planner_variant` ROS param now (values: `greedy` today, `triwalk` later)
      and log it into results.

## Step 5 — Record the baseline
- [ ] ≥ 10 runs per track with the (bug-fixed) greedy planner.
- [ ] Commit `results.csv` + a few representative bags (or store bags outside git,
      link them).
- [ ] Write a 5-line summary in PROJECT.md session log: completion rate per track,
      mean lap time, doubling-back count on the hairpin track.

## Exit criteria for Phase 0
- Baseline results table exists for all tracks.
- At least one recorded video/bag clearly showing the greedy hairpin failure
  (this becomes the "before" in your final demo).
- PROJECT.md §5/§6 fully filled in.

## Bring to the next session
- results.csv (or whatever partial numbers you have)
- Any script that misbehaved + its error output
- The hairpin failure bag/video observations
