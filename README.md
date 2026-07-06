# Path Planning Upgrade — Formula Bharat Driverless Cup

> **How to use this file:** Paste the ENTIRE file at the start of every Claude session,
> along with any code/errors relevant to the day. At the end of each session, ask Claude
> to write the day's log entry, paste it into the Session Log below, and commit.

---

## 1. Project Goal (one paragraph)

Replace the greedy nearest-neighbor path ordering in our SLAM-based cone planner
(`slam_path_sector_visualiser.py`) with a **Delaunay triangle-walk planner** that emits
topologically ordered centerline midpoints, then add a **spline smoothing + velocity
profiling stage** so the stack outputs a full drivable trajectory (position, heading,
speed) instead of raw waypoints. Success = measurably higher track-completion rate on
hairpin/slalom layouts in EUFS sim and a lap-time improvement from curvature-based
speed targets, backed by before/after benchmark numbers.

## 2. Current Stack Context

- ROS 2 node subscribes to `/slam/odom` (nav_msgs/Odometry) and `/slam/map_cones`
  (eufs_msgs/ConeArrayWithCovariance), publishes `/path_points` (nav_msgs/Path), 20 Hz tick.
- Pipeline today: sector FOV gate (30 m, 60°, vertex −5 m) → Delaunay → color/length
  edge filter (keep blue–yellow; big-orange special-cased) → edge midpoints →
  farthest-first 1 m spacing filter → greedy NN chain (7 m max hop) → Path msg.
- Known issues in current node (from initial review):
  - [ ] `_tick` write-back race: cone lists copied out under lock, then written back,
        can overwrite a fresher map from `cb_map`. **Fix before baseline.**
  - [ ] Greedy NN cuts hairpins / doubles back (the core thing this project replaces).
  - [ ] Farthest-first spacing filter suppresses good near-field candidates.
  - [ ] No QhullError guard on Delaunay (collinear cones on straights crash the tick).
  - [ ] Path stamped with now() instead of odom stamp; frame_id hardcoded "map";
        waypoints have identity orientation (no yaw).
  - [ ] Small-orange–orange edge midpoints can land on track boundary.

## 3. Semester Plan

| Phase | Weeks | Deliverable | Status |
|-------|-------|-------------|--------|
| 0. Baseline & metrics | 1–2 | Benchmark suite + baseline numbers on 3–4 sim tracks (rosbags committed) | ⬜ not started |
| 1. Triangle-walk planner | 3–6 | Ordered-midpoint planner via `tri.neighbors` walk, ROS-param switchable vs greedy; hairpin demo video | ⬜ |
| 2. Smoothing + velocity profile | 7–10 | Spline fit + arc-length resample + 3-pass velocity profile; trajectory published to controller | ⬜ |
| 3. Hardening & evaluation | 11–14 | Re-run benchmarks; noise/occlusion stress tests; graceful-degradation patches; final numbers | ⬜ |

**Explicitly out of scope this semester:** global racing-line optimizer, learned cone
matching, controller/MPC changes. (Listed as future work in report.)

## 4. Key Design Decisions

| # | Decision | Rationale | Date |
|---|----------|-----------|------|
| 1 | Triangle-walk over greedy NN | Triangulation adjacency gives track order for free; fixes hairpin cut-through | 2026-07-06 |
| 2 | Keep greedy planner behind a ROS parameter | A/B testing in sim + competition fallback | 2026-07-06 |
| 3 | Smoothing spline (`splprep`, smoothing > 0), 0.5 m arc-length resample | Midpoints are noisy; controller needs uniform spacing + curvature | 2026-07-06 |
| 4 | Velocity: v=√(a_lat/κ) cap + forward accel pass + backward braking pass, start a_lat ≈ 3–5 m/s² | Standard, tunable, conservative-first | 2026-07-06 |

## 5. Current State

- **Phase:** 0 (Baseline & metrics)
- **This week's milestone:** benchmark scripts written; baseline rosbags recorded on all
  test tracks; `_tick` race bug fixed on a branch.
- **Blockers:** none yet.
- **Next session's goal:** _(fill in at end of each session)_

## 6. Environment Notes

- Simulator: EUFS sim (running ✅)
- ROS 2 distro: _(fill in)_
- Test tracks chosen: _(fill in — need ≥1 hairpin-heavy, ≥1 slalom, 1 mixed)_
- Controller consuming /path_points: _(fill in — pure pursuit / Stanley / MPC?)_
- Repo / branch for this project: _(fill in)_

## 7. Session Log

_(Newest first. One short entry per session: what was done, what was decided, what broke.)_

### 2026-07-06 — Day 1 (offline prototype, cells 1–3)
- Built offline harness: Cell 1 (fake tracks), Cell 2 (Delaunay + blue-yellow
  filter + midpoints), Cell 3 (greedy NN vs triangle-walk, side by side).
- **Key finding:** greedy's failure depends on the ratio of MAX_HOP (7 m) to
  hairpin leg spacing. On the WIDE default hairpin (legs ~9 m > 7 m hop), greedy
  does NOT cut across — the hop leash saves it, so both planners look identical.
  On a TIGHT hairpin (radius 2.5, legs ~5 m < 7 m hop), greedy clearly jumps the
  infield and skips the top bend. Lesson: our benchmark tracks MUST include a
  hairpin with leg spacing < 7 m or we won't reproduce the real failure.
- **Open bug:** triangle-walk output looks too short / barely visible on the
  tight hairpin — termination logic likely stops early when midpoints are dense.
  Investigate next session (make triangle-walk actually complete the loop).
- **Next session goal:** debug triangle-walk termination on tight hairpin; get it
  to trace the full centerline including the top bend, then overlay cleanly vs greedy.

### 2026-07-06 — Day 0 (kickoff)
- Reviewed current planner node end-to-end; identified issues listed in §2.
- Chose project scope (triangle-walk + trajectory layer) and semester plan (§3).
- Created PROJECT.md and Phase 0 benchmark checklist.
- **Homework:** fill in §6 blanks; fix `_tick` race on a branch; record baseline
  rosbags per BENCHMARK_PHASE0.md.
