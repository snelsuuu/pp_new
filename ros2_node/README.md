# ROS2 node

`slam_path_planner.py` — drop-in replacement for the original
`slam_path_sector_visualiser.py`. Same topics (/slam/odom,
/slam/map_cones -> /path_points), same QoS, same base parameters, but
the planning core is `planner.improved_plan_v2` (benchmarked: 54% -> 85%
lap completion under degraded maps, p95 plan time 2.4 ms at 20 Hz).

## Integration

The node imports the pure-python `planner` package, so it must be on
PYTHONPATH. Quickest options:

1. Copy the `planner/` folder next to the node inside your existing
   ROS2 package, or
2. `pip install -e .` from the repo root inside the same environment
   your ROS2 workspace uses (add a minimal setup.py first), or
3. Add the repo root to PYTHONPATH in your launch environment.

Extra parameters vs the original node:

    planner.base_max_len_m     (6.0)   normal Delaunay edge limit
    planner.relaxed_max_len_m  (12.0)  fallback edge limit when graph empty
    planner.half_width_m       (1.75)  virtual-candidate lateral offset
    planner.w_turn             (3.0)   heading-change weight in hop cost
    planner.max_turn_deg       (75.0)  max turn per hop

Note: fov.* parameters are declared for compatibility but the FOV
geometry currently uses the defaults baked into
`planner.geometry.fov_filter`; thread them through if you need runtime
FOV tuning.

## Smoke test without a car

Play a rosbag containing /slam/odom and /slam/map_cones, run the node,
and echo /path_points. The planner core itself is fully tested offline
(`python -m pytest tests/` from repo root).
