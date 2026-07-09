#!/usr/bin/env python3
# eufs_adapter.py
#
# Bridges EUFS Sim to the improved_plan_v2 planner and (optionally) applies
# the SLAM degradation model live, so the sim reproduces the offline
# benchmark's stress conditions.
#
# KEY FRAME FACT (discovered from the running sim):
#   /ground_truth/cones are in the CAR-LOCAL frame (x forward, y left,
#   origin at the car). The planner's fov_filter expects GLOBAL cones plus
#   a car pose and internally transforms to local. So we feed the planner
#   car pose (0, 0, 0): _world_to_local becomes identity and the whole
#   pipeline runs correctly in the car frame. The resulting path is in the
#   car frame, so it is published with frame_id = base_footprint.
#
# Run (baseline vs v2, clean):
#   PYTHONPATH=~/fs_planning/fs-path-planning \
#   python3 eufs_adapter.py --ros-args -p planner:=v2 -p use_sim_time:=true
#
# Run with degradation (the real robustness test):
#   ... -p degrade.drop_rate:=0.30 -p degrade.pos_noise:=0.2
#
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSHistoryPolicy,
                       QoSReliabilityPolicy, QoSDurabilityPolicy)

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from eufs_msgs.msg import ConeArrayWithCovariance

from planner import baseline_plan, improved_plan_v2

# degradation reuses the offline model (single-frame subset: noise/drop/flip)
_rng = np.random.default_rng(0)


class EufsAdapter(Node):
    def __init__(self):
        super().__init__("eufs_adapter")

        self.declare_parameter("planner", "v2")            # "baseline" | "v2"
        self.declare_parameter("cones_in", "/ground_truth/cones")
        self.declare_parameter("path_out", "/path_points")
        self.declare_parameter("car_frame", "base_footprint")
        self.declare_parameter("rate_hz", 20.0)

        # live degradation (0 = off). Applied to the LOCAL cone set each frame.
        self.declare_parameter("degrade.pos_noise", 0.0)
        self.declare_parameter("degrade.drop_rate", 0.0)
        self.declare_parameter("degrade.color_flip", 0.0)

        gp = self.get_parameter
        self.which = str(gp("planner").value)
        cones_in = str(gp("cones_in").value)
        self.path_topic = str(gp("path_out").value)
        self.car_frame = str(gp("car_frame").value)
        rate = float(gp("rate_hz").value)

        self.pos_noise = float(gp("degrade.pos_noise").value)
        self.drop_rate = float(gp("degrade.drop_rate").value)
        self.color_flip = float(gp("degrade.color_flip").value)

        self.plan_fn = improved_plan_v2 if self.which == "v2" else baseline_plan

        sub_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,   # match EUFS publisher
            durability=QoSDurabilityPolicy.VOLATILE)
        pub_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.create_subscription(ConeArrayWithCovariance, cones_in,
                                 self.cb_cones, sub_qos)
        self.path_pub = self.create_publisher(Path, self.path_topic, pub_qos)

        self.latest = None
        self.get_logger().info(
            f"[eufs_adapter] planner={self.which} cones_in={cones_in} "
            f"path_out={self.path_topic} frame={self.car_frame} | "
            f"degrade(noise={self.pos_noise},drop={self.drop_rate},"
            f"flip={self.color_flip})")

        self.create_timer(1.0 / max(rate, 1e-6), self._tick)

    def cb_cones(self, msg: ConeArrayWithCovariance):
        def arr(cones):
            return [(float(c.point.x), float(c.point.y)) for c in cones]
        self.latest = {
            "blue": arr(msg.blue_cones),
            "yellow": arr(msg.yellow_cones),
            "orange": arr(msg.orange_cones),
            "big": arr(msg.big_orange_cones),
        }

    def _degrade_local(self, cones):
        """Apply noise/drop/flip to the local cone dict (live stress test)."""
        blue = np.array(cones["blue"], dtype=float).reshape(-1, 2)
        yellow = np.array(cones["yellow"], dtype=float).reshape(-1, 2)

        if self.drop_rate > 0:
            blue = blue[_rng.random(len(blue)) > self.drop_rate]
            yellow = yellow[_rng.random(len(yellow)) > self.drop_rate]
        if self.color_flip > 0 and len(blue) and len(yellow):
            fb = _rng.random(len(blue)) < self.color_flip
            fy = _rng.random(len(yellow)) < self.color_flip
            nb = np.vstack([blue[~fb], yellow[fy]]) if fy.any() or (~fb).any() else np.zeros((0, 2))
            ny = np.vstack([yellow[~fy], blue[fb]]) if fb.any() or (~fy).any() else np.zeros((0, 2))
            blue, yellow = nb.reshape(-1, 2), ny.reshape(-1, 2)
        if self.pos_noise > 0:
            if len(blue):
                blue = blue + _rng.normal(0, self.pos_noise, blue.shape)
            if len(yellow):
                yellow = yellow + _rng.normal(0, self.pos_noise, yellow.shape)

        return {"blue": [tuple(p) for p in blue],
                "yellow": [tuple(p) for p in yellow],
                "orange": cones["orange"],
                "big": cones["big"]}

    def _tick(self):
        if self.latest is None:
            return
        cones = self.latest
        if self.pos_noise or self.drop_rate or self.color_flip:
            cones = self._degrade_local(cones)

        # cones already local -> car pose is the origin
        path, _, _, _ = self.plan_fn((0.0, 0.0, 0.0), cones)
        self._publish(path)

    def _publish(self, path_points):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.car_frame     # path is in the car frame

        def pose(x, y):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            return ps

        # start at the car origin, then the planned points
        msg.poses = [pose(0.0, 0.0)] + [pose(x, y) for x, y in path_points]
        self.path_pub.publish(msg)


def main():
    rclpy.init()
    node = EufsAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

