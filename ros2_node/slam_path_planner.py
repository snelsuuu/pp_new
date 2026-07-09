#!/usr/bin/env python3
# slam_path_planner.py
#
# ROS2 wrapper around planner.improved_plan_v2 (benchmarked winner).
# Drop-in replacement for slam_path_sector_visualiser.py:
#   - same topics:   /slam/odom, /slam/map_cones -> /path_points
#   - same QoS:      BEST_EFFORT publisher, configurable subscriber
#   - same base parameters, plus the v2 planner parameters
#
# Benchmark (closed-loop laps, 15 seeds x 10 degradation scenarios):
#   baseline 54% lap completion -> v2 85%; p95 plan time 2.4 ms.
#
import math
import threading
from typing import List, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from eufs_msgs.msg import ConeArrayWithCovariance

# Pure-python planner core (this repo's `planner` package must be on
# PYTHONPATH, e.g. installed via the package's setup.py or colcon).
from planner import improved_plan_v2
from planner.geometry import yaw_from_quat


class SlamPathPlanner(Node):
    def __init__(self):
        super().__init__("slam_path_planner")

        # ---- Parameters (same names as the original node) ----
        self.declare_parameter("topics.odom_in", "/slam/odom")
        self.declare_parameter("topics.map_in", "/slam/map_cones")
        self.declare_parameter("topics.path_out", "/path_points")

        self.declare_parameter("qos.best_effort", True)
        self.declare_parameter("qos.depth", 50)

        self.declare_parameter("fov.vertex_offset_m", -5.0)
        self.declare_parameter("fov.radius_m", 30.0)
        self.declare_parameter("fov.angle_deg", 60.0)

        self.declare_parameter("candidates.min_spacing_m", 1.0)
        self.declare_parameter("candidates.min_car_dist_m", 2.0)

        self.declare_parameter("greedy.max_hop_m", 7.0)
        self.declare_parameter("publish.rate_hz", 20.0)

        # ---- v2 planner parameters ----
        self.declare_parameter("planner.base_max_len_m", 6.0)
        self.declare_parameter("planner.relaxed_max_len_m", 12.0)
        self.declare_parameter("planner.half_width_m", 1.75)
        self.declare_parameter("planner.w_turn", 3.0)
        self.declare_parameter("planner.max_turn_deg", 75.0)

        gp = self.get_parameter
        odom_topic = str(gp("topics.odom_in").value)
        map_topic = str(gp("topics.map_in").value)
        path_topic = str(gp("topics.path_out").value)

        best_effort = bool(gp("qos.best_effort").value)
        depth = int(gp("qos.depth").value)

        # NOTE: fov.* parameters are currently fixed inside
        # planner.geometry.fov_filter defaults; passing them through is a
        # one-line change there if you need runtime tuning.
        self.min_spacing = float(gp("candidates.min_spacing_m").value)
        self.min_car_dist = float(gp("candidates.min_car_dist_m").value)
        self.max_hop = float(gp("greedy.max_hop_m").value)
        self.publish_rate_hz = float(gp("publish.rate_hz").value)

        self.base_max_len = float(gp("planner.base_max_len_m").value)
        self.relaxed_max_len = float(gp("planner.relaxed_max_len_m").value)
        self.half_width = float(gp("planner.half_width_m").value)
        self.w_turn = float(gp("planner.w_turn").value)
        self.max_turn_deg = float(gp("planner.max_turn_deg").value)

        sub_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=depth,
            reliability=(
                QoSReliabilityPolicy.BEST_EFFORT
                if best_effort
                else QoSReliabilityPolicy.RELIABLE
            ),
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        pub_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=depth,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Odometry, odom_topic, self.cb_odom, sub_qos)
        self.create_subscription(
            ConeArrayWithCovariance, map_topic, self.cb_map, sub_qos)
        self.path_pub = self.create_publisher(Path, path_topic, pub_qos)

        self.data_lock = threading.Lock()
        self.car = None  # (x, y, yaw)
        self.cones = {
            "blue": np.zeros((0, 2)),
            "yellow": np.zeros((0, 2)),
            "orange": np.zeros((0, 2)),
            "big": np.zeros((0, 2)),
        }

        self.get_logger().info(
            f"[slam_path_planner] v2 planner | odom={odom_topic} "
            f"map={map_topic} path_out={path_topic} | "
            f"max_hop={self.max_hop}m w_turn={self.w_turn} "
            f"max_turn={self.max_turn_deg}deg")

        period = 1.0 / max(self.publish_rate_hz, 1e-6)
        self.timer = self.create_timer(period, self._tick)

    # --------------------- Callbacks ---------------------

    def cb_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        car = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw_from_quat(q.x, q.y, q.z, q.w),
        )
        with self.data_lock:
            self.car = car

    def cb_map(self, msg: ConeArrayWithCovariance):
        def arr(cones):
            return np.array(
                [(float(c.point.x), float(c.point.y)) for c in cones],
                dtype=float).reshape(-1, 2)

        cones = {
            "blue": arr(msg.blue_cones),
            "yellow": arr(msg.yellow_cones),
            "orange": arr(msg.orange_cones),
            "big": arr(msg.big_orange_cones),
        }
        with self.data_lock:
            self.cones = cones

    # --------------------- Main loop ---------------------

    def _tick(self):
        with self.data_lock:
            car = self.car
            cones = self.cones

        if car is None:
            return

        path, _, _, _ = improved_plan_v2(
            car, cones,
            min_spacing=self.min_spacing,
            min_car_dist=self.min_car_dist,
            max_hop=self.max_hop,
            base_max_len=self.base_max_len,
            relaxed_max_len=self.relaxed_max_len,
            half_width=self.half_width,
            w_turn=self.w_turn,
            max_turn_deg=self.max_turn_deg,
        )
        self._publish_path(car[0], car[1], path)

    def _publish_path(self, car_x: float, car_y: float,
                      path_points: List[Tuple[float, float]]):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        def pose(x, y):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            return ps

        msg.poses = [pose(car_x, car_y)] + [pose(x, y) for x, y in path_points]
        self.path_pub.publish(msg)


def main():
    rclpy.init()
    node = SlamPathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
