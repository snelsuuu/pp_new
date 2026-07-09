#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSHistoryPolicy,
                       QoSReliabilityPolicy, QoSDurabilityPolicy)
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDriveStamped


class PurePursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit")
        self.declare_parameter("path_in", "/path_points")
        self.declare_parameter("cmd_out", "/cmd")
        self.declare_parameter("speed", 3.0)
        self.declare_parameter("lookahead", 4.0)
        self.declare_parameter("wheelbase", 1.58)
        self.declare_parameter("max_steer", 0.52)
        self.declare_parameter("stop_if_no_path", True)
        gp = self.get_parameter
        path_in = str(gp("path_in").value)
        cmd_out = str(gp("cmd_out").value)
        self.speed = float(gp("speed").value)
        self.ld = float(gp("lookahead").value)
        self.L = float(gp("wheelbase").value)
        self.max_steer = float(gp("max_steer").value)
        self.stop_if_no_path = bool(gp("stop_if_no_path").value)
        path_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(Path, path_in, self.cb_path, path_qos)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, cmd_out, 10)
        self.last_steer = 0.0
        self.get_logger().info(
            f"[pure_pursuit] path_in={path_in} cmd_out={cmd_out} "
            f"speed={self.speed} lookahead={self.ld} wheelbase={self.L}")

    def cb_path(self, msg: Path):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses[1:]]
        if not pts:
            if self.stop_if_no_path:
                self._drive(0.0, self.last_steer)
            return
        target = None
        for x, y in pts:
            if math.hypot(x, y) >= self.ld:
                target = (x, y)
                break
        if target is None:
            target = pts[-1]
        tx, ty = target
        ld2 = tx * tx + ty * ty
        if ld2 < 1e-6:
            self._drive(0.0, self.last_steer)
            return
        steer = math.atan2(2.0 * self.L * ty, ld2)
        steer = max(-self.max_steer, min(self.max_steer, steer))
        self.last_steer = steer
        self._drive(self.speed, steer)

    def _drive(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.acceleration = 1.0
        msg.drive.steering_angle = float(steer)
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
