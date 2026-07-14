#!/usr/bin/env python3
"""
slam_safe_velocity_limiter.py
==============================
Sits between teleop_twist_keyboard and the robot during SLAM mapping.
Clamps linear and angular velocity to levels that keep SLAM Toolbox's
scan-matching reliable, regardless of how fast keys are pressed.

Why this exists:
-----------------
SLAM Toolbox only accepts a new scan once the robot has moved
minimum_travel_distance or rotated minimum_travel_heading since the
last accepted scan. If the robot moves/rotates too fast between scan
cycles, consecutive scans can be matched incorrectly, producing
sheared, duplicated, or noisy maps (exactly the artifact seen when
mapping was done with default teleop_twist_keyboard speeds, which
allow ~1.0 rad/s rotation — too fast for reliable scan matching at
this lidar's 10 Hz update rate).

This node subscribes to /cmd_vel_teleop (raw teleop output) and
republishes a clamped version to /cmd_vel (what the robot actually
listens to).

Usage:
    Terminal 1: ros2 launch butler_bringup slam_mapping.launch.py
    Terminal 2: ros2 run butler_core slam_safe_velocity_limiter.py
    Terminal 3: ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
                    --ros-args -r cmd_vel:=cmd_vel_teleop
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocityLimiterNode(Node):

    def __init__(self):
        super().__init__("slam_safe_velocity_limiter")

        self.declare_parameter("max_linear_vel", 0.15)   # m/s
        self.declare_parameter("max_angular_vel", 0.4)   # rad/s — well below default ~1.0

        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Twist, "/cmd_vel_teleop", self._cmd_cb, 10)

        self.get_logger().info(
            f"SLAM-safe velocity limiter active: "
            f"max_linear={self.get_parameter('max_linear_vel').value} m/s, "
            f"max_angular={self.get_parameter('max_angular_vel').value} rad/s"
        )

    def _clamp(self, value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _cmd_cb(self, msg: Twist):
        max_lin = self.get_parameter("max_linear_vel").value
        max_ang = self.get_parameter("max_angular_vel").value

        out = Twist()
        out.linear.x = self._clamp(msg.linear.x, max_lin)
        out.linear.y = self._clamp(msg.linear.y, max_lin)
        out.angular.z = self._clamp(msg.angular.z, max_ang)

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityLimiterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
