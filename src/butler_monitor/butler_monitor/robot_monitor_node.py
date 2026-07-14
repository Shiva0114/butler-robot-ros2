#!/usr/bin/env python3
"""
robot_monitor_node.py
=====================
Subscribes to /butler/robot_state and /butler/order_queue, prints a
live status dashboard, and publishes delivery metrics.
Observational only — no effect on robot behaviour.
"""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from butler_msgs.msg import RobotState


class RobotMonitorNode(Node):

    def __init__(self):
        super().__init__("robot_monitor")

        self._total_orders = 0
        self._total_delivered = 0
        self._total_skipped = 0
        self._last_state = "IDLE"
        self._queue_depth = 0
        self._nav_dist = 0.0

        self.create_subscription(RobotState, "/butler/robot_state", self._state_cb, 10)
        self.create_subscription(String, "/butler/order_queue", self._queue_cb, 10)
        self.create_subscription(String, "/butler/nav_feedback", self._nav_feedback_cb, 10)

        self._metrics_pub = self.create_publisher(String, "/butler/metrics", 10)
        self.create_timer(2.0, self._print_dashboard)
        self.get_logger().info("RobotMonitor started.")

    def _state_cb(self, msg: RobotState):
        new = msg.state
        if new != self._last_state:
            self.get_logger().info(f"[STATE] {self._last_state} -> {new}")
            if new == "IDLE" and self._last_state not in ("IDLE", "ERROR"):
                self._total_orders += 1
                self._total_delivered += len(msg.delivered_tables)
                self._total_skipped += len(msg.skipped_tables)
            self._last_state = new

        m = String()
        m.data = json.dumps({
            "orders": self._total_orders,
            "delivered": self._total_delivered,
            "skipped": self._total_skipped,
        })
        self._metrics_pub.publish(m)

    def _queue_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
            self._queue_depth = len(d.get("queued", []))
        except json.JSONDecodeError:
            pass

    def _nav_feedback_cb(self, msg: String):
        try:
            self._nav_dist = float(msg.data.split("=")[1].replace("m", ""))
        except (IndexError, ValueError):
            pass

    def _print_dashboard(self):
        sep = "-" * 42
        self.get_logger().info(
            f"\n{sep}\n"
            f"  Butler Robot Live Status\n"
            f"{sep}\n"
            f"  State      : {self._last_state}\n"
            f"  Queue depth: {self._queue_depth}\n"
            f"  Nav dist   : {self._nav_dist:.2f} m\n"
            f"  Orders done: {self._total_orders}\n"
            f"  Deliveries : {self._total_delivered}\n"
            f"  Skipped    : {self._total_skipped}\n"
            f"{sep}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
