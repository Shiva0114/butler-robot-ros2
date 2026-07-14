#!/usr/bin/env python3
"""
timeout_manager_node.py
========================
Manages named timers via pub/sub.
Sub /butler/timeout_cmd  (String) JSON: {action, id, sec}
Pub /butler/timeout_event (String) JSON: {id, event:"expired"}
"""
import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TimeoutManagerNode(Node):

    def __init__(self):
        super().__init__("timeout_manager")
        self._timers: dict = {}
        self._lock = threading.Lock()

        self._pub = self.create_publisher(String, "/butler/timeout_event", 10)
        self.create_subscription(String, "/butler/timeout_cmd", self._cmd_cb, 10)
        self.get_logger().info("TimeoutManager ready.")

    def _cmd_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
            action = d.get("action", "start")
            tid = d.get("id", "default")
            sec = float(d.get("sec", 30.0))
        except (json.JSONDecodeError, ValueError) as e:
            self.get_logger().error(f"Bad timeout cmd: {e}")
            return

        if action == "start":
            self._start(tid, sec)
        elif action == "stop":
            self._stop(tid)
        elif action == "reset":
            self._stop(tid)
            self._start(tid, sec)

    def _start(self, tid: str, sec: float):
        self._stop(tid)
        t = threading.Timer(sec, self._expire, args=[tid])
        with self._lock:
            self._timers[tid] = t
        t.start()
        self.get_logger().info(f"Timer '{tid}' started ({sec}s)")

    def _stop(self, tid: str):
        with self._lock:
            t = self._timers.pop(tid, None)
        if t:
            t.cancel()

    def _expire(self, tid: str):
        with self._lock:
            self._timers.pop(tid, None)
        msg = String()
        msg.data = json.dumps({"id": tid, "event": "expired"})
        self._pub.publish(msg)
        self.get_logger().info(f"Timer '{tid}' expired")


def main(args=None):
    rclpy.init(args=args)
    node = TimeoutManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
