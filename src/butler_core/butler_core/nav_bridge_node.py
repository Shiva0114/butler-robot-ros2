#!/usr/bin/env python3
"""
nav_bridge_node.py
==================
Wraps Nav2's NavigateToPose action and exposes simple navigation
to the FSM. The FSM calls navigate_to(location_id) and gets back a bool.
Owns the location registry (loaded from locations.yaml parameters).
"""
import threading
from typing import Dict, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


class NavBridgeNode(Node):

    NAV2_ACTION = "/navigate_to_pose"

    def __init__(self):
        super().__init__("nav_bridge")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("goal_timeout_sec", 120.0)

        self._cb_group = ReentrantCallbackGroup()
        self._locations: Dict[str, Dict[str, Any]] = {}
        self._active_goal_handle = None
        self._nav_event: Optional[threading.Event] = None
        self._nav_succeeded = False

        self._load_locations()

        self._nav2_client = ActionClient(
            self, NavigateToPose, self.NAV2_ACTION,
            callback_group=self._cb_group,
        )

        self._feedback_pub = self.create_publisher(String, "/butler/nav_feedback", 10)

        self.get_logger().info(
            f"NavBridge ready. Locations: {list(self._locations.keys())}"
        )

    def _load_locations(self):
        params = self.get_parameters_by_prefix("locations")
        defaults = {
            "x": 0.0, "y": 0.0, "z": 0.0,
            "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
            "frame_id": "map",
        }
        for dotted, param in params.items():
            parts = dotted.split(".")
            if len(parts) < 2:
                continue
            loc_name, field = parts[0], ".".join(parts[1:])
            self._locations.setdefault(loc_name, dict(defaults))
            try:
                if field in ("x", "y", "z", "qx", "qy", "qz", "qw"):
                    self._locations[loc_name][field] = float(param.value)
                elif field == "frame_id":
                    self._locations[loc_name][field] = str(param.value)
            except (TypeError, ValueError):
                pass

    def _build_pose(self, location_id: str) -> Optional[PoseStamped]:
        loc = self._locations.get(location_id)
        if loc is None:
            self.get_logger().error(f"NavBridge: unknown location '{location_id}'")
            return None
        pose = PoseStamped()
        pose.header.frame_id = loc.get("frame_id", "map")
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = loc["x"]
        pose.pose.position.y = loc["y"]
        pose.pose.position.z = loc["z"]
        pose.pose.orientation.x = loc.get("qx", 0.0)
        pose.pose.orientation.y = loc.get("qy", 0.0)
        pose.pose.orientation.z = loc.get("qz", 0.0)
        pose.pose.orientation.w = loc.get("qw", 1.0)
        return pose

    def navigate_to(self, location_id: str) -> bool:
        pose = self._build_pose(location_id)
        if pose is None:
            return False

        if not self._nav2_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available!")
            return False

        self._nav_event = threading.Event()
        self._nav_succeeded = False
        self._active_goal_handle = None

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(
            f"NavBridge: navigating to '{location_id}' "
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
        )

        future = self._nav2_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        future.add_done_callback(self._goal_response_cb)

        timeout = self.get_parameter("goal_timeout_sec").value
        if not self._nav_event.wait(timeout=timeout):
            self.get_logger().error(
                f"NavBridge: navigation to '{location_id}' timed out after {timeout}s"
            )
            return False

        return self._nav_succeeded

    def cancel_navigation(self):
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()
            self.get_logger().info("NavBridge: cancel requested")

    def _goal_response_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error("NavBridge: goal rejected by Nav2")
            self._nav_succeeded = False
            if self._nav_event:
                self._nav_event.set()
            return
        self._active_goal_handle = gh
        gh.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        status = future.result().status
        self._nav_succeeded = (status == 4)  # STATUS_SUCCEEDED
        self._active_goal_handle = None
        if self._nav_event:
            self._nav_event.set()
        self.get_logger().info(
            f"NavBridge: result = {'SUCCESS' if self._nav_succeeded else 'FAILED/CANCELLED'}"
        )

    def _feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        msg = String()
        msg.data = f"distance_remaining={dist:.2f}m"
        self._feedback_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
