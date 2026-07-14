#!/usr/bin/env python3
"""
state_machine_node.py
======================
Butler Robot finite state machine. Implements all 7 assessment milestones.

States: IDLE -> NAVIGATING_TO_KITCHEN -> WAITING_AT_KITCHEN
        -> NAVIGATING_TO_TABLE -> WAITING_AT_TABLE
        -> NAVIGATING_HOME -> IDLE

ARCHITECTURE NOTE (v3):
-----------------------
Earlier versions tried to share a NavBridge Python object between this node
and a separate nav_bridge_node process. That is impossible — they are two
separate OS processes with separate memory, so the shared reference was
always None and navigation was only ever a fake 2-second sleep.

This version makes state_machine_node self-contained: it loads the named
location registry itself (from locations.yaml parameters) and owns its own
Nav2 NavigateToPose action client directly. No second process is needed.
nav_bridge_node.py is no longer used by the launch file.
"""
import time
import threading
from enum import Enum, auto
from typing import List, Dict, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from butler_msgs.action import DeliverOrder
from butler_msgs.msg import RobotState as RobotStateMsg


class State(Enum):
    IDLE = auto()
    NAVIGATING_TO_KITCHEN = auto()
    WAITING_AT_KITCHEN = auto()
    NAVIGATING_TO_TABLE = auto()
    WAITING_AT_TABLE = auto()
    NAVIGATING_HOME = auto()
    ERROR = auto()


class StateMachineNode(Node):

    KITCHEN_ID = "kitchen"
    HOME_ID = "home"
    NAV2_ACTION = "/navigate_to_pose"

    def __init__(self):
        # automatically_declare_parameters_from_overrides=True is the actual
        # fix for the long-running "Locations loaded: []" bug. Without this
        # flag, rclpy only exposes parameters that were explicitly declared
        # in code via declare_parameter(). Values supplied through a
        # --params-file (i.e. locations.yaml passed by the launch file) are
        # stored internally as "parameter overrides" but get_parameters_by_prefix()
        # returns nothing for them unless they are also declared. Since the
        # location names (home, kitchen, table1, ...) are not known ahead of
        # time, we cannot declare_parameter() each one individually — this
        # flag tells rclpy to auto-declare every override key it received,
        # which is exactly what we need for a dynamic, YAML-driven registry.
        super().__init__(
            "robot_state_machine",
            automatically_declare_parameters_from_overrides=True,
        )

        # Guard each declare_parameter call: with
        # automatically_declare_parameters_from_overrides=True, any of these
        # may already have been auto-declared if a launch file passed a
        # matching override (e.g. default_kitchen_timeout_sec is passed
        # directly in navigation_with_map.launch.py). Calling
        # declare_parameter() again on an already-declared name raises
        # ParameterAlreadyDeclaredException, so we check first.
        for name, default in (
            ("default_kitchen_timeout_sec", 30.0),
            ("default_table_timeout_sec", 30.0),
            ("map_frame", "map"),
            ("goal_timeout_sec", 120.0),
        ):
            if not self.has_parameter(name):
                self.declare_parameter(name, default)

        self._state = State.IDLE
        self._state_lock = threading.Lock()
        self._current_target = "home"
        self._order_start_time = 0.0

        self._remaining_tables: List[str] = []
        self._delivered_tables: List[str] = []
        self._skipped_tables: List[str] = []
        self._cancelled_tables: List[str] = []
        self._kitchen_confirmed = False

        self._confirm_event = threading.Event()
        self._cancel_requested = threading.Event()

        self._goal_handle = None

        self._cb_group = ReentrantCallbackGroup()

        # ── Location registry (loaded directly, no separate process) ──────────
        self._locations: Dict[str, Dict[str, Any]] = {}
        self._load_locations()

        # ── Nav2 action client (owned directly by this node) ──────────────────
        self._nav2_client = ActionClient(
            self, NavigateToPose, self.NAV2_ACTION,
            callback_group=self._cb_group,
        )
        self._nav_active_goal_handle = None
        self._nav_event: Optional[threading.Event] = None
        self._nav_succeeded = False

        self._nav_feedback_pub = self.create_publisher(String, "/butler/nav_feedback", 10)

        # ── DeliverOrder action server ──────────────────────────────────────────
        self._action_server = ActionServer(
            self, DeliverOrder, "/butler/deliver_order",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

        self.create_subscription(
            String, "/butler/confirmation", self._confirmation_cb, 10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            String, "/butler/cancel_order", self._cancel_topic_cb, 10,
            callback_group=self._cb_group,
        )

        self._state_pub = self.create_publisher(RobotStateMsg, "/butler/robot_state", 10)
        self.create_timer(0.5, self._publish_state)

        if not self._locations:
            # Loud, unmissable failure mode — this exact bug (empty location
            # registry due to a YAML top-level key / node-name mismatch, or a
            # stale install/ copy) has recurred multiple times in this project.
            # An ERROR with a banner is much harder to miss than a buried INFO line.
            self.get_logger().error(
                "\n"
                "############################################################\n"
                "# StateMachine started with ZERO locations loaded!         #\n"
                "# Every navigation goal will fail with 'Unknown location'. #\n"
                "#                                                          #\n"
                "# Most likely causes:                                     #\n"
                "#  1. automatically_declare_parameters_from_overrides is  #\n"
                "#     False (or missing) in super().__init__(...). This  #\n"
                "#     is the actual root cause found in this project --  #\n"
                "#     YAML overrides are not auto-declared otherwise.    #\n"
                "#  2. locations.yaml top-level key does not match this   #\n"
                "#     node's name ('robot_state_machine').               #\n"
                "#  3. The installed copy of locations.yaml is stale.     #\n"
                "#     Run: bash scripts/diagnose_locations.sh            #\n"
                "############################################################\n"
            )
        else:
            self.get_logger().info(
                f"StateMachine ready. Locations loaded: {list(self._locations.keys())}"
            )

    # ── Location registry ──────────────────────────────────────────────────────

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
            self.get_logger().error(f"Unknown location: '{location_id}'")
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

    # ── Action server callbacks ────────────────────────────────────────────────

    def _goal_cb(self, goal_request):
        with self._state_lock:
            if self._state != State.IDLE:
                self.get_logger().warn("Goal rejected, robot not idle")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info("Cancel request from action client")
        self._cancel_requested.set()
        self._cancel_navigation()
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        """
        Synchronous action execute callback (no asyncio).
        Runs the blocking delivery FSM on a plain Python thread and waits
        for it to finish, then returns the result directly. This avoids
        any dependency on an asyncio event loop being present in rclpy's
        executor, which is not guaranteed for action execute callbacks.
        """
        self._goal_handle = goal_handle
        goal = goal_handle.request

        self._confirm_event.clear()
        self._cancel_requested.clear()
        self._remaining_tables = list(goal.table_ids)
        self._delivered_tables = []
        self._skipped_tables = []
        self._cancelled_tables = []
        self._kitchen_confirmed = False
        self._order_start_time = time.time()

        result_holder = {}
        done_event = threading.Event()

        def _worker():
            result_holder["result"] = self._run_delivery(goal_handle, goal)
            done_event.set()

        thread = threading.Thread(target=_worker, daemon=True)
        thread.start()
        done_event.wait()

        return result_holder["result"]

    # ── Core FSM ────────────────────────────────────────────────────────────────

    def _run_delivery(self, goal_handle, goal: DeliverOrder.Goal) -> DeliverOrder.Result:
        result = DeliverOrder.Result()

        self._set_state(State.NAVIGATING_TO_KITCHEN, self.KITCHEN_ID)
        self._send_feedback(goal_handle, "NAVIGATING_TO_KITCHEN", self.KITCHEN_ID)

        nav_ok = self._navigate(self.KITCHEN_ID)

        if not nav_ok or self._cancel_requested.is_set():
            self.get_logger().warn("Aborted going to kitchen, returning home")
            self._set_state(State.NAVIGATING_HOME, self.HOME_ID)
            self._navigate(self.HOME_ID)
            self._set_state(State.IDLE, self.HOME_ID)
            result.success = False
            result.message = "Aborted during kitchen navigation"
            result.skipped_tables = list(self._remaining_tables)
            goal_handle.succeed()
            return result

        self._set_state(State.WAITING_AT_KITCHEN, self.KITCHEN_ID)
        self._send_feedback(goal_handle, "WAITING_AT_KITCHEN", self.KITCHEN_ID)

        if goal.require_kitchen_confirm:
            timeout = goal.kitchen_timeout_sec or self.get_parameter(
                "default_kitchen_timeout_sec").value
            confirmed = self._wait_confirm(timeout)
            if not confirmed:
                self.get_logger().warn("Kitchen timeout, returning home directly")
                self._set_state(State.NAVIGATING_HOME, self.HOME_ID)
                self._navigate(self.HOME_ID)
                self._set_state(State.IDLE, self.HOME_ID)
                result.success = False
                result.message = "Kitchen confirmation timeout"
                result.skipped_tables = list(self._remaining_tables)
                goal_handle.succeed()
                return result

        self._kitchen_confirmed = True
        self.get_logger().info("Kitchen confirmed, proceeding to tables")

        for table_id in list(self._remaining_tables):

            if table_id in self._cancelled_tables:
                self.get_logger().info(f"Table {table_id} pre-cancelled, skipping")
                self._skipped_tables.append(table_id)
                continue

            self._set_state(State.NAVIGATING_TO_TABLE, table_id)
            self._send_feedback(goal_handle, "NAVIGATING_TO_TABLE", table_id)

            nav_ok = self._navigate(table_id)

            if self._cancel_requested.is_set():
                remaining = [t for t in self._remaining_tables
                             if t != table_id and t not in self._delivered_tables]
                self._skipped_tables.extend([table_id] + remaining)
                self.get_logger().warn(f"Cancelled en-route to {table_id}, going kitchen then home")
                if self._kitchen_confirmed:
                    self._set_state(State.NAVIGATING_TO_KITCHEN, self.KITCHEN_ID)
                    self._navigate(self.KITCHEN_ID)
                self._set_state(State.NAVIGATING_HOME, self.HOME_ID)
                self._navigate(self.HOME_ID)
                self._set_state(State.IDLE, self.HOME_ID)
                result.success = False
                result.message = "Cancelled during table delivery"
                result.delivered_tables = list(self._delivered_tables)
                result.skipped_tables = list(self._skipped_tables)
                goal_handle.succeed()
                return result

            if not nav_ok:
                self.get_logger().warn(f"Navigation to {table_id} failed, skipping")
                self._skipped_tables.append(table_id)
                continue

            self._set_state(State.WAITING_AT_TABLE, table_id)
            self._send_feedback(goal_handle, "WAITING_AT_TABLE", table_id)
            self._confirm_event.clear()

            if goal.require_table_confirm:
                timeout = goal.table_timeout_sec or self.get_parameter(
                    "default_table_timeout_sec").value
                confirmed = self._wait_confirm(timeout)
            else:
                confirmed = True

            if confirmed:
                self.get_logger().info(f"Table {table_id} delivered")
                self._delivered_tables.append(table_id)
            else:
                self.get_logger().warn(f"Table {table_id} timeout, skipping")
                self._skipped_tables.append(table_id)

        if self._skipped_tables and self._kitchen_confirmed:
            self.get_logger().info("Undelivered food, returning to kitchen before home")
            self._set_state(State.NAVIGATING_TO_KITCHEN, self.KITCHEN_ID)
            self._send_feedback(goal_handle, "NAVIGATING_TO_KITCHEN", self.KITCHEN_ID)
            self._navigate(self.KITCHEN_ID)

        self._set_state(State.NAVIGATING_HOME, self.HOME_ID)
        self._send_feedback(goal_handle, "NAVIGATING_HOME", self.HOME_ID)
        self._navigate(self.HOME_ID)
        self._set_state(State.IDLE, self.HOME_ID)

        result.success = bool(self._delivered_tables)
        result.message = f"delivered={self._delivered_tables}, skipped={self._skipped_tables}"
        result.delivered_tables = list(self._delivered_tables)
        result.skipped_tables = list(self._skipped_tables)
        goal_handle.succeed()
        return result

    # ── Navigation (direct Nav2 client, no separate process) ──────────────────

    def _navigate(self, location_id: str) -> bool:
        """
        Synchronously navigate to a named location by sending a real
        NavigateToPose goal to Nav2. Blocks the calling thread (which is
        the per-delivery worker thread, NOT the rclpy executor thread) until
        Nav2 finishes. Returns True on success, False on failure/cancel.
        """
        pose = self._build_pose(location_id)
        if pose is None:
            return False

        if not self._nav2_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available!")
            return False

        self._nav_event = threading.Event()
        self._nav_succeeded = False
        self._nav_active_goal_handle = None

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(
            f"Navigating to '{location_id}' "
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
        )

        send_future = self._nav2_client.send_goal_async(
            goal, feedback_callback=self._nav2_feedback_cb
        )
        send_future.add_done_callback(self._nav2_goal_response_cb)

        timeout = self.get_parameter("goal_timeout_sec").value
        if not self._nav_event.wait(timeout=timeout):
            self.get_logger().error(f"Navigation to '{location_id}' timed out after {timeout}s")
            return False

        return self._nav_succeeded

    def _cancel_navigation(self):
        if self._nav_active_goal_handle is not None:
            self._nav_active_goal_handle.cancel_goal_async()
            self.get_logger().info("Navigation cancel requested")

    def _nav2_goal_response_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error("Nav2 rejected the goal")
            self._nav_succeeded = False
            if self._nav_event:
                self._nav_event.set()
            return
        self._nav_active_goal_handle = gh
        gh.get_result_async().add_done_callback(self._nav2_result_cb)

    def _nav2_result_cb(self, future):
        status = future.result().status
        self._nav_succeeded = (status == 4)  # GoalStatus.STATUS_SUCCEEDED
        self._nav_active_goal_handle = None
        if self._nav_event:
            self._nav_event.set()
        self.get_logger().info(
            f"Nav2 result: {'SUCCESS' if self._nav_succeeded else 'FAILED/CANCELLED'}"
        )

    def _nav2_feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        msg = String()
        msg.data = f"distance_remaining={dist:.2f}m"
        self._nav_feedback_pub.publish(msg)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _wait_confirm(self, timeout_sec: float) -> bool:
        self._confirm_event.clear()
        self.get_logger().info(f"Waiting for confirmation (timeout={timeout_sec}s)")
        start = time.time()
        while not self._confirm_event.is_set():
            if self._cancel_requested.is_set():
                return False
            if time.time() - start >= timeout_sec:
                return False
            time.sleep(0.1)
        return True

    def _set_state(self, state: State, target: str):
        with self._state_lock:
            self._state = state
            self._current_target = target
        self.get_logger().info(f"[FSM] -> {state.name} (target={target})")

    def _send_feedback(self, goal_handle, state_name: str, target: str):
        fb = DeliverOrder.Feedback()
        fb.current_state = state_name
        fb.current_target = target
        fb.remaining_tables = list(self._remaining_tables)
        fb.elapsed_sec = time.time() - self._order_start_time
        goal_handle.publish_feedback(fb)

    # ── Subscriber callbacks ────────────────────────────────────────────────────

    def _confirmation_cb(self, msg: String):
        self.get_logger().info(f"Confirmation received: '{msg.data}'")
        self._confirm_event.set()

    def _cancel_topic_cb(self, msg: String):
        payload = msg.data.strip()
        self.get_logger().info(f"Cancel signal: '{payload}'")
        if payload in self._remaining_tables:
            self._cancelled_tables.append(payload)
            self.get_logger().info(f"Table '{payload}' pre-cancelled")
        else:
            self._cancel_requested.set()
            self._cancel_navigation()

    # ── State publisher ─────────────────────────────────────────────────────────

    def _publish_state(self):
        msg = RobotStateMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self._state_lock:
            msg.state = self._state.name
            msg.current_target = self._current_target
        msg.remaining_tables = list(self._remaining_tables)
        msg.delivered_tables = list(self._delivered_tables)
        msg.skipped_tables = list(self._skipped_tables)
        msg.elapsed_sec = (time.time() - self._order_start_time
                           if self._order_start_time else 0.0)
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
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
