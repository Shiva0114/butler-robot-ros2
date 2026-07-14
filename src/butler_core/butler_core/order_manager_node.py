#!/usr/bin/env python3
"""
order_manager_node.py
=====================
Manages the order queue, dispatches orders to the FSM via DeliverOrder action,
and exposes PlaceOrder / CancelOrder services.
"""
import json
import threading
import uuid
from collections import deque
from dataclasses import dataclass
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from butler_msgs.srv import PlaceOrder, CancelOrder
from butler_msgs.action import DeliverOrder


@dataclass
class Order:
    order_id: str
    table_ids: List[str]
    require_kitchen_confirm: bool
    require_table_confirm: bool
    kitchen_timeout_sec: float
    table_timeout_sec: float

    @staticmethod
    def from_request(req: PlaceOrder.Request) -> "Order":
        return Order(
            order_id=str(uuid.uuid4()),
            table_ids=list(req.table_ids),
            require_kitchen_confirm=req.require_kitchen_confirm,
            require_table_confirm=req.require_table_confirm,
            kitchen_timeout_sec=req.kitchen_timeout_sec,
            table_timeout_sec=req.table_timeout_sec,
        )

    def to_goal(self) -> DeliverOrder.Goal:
        g = DeliverOrder.Goal()
        g.table_ids = self.table_ids
        g.require_kitchen_confirm = self.require_kitchen_confirm
        g.require_table_confirm = self.require_table_confirm
        g.kitchen_timeout_sec = self.kitchen_timeout_sec
        g.table_timeout_sec = self.table_timeout_sec
        return g


class OrderManagerNode(Node):

    def __init__(self):
        super().__init__("order_manager")
        self.declare_parameter("queue_publish_hz", 1.0)

        self._lock = threading.Lock()
        self._queue: deque = deque()
        self._active_order: Optional[Order] = None
        self._active_goal_handle = None

        self._cb_group = ReentrantCallbackGroup()

        self.create_service(
            PlaceOrder, "/butler/place_order", self._place_order_cb,
            callback_group=self._cb_group,
        )
        self.create_service(
            CancelOrder, "/butler/cancel_order_srv", self._cancel_order_cb,
            callback_group=self._cb_group,
        )

        self._action_client = ActionClient(
            self, DeliverOrder, "/butler/deliver_order",
            callback_group=self._cb_group,
        )

        self._queue_pub = self.create_publisher(String, "/butler/order_queue", 10)
        hz = self.get_parameter("queue_publish_hz").value
        self.create_timer(1.0 / hz, self._publish_queue)

        self.get_logger().info("OrderManager ready.")

    def _place_order_cb(self, req, res):
        if not req.table_ids:
            res.accepted = False
            res.message = "Rejected: table_ids empty"
            return res

        order = Order.from_request(req)
        with self._lock:
            self._queue.append(order)
            is_first = len(self._queue) == 1 and self._active_order is None

        self.get_logger().info(f"Order {order.order_id[:8]} queued, tables {order.table_ids}")
        if is_first:
            self._dispatch_next()

        res.accepted = True
        res.order_id = order.order_id
        res.message = "Accepted"
        return res

    def _cancel_order_cb(self, req, res):
        target_id = req.order_id
        with self._lock:
            for o in list(self._queue):
                if not target_id or o.order_id == target_id:
                    self._queue.remove(o)
                    res.cancelled = True
                    res.message = f"Queued order {o.order_id} removed"
                    return res
            if self._active_goal_handle and (
                not target_id or (self._active_order and self._active_order.order_id == target_id)
            ):
                self._active_goal_handle.cancel_goal_async()
                res.cancelled = True
                res.message = "Active order cancel requested"
                return res
        res.cancelled = False
        res.message = "Order not found"
        return res

    def _dispatch_next(self):
        with self._lock:
            if not self._queue:
                self._active_order = None
                self._active_goal_handle = None
                self.get_logger().info("Queue empty, idle")
                return
            self._active_order = self._queue.popleft()
        order = self._active_order
        self.get_logger().info(f"Dispatching {order.order_id[:8]} -> {order.table_ids}")

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("DeliverOrder action server unavailable!")
            self._dispatch_next()
            return

        future = self._action_client.send_goal_async(
            order.to_goal(), feedback_callback=self._feedback_cb
        )
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error("Goal rejected by FSM")
            self._dispatch_next()
            return
        with self._lock:
            self._active_goal_handle = gh
        gh.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().debug(f"[fb] {fb.current_state} -> {fb.current_target}")

    def _result_cb(self, future):
        result = future.result().result
        order = self._active_order
        if result.success:
            self.get_logger().info(
                f"Order {order.order_id[:8]} complete, delivered={list(result.delivered_tables)}"
            )
        else:
            self.get_logger().warn(
                f"Order {order.order_id[:8]} ended: {result.message}"
            )
        self._dispatch_next()

    def _publish_queue(self):
        with self._lock:
            active = (
                {"id": self._active_order.order_id, "tables": self._active_order.table_ids}
                if self._active_order else None
            )
            queued = [{"id": o.order_id, "tables": o.table_ids} for o in self._queue]
        msg = String()
        msg.data = json.dumps({"active": active, "queued": queued})
        self._queue_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OrderManagerNode()
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
