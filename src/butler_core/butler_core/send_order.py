#!/usr/bin/env python3
"""
send_order.py
=============
CLI tool to place orders on the butler robot.

Usage:
  python3 send_order.py --tables table1
  python3 send_order.py --tables table1 table2 table3 --kitchen-confirm
  python3 send_order.py --demo
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from butler_msgs.srv import PlaceOrder


MILESTONE_DEMOS = [
    {
        "name": "Milestone 1 - Single delivery, no confirmation",
        "tables": ["table1"],
        "kitchen_confirm": False,
        "table_confirm": False,
        "kitchen_timeout": 0.0,
        "table_timeout": 0.0,
    },
    {
        "name": "Milestone 5 - Multi-table delivery",
        "tables": ["table1", "table2", "table3"],
        "kitchen_confirm": True,
        "table_confirm": False,
        "kitchen_timeout": 30.0,
        "table_timeout": 0.0,
    },
]


class OrderClient(Node):
    def __init__(self):
        super().__init__("order_cli_client")
        self._client = self.create_client(PlaceOrder, "/butler/place_order")

    def send(self, tables, kitchen_confirm=False, table_confirm=False,
             kitchen_timeout=30.0, table_timeout=30.0):
        if not self._client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/butler/place_order service not available")
            return None

        req = PlaceOrder.Request()
        req.table_ids = tables
        req.require_kitchen_confirm = kitchen_confirm
        req.require_table_confirm = table_confirm
        req.kitchen_timeout_sec = kitchen_timeout
        req.table_timeout_sec = table_timeout

        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done():
            self.get_logger().error("Service call timed out")
            return None

        res = future.result()
        if res.accepted:
            self.get_logger().info(f"Order {res.order_id[:8]} accepted, tables {tables}")
        else:
            self.get_logger().warn(f"Order rejected: {res.message}")
        return res.order_id


def main():
    parser = argparse.ArgumentParser(description="Butler Robot Order CLI")
    parser.add_argument("--tables", nargs="+", default=["table1"])
    parser.add_argument("--kitchen-confirm", action="store_true")
    parser.add_argument("--table-confirm", action="store_true")
    parser.add_argument("--kitchen-timeout", type=float, default=30.0)
    parser.add_argument("--table-timeout", type=float, default=30.0)
    parser.add_argument("--demo", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    client = OrderClient()

    if args.demo:
        print("\n=== Butler Robot Interactive Demo ===\n")
        for m in MILESTONE_DEMOS:
            print(f"\n>> {m['name']}")
            input("   Press ENTER to send this order...")
            client.send(
                tables=m["tables"],
                kitchen_confirm=m["kitchen_confirm"],
                table_confirm=m["table_confirm"],
                kitchen_timeout=m["kitchen_timeout"],
                table_timeout=m["table_timeout"],
            )
            time.sleep(1.0)
    else:
        client.send(
            tables=args.tables,
            kitchen_confirm=args.kitchen_confirm,
            table_confirm=args.table_confirm,
            kitchen_timeout=args.kitchen_timeout,
            table_timeout=args.table_timeout,
        )

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
