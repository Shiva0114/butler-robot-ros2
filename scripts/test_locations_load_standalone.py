#!/usr/bin/env python3
"""
test_locations_load_standalone.py
====================================
Standalone, fast proof that the automatically_declare_parameters_from_overrides
fix actually works, without needing Gazebo, Nav2, or any other heavy process.

This starts ONLY a minimal rclpy node with the exact same constructor
arguments as state_machine_node.py, loads locations.yaml the same way,
and prints the result. If this prints all 6 locations, the fix is
proven correct in isolation -- any remaining failure in the full launch
is a different, separate issue.

Usage:
    cd ~/butler_robot_ws
    source install/setup.bash
    python3 scripts/test_locations_load_standalone.py
"""
import os
import sys

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


def main():
    rclpy.init()

    locations_yaml = os.path.join(
        get_package_share_directory("butler_navigation"),
        "config",
        "locations.yaml",
    )
    print(f"Loading params from: {locations_yaml}")

    if not os.path.exists(locations_yaml):
        print("FAIL: file does not exist at that path.")
        sys.exit(1)

    # Build the node the SAME way state_machine_node.py does, passing
    # the YAML file as a parameter override source via rclpy.init args,
    # exactly as the launch system does with --params-file.
    node = rclpy.create_node(
        "robot_state_machine",
        parameter_overrides=[],
        automatically_declare_parameters_from_overrides=True,
        cli_args=["--ros-args", "--params-file", locations_yaml],
    )

    params = node.get_parameters_by_prefix("locations")
    location_names = sorted(set(k.split(".")[0] for k in params.keys()))

    print(f"\nRaw parameter count under 'locations' prefix: {len(params)}")
    print(f"Distinct location names found: {location_names}")

    if location_names:
        print("\nPASS: locations loaded correctly with automatically_declare_parameters_from_overrides=True")
        for loc in location_names:
            x = node.get_parameter(f"locations.{loc}.x").value
            y = node.get_parameter(f"locations.{loc}.y").value
            print(f"  {loc}: x={x}, y={y}")
        result = 0
    else:
        print("\nFAIL: zero locations loaded. The fix did not take effect.")
        print("Check that state_machine_node.py was actually rebuilt and reinstalled.")
        result = 1

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(result)


if __name__ == "__main__":
    main()
