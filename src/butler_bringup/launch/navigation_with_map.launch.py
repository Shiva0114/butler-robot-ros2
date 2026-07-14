"""
navigation_with_map.launch.py (simple_cafe version)
=====================================================
Autonomous navigation with saved map of the simple_cafe world.
Robot spawns at (0.5, 0.0) — the home dock position.

Usage:
    ros2 launch butler_bringup navigation_with_map.launch.py
    ros2 launch butler_bringup navigation_with_map.launch.py map:=/full/path/cafe_map.yaml

After launch:
  1. Set 2D Pose Estimate in RViz at the blue home dock (0.5, 0.0)
  2. ros2 run butler_core send_order.py --tables table1
"""
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, LogInfo,
    ExecuteProcess, SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_desc   = get_package_share_directory("butler_description")
    pkg_gazebo = get_package_share_directory("butler_gazebo")
    pkg_nav    = get_package_share_directory("butler_navigation")

    xacro_file        = os.path.join(pkg_desc,  "urdf",   "butler_robot.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()
    world_file        = os.path.join(pkg_gazebo, "worlds", "simple_cafe.world")
    nav2_params       = os.path.join(pkg_nav,    "config", "nav2_params.yaml")
    rviz_config       = os.path.join(pkg_desc,   "rviz",   "butler_navigation.rviz")
    locations_cfg     = os.path.join(pkg_nav,    "config", "locations.yaml")

    default_map = os.path.join(
        os.path.expanduser("~"), "butler_robot_ws", "maps", "cafe_map.yaml"
    )

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    map_arg = DeclareLaunchArgument(
        "map", default_value=default_map,
        description="Full path to saved map YAML",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml     = LaunchConfiguration("map")

    force_xcb = SetEnvironmentVariable("QT_QPA_PLATFORM",     "xcb")
    force_sw  = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE","1")

    # ── 1. Gazebo ──────────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose", world_file,
             "-s", "libgazebo_ros_init.so",
             "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )

    # ── 2. Robot ───────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_butler_robot",
        output="screen",
        arguments=[
            "-entity", "butler_robot",
            "-topic", "/robot_description",
            "-x", "0.5", "-y", "0.0", "-z", "0.1", "-Y", "0.0",
        ],
    )

    # ── 3. Nav2 localization at 6s ─────────────────────────────────────────
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
        remappings=[("scan", "/scan")],
    )

    lifecycle_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
        }],
    )

    localization_group = TimerAction(period=6.0, actions=[
        LogInfo(msg="[Nav2] Starting map_server + amcl..."),
        map_server, amcl, lifecycle_localization,
    ])

    # ── 4. Nav2 navigation stack at 12s ────────────────────────────────────
    controller_server = Node(
        package="nav2_controller", executable="controller_server",
        name="controller_server", output="screen",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
        remappings=[("cmd_vel", "/cmd_vel")],
    )

    planner_server = Node(
        package="nav2_planner", executable="planner_server",
        name="planner_server", output="screen",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
    )

    behavior_server = Node(
        package="nav2_behaviors", executable="behavior_server",
        name="behavior_server", output="screen",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator", executable="bt_navigator",
        name="bt_navigator", output="screen",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
    )

    smoother_server = Node(
        package="nav2_smoother", executable="smoother_server",
        name="smoother_server", output="screen",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
    )

    lifecycle_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": [
                "controller_server", "planner_server",
                "behavior_server", "bt_navigator", "smoother_server",
            ],
        }],
    )

    navigation_group = TimerAction(period=12.0, actions=[
        LogInfo(msg="[Nav2] Starting navigation stack..."),
        controller_server, planner_server, behavior_server,
        bt_navigator, smoother_server, lifecycle_navigation,
    ])

    # ── 5. Butler FSM at 18s ───────────────────────────────────────────────
    state_machine_node = Node(
        package="butler_core",
        executable="state_machine_node.py",
        name="robot_state_machine",
        output="screen",
        parameters=[
            locations_cfg,
            {
                "use_sim_time": use_sim_time,
                "default_kitchen_timeout_sec": 30.0,
                "default_table_timeout_sec":   30.0,
            },
        ],
    )

    order_manager_node = Node(
        package="butler_core",
        executable="order_manager_node.py",
        name="order_manager",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    butler_group = TimerAction(period=18.0, actions=[
        LogInfo(msg="[Butler] Starting FSM nodes..."),
        state_machine_node, order_manager_node,
    ])

    # ── 6. RViz at 8s ─────────────────────────────────────────────────────
    rviz = TimerAction(period=8.0, actions=[
        LogInfo(msg="[RViz] Starting RViz..."),
        Node(
            package="rviz2", executable="rviz2", name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])

    # ── Instructions at 20s ────────────────────────────────────────────────
    instructions = TimerAction(period=20.0, actions=[
        LogInfo(msg=(
            "\n================================================\n"
            " NAVIGATION READY — simple_cafe world\n"
            "================================================\n"
            " 1. In RViz: click '2D Pose Estimate'\n"
            "    Click on the BLUE HOME DOCK at (0.5, 0.0)\n"
            "    facing RIGHT (+X direction)\n"
            "\n"
            " 2. Send an order:\n"
            " source ~/butler_robot_ws/install/setup.bash\n"
            " ros2 run butler_core send_order.py --tables table1\n"
            "\n"
            " 3. Monitor:\n"
            " ros2 topic echo /butler/robot_state\n"
            "================================================\n"
        )),
    ])

    return LaunchDescription([
        use_sim_time_arg, map_arg,
        force_xcb, force_sw,
        LogInfo(msg="=== BUTLER ROBOT — NAVIGATION (simple_cafe) ==="),
        gazebo, robot_state_publisher, spawn_robot,
        localization_group, rviz,
        navigation_group, butler_group,
        instructions,
    ])
