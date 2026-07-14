"""
slam_mapping.launch.py (simple_cafe version)
=============================================
Simple rectangular cafe world — no dividers, no nooks.
Robot spawns at (0.5, 0.0). SLAM starts at 10s (TF race fix).
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
    slam_params       = os.path.join(pkg_nav,    "config", "slam_toolbox_params.yaml")
    rviz_config       = os.path.join(pkg_desc,   "rviz",   "slam_mapping.rviz")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    use_sim_time     = LaunchConfiguration("use_sim_time")

    force_xcb     = SetEnvironmentVariable("QT_QPA_PLATFORM",     "xcb")
    force_sw      = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE","1")

    gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose", world_file,
             "-s", "libgazebo_ros_init.so",
             "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
    )

    spawn_robot = TimerAction(period=4.0, actions=[
        LogInfo(msg="[Gazebo] Spawning robot at (0.5, 0.0)..."),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_butler_robot",
            output="screen",
            arguments=[
                "-entity", "butler_robot",
                "-topic", "/robot_description",
                "-x", "0.5", "-y", "0.0", "-z", "0.1", "-Y", "0.0",
            ],
        ),
    ])

    slam_toolbox = TimerAction(period=10.0, actions=[
        LogInfo(msg="[SLAM] Starting SLAM Toolbox (10s delay — TF race fix)..."),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params, {"use_sim_time": use_sim_time}],
        ),
    ])

    velocity_limiter = TimerAction(period=11.0, actions=[
        LogInfo(msg="[Limiter] Velocity limiter active: 0.15m/s, 0.35rad/s"),
        Node(
            package="butler_core",
            executable="slam_safe_velocity_limiter.py",
            name="slam_safe_velocity_limiter",
            output="screen",
            parameters=[{"max_linear_vel": 0.15, "max_angular_vel": 0.35}],
        ),
    ])

    rviz = TimerAction(period=12.0, actions=[
        LogInfo(msg="[RViz] Starting RViz..."),
        Node(
            package="rviz2", executable="rviz2", name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])

    instructions = TimerAction(period=13.0, actions=[
        LogInfo(msg=(
            "\n================================================\n"
            " SLAM MAPPING READY — simple_cafe world\n"
            "================================================\n"
            " New terminal:\n"
            " ros2 run teleop_twist_keyboard teleop_twist_keyboard\n"
            "     --ros-args -r cmd_vel:=cmd_vel_teleop\n"
            "\n"
            " MAPPING ROUTE:\n"
            "  1. From home dock (blue pad) drive east\n"
            "  2. Follow perimeter: E wall -> N wall -> W wall -> S wall\n"
            "  3. Slow loop around each table cluster\n"
            "  4. Return to home dock to CLOSE THE LOOP\n"
            "\n"
            " Save map when clean:\n"
            " mkdir -p ~/butler_robot_ws/maps\n"
            " ros2 run nav2_map_server map_saver_cli -f maps/cafe_map\n"
            "================================================\n"
        )),
    ])

    return LaunchDescription([
        use_sim_time_arg, force_xcb, force_sw,
        LogInfo(msg="=== BUTLER ROBOT — SLAM MAPPING (simple_cafe) ==="),
        gazebo, robot_state_publisher, spawn_robot,
        slam_toolbox, velocity_limiter, rviz, instructions,
    ])
