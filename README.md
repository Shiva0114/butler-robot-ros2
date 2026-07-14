# Butler Robot – Autonomous Restaurant Service Robot

A ROS2 Humble based autonomous indoor service robot developed for restaurant automation. The project demonstrates autonomous mapping, localization, navigation, and order delivery inside a custom Gazebo simulation environment.

---

## Overview

The Butler Robot is designed to autonomously deliver food between predefined locations inside a restaurant environment.

The complete system was developed using ROS2 Humble and consists of a custom robot model, a custom Gazebo world, Navigation2, SLAM Toolbox, RViz visualization, and a modular software architecture.

The project is intended to simulate the workflow of an autonomous restaurant assistant capable of:

- Mapping an unknown environment
- Localizing itself
- Planning collision-free paths
- Navigating to restaurant tables
- Returning to the charging station
- Managing delivery tasks through a finite state machine

---

# Features

- Custom Differential Drive Robot
- ROS2 Humble
- Gazebo Classic Simulation
- SLAM Toolbox Mapping
- Navigation2
- Autonomous Path Planning
- RViz Visualization
- Custom Restaurant Environment
- Custom ROS Messages, Services and Actions
- Order Management Workflow
- Delivery State Machine

---

# Software Stack

| Component | Technology |
|------------|------------|
| OS | Ubuntu 22.04 |
| Middleware | ROS2 Humble |
| Simulator | Gazebo Classic |
| Visualization | RViz2 |
| Mapping | SLAM Toolbox |
| Navigation | Navigation2 |
| Language | Python |
| Robot Description | URDF/Xacro |

---

# Project Structure

```
butler_robot_ws/

├── src/
│   ├── butler_bringup
│   ├── butler_core
│   ├── butler_description
│   ├── butler_gazebo
│   ├── butler_monitor
│   ├── butler_msgs
│   └── butler_navigation
│
├── maps/
│
├── scripts/
│
├── build/
├── install/
└── log/
```

---

# Package Overview

## butler_bringup

Contains launch files responsible for starting the complete robot system.

Responsibilities

- Launch Gazebo
- Spawn Robot
- Start SLAM
- Launch Navigation

---

## butler_description

Contains the complete robot model.

Includes

- URDF/Xacro
- RViz Configurations

Responsible for robot geometry, links, joints, and sensors.

---

## butler_gazebo

Contains the Gazebo simulation environment.

Includes

- Restaurant world
- Robot spawn configuration

---

## butler_navigation

Contains Navigation2 and SLAM configuration.

Includes

- Nav2 Parameters
- SLAM Toolbox Parameters
- Saved Locations

Responsible for

- Localization
- Planning
- Controller configuration
- Costmaps

---

## butler_core

Implements the robot application logic.

Includes

- State Machine
- Order Manager
- Navigation Bridge
- Delivery Logic
- Velocity Limiter

Responsible for autonomous task execution.

---

## butler_monitor

Responsible for robot monitoring and timeout handling.

---

## butler_msgs

Defines custom ROS interfaces.

Includes

- Custom Messages
- Services
- Actions

Used for communication between nodes.

---

# System Workflow

```
Restaurant Starts

↓

Robot Spawned

↓

Robot Model Loaded

↓

SLAM / Map Loading

↓

Localization

↓

Order Received

↓

Navigate to Kitchen

↓

Navigate to Customer Table

↓

Delivery Complete

↓

Return Home
```

---

# Build

```bash
cd ~/butler_robot_ws

colcon build

source install/setup.bash
```

---

# Mapping

```bash
ros2 launch butler_bringup slam_mapping.launch.py
```

Drive the robot around the restaurant to build the map.

Save the generated map:

```bash
ros2 run nav2_map_server map_saver_cli -f maps/cafe_map
```

---

# Navigation

```bash
ros2 launch butler_bringup navigation_with_map.launch.py
```

The robot loads the saved map and performs autonomous navigation.

---

# Custom ROS Interfaces

### Messages

- RobotState.msg

### Services

- PlaceOrder.srv
- CancelOrder.srv

### Actions

- DeliverOrder.action

---

# Simulation

The project includes a custom restaurant simulation environment featuring:

- Restaurant layout
- Kitchen area
- Dining tables
- Charging station
- Autonomous navigation paths

---

# Future Improvements

- Autonomous docking
- Dynamic obstacle avoidance
- Multi-robot coordination
- Voice command interface
- Vision-based table recognition
- Web dashboard

---

# License

This project was developed for educational and robotics engineering purposes.
