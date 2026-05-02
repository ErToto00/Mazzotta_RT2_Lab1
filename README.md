# Mazzotta_RT2_Lab1

## Overview
This repository contains a ROS2 workspace developed for a navigation task in a 3D Gazebo environment using RViz for visualization. It implements an action server and an interactive action client to control the movement of a simulated custom robot (from `bme_gazebo_sensors`) towards a specified goal using proportional control.

The workspace consists of two main ROS2 packages:
* **`navigation_interfaces`**: Defines the custom action used for the navigation task.
* **`navigation_action_server`**: Contains the C++ implementation of the action server, action client, and the launch file.

## Packages Description

### 1. navigation_interfaces
Contains the definition of the custom action `Navigate.action`:
* **Goal**: Target pose (`float64 x`, `float64 y`, `float64 theta`).
* **Result**: Success flag (`bool success`).
* **Feedback**: Remaining distance to the goal (`float64 distance_remaining`).

### 2. navigation_action_server
Implements the core logic for the navigation task using ROS2 Actions and `tf2` for coordinate frame transformations.

* **`nav_action_server_node`**: An action server that receives target coordinates and publishes velocity commands (`cmd_vel`) to move the robot towards the goal. It features a **hybrid localization approach**: it prioritizes using `tf2` transforms (from `odom` to `base_footprint`) to compute distance and angle errors, but seamlessly falls back to a direct `/odom` topic subscription if the TF tree is delayed or broken. It also listens to a `/shutdown` topic to terminate cleanly.
* **`nav_action_client_node`**: An interactive command-line interface (CLI) to send goals ('g'), cancel the current goal ('c'), or safely shut down the entire system ('q') in real-time.
* **`navigation_launch.py`**: A launch file that automates the execution of the entire system.

## Dependencies
* ROS2 Jazzy
* `ros_gz` (Gazebo Sim - Harmonic)
* `rviz2`
* `robot_state_publisher`
* External repository **`bme_gazebo_sensors`** (automatically symlinked by `start.sh`)
* Standard ROS2 C++ libraries: `rclcpp`, `rclcpp_action`, `rclcpp_components`
* Transform libraries: `tf2`, `tf2_ros`, `tf2_geometry_msgs`
* Message types: `geometry_msgs`, `nav_msgs`, `std_msgs`
* **`gnome-terminal`**: The launch file opens individual nodes in separate terminal windows for better logging and CLI interaction.

## Build and Run Instructions

The easiest way to build and run the entire workspace is by using the provided `start.sh` automation script. This script automatically handles external dependencies (like symlinking `bme_gazebo_sensors`), builds the workspace with `colcon`, sources the environment, and executes the launch file.

1. Navigate to the root of the workspace:
   ```bash
   cd Mazzotta_RT2_Lab1
   ```
2. Make sure the script is executable and run it:
   ```bash
   chmod +x start.sh
   ./start.sh
   ```

This command will launch the Gazebo simulation, spawn the custom robot URDF from `bme_gazebo_sensors`, start RViz2, and open new `gnome-terminal` windows for both the action server and the action client.

### Interacting with the Action Client
Once the client terminal is open, you will be prompted with the CLI:
```text
--- Navigation CLI ---
Enter 'g' to send a new goal, 'c' to cancel current goal, or 'q' to quit:
```
* **Send a goal**: Type `g`, press Enter, then type the target coordinates separated by spaces (e.g., `5.0 2.5 0.0`) and press Enter.
* **Cancel a goal**: Type `c` and press Enter to stop the robot.
* **Quit and Shutdown**: Type `q` and press Enter. This will cancel the active goal, close the server terminal, and cleanly shut down Gazebo and RViz.
