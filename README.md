# Mazzotta_RT2_Lab1

## Overview
This repository contains a ROS2 workspace developed for a navigation task in a 3D Gazebo environment using RViz for visualization. It implements an action server and an interactive action client to control the movement of a simulated custom robot (from `Recchiuto_world`) towards a specified goal using proportional control.

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

* **`nav_action_server_node`**: An action server that receives target coordinates and publishes velocity commands (`cmd_vel`) to move the robot towards the goal. It computes distance and angle errors by continuously monitoring the robot's pose via `tf2` transforms (from `odom` to `link_chassis`).
* **`nav_action_client_node`**: An interactive command-line interface (CLI) to send goals (x, y, theta) or cancel the current goal in real-time.
* **`navigation_launch.py`**: A launch file that automates the execution of the entire system.

## Dependencies
* ROS2 Jazzy
* `ros_gz` (Gazebo Sim)
* `rviz2`
* `robot_state_publisher`
* External repository `Recchiuto_world` must be located at `C:\GitHubRepos\Recchiuto_world`
* Standard ROS2 C++ libraries: `rclcpp`, `rclcpp_action`, `rclcpp_components`
* Transform libraries: `tf2`, `tf2_ros`
* Message types: `geometry_msgs`
* **`terminator`**: The launch file uses `terminator -x` to open individual nodes in separate terminal windows for better logging and CLI interaction.

## Build Instructions
1. Navigate to the root of the workspace:
   ```bash
   cd Mazzotta_RT2_Lab1
   ```
2. Build the packages using `colcon`:
   ```bash
   colcon build
   ```
3. Source the setup script:
   ```bash
   source install/setup.bash
   ```

## How to Run

To launch the entire simulation and the related nodes, use the provided launch file:
```bash
ros2 launch navigation_action_server navigation_launch.py
```
This command will launch an empty Gazebo simulation, spawn the custom robot URDF from the external `Recchiuto_world` repository, start RViz2, and open new `terminator` windows for both the action server and the action client.

### Interacting with the Action Client
Once the client terminal is open, you will be prompted with the CLI:
```text
--- Navigation CLI ---
Enter 'x y theta' to send a new target, or 'cancel' to cancel current goal:
```
* **Send a goal**: Type the target coordinates separated by spaces (e.g., `5.0 5.0 1.57`) and press Enter.
* **Cancel a goal**: Type `cancel` and press Enter to stop the robot.
