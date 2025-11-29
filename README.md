# Gazebo Practise: FR3 Robot Simulation with ROS 2 & Ignition Gazebo

![FR3 Robot in Gazebo](https://img.shields.io/badge/FR3%20Robot-Simulation-success?style=for-the-badge)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Iron%20%7C%20Jazzy-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Ignition%20Fortress%20%7C%20Garden-green)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)

A **fully functional simulation** of the **Franka Emika FR3** robot arm using **ROS 2**, **Ignition Gazebo**, and **ros2_control**.  
Includes joint control, trajectory execution, force/torque sensing, and GUI joint manipulation.

---

## Features

- **FR3 7-DOF Arm + Hand** (from `franka_description`)
- **ROS 2 Control Stack**:
  - `joint_state_broadcaster`
  - `joint_trajectory_controller`
  - Custom C++ controller (`CustomRobotController`)
  - Force/torque sensor broadcaster
- **Ignition Gazebo Integration** (physics, sensors, spawning)
- **GUI Joint Control** via `joint_state_publisher_gui`
- **RViz Visualization** (pre-configured)
- **MoveIt 2 Ready** (optional setup)

---

## Project Structure

