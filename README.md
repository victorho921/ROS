# Force & Torque Simulation with Fr3 Robot Arm in Gazebo Ignition

![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange?logo=Ubuntu&labelColor=orange
)
![ROS 2](https://img.shields.io/badge/ROS_2-HUMBLE-blue?logo=ROS)
![Gazebo](https://img.shields.io/badge/Gazebo-Ignition%20Fortress-green)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)

## Overview

This repository contains ROS2 packages for simulating force and torque data using Franka Robotic Arm. 

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

## How to Use

### Prerequisites

- **Ubuntu 22.04** (or compatible Linux distribution)
- **ROS 2 Humble** installed and sourced
- **Gazebo Ignition Fortress** 
- **MoveIt 2** (optional, for motion planning)

### Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/victorho921/RobotArmSim.git
   cd RobotArmSim
   ```

2. **Install dependencies:**
   ```bash
   sudo apt update
   sudo apt install ros-humble-ros-gz ros-humble-moveit
   ```

3. **Build the workspace:**
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build
   source install/setup.bash
   ```

4. **If cannot find model file:**
    ```bash
    export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:~/RobotArmSim/src
    ```

### Running the Simulation

1. **Launch the empty world simulation:**
   ```bash
   ros2 launch gazebo_practise empty_world.launch.py
   ```
   This will:
   - Start Gazebo Ignition with an empty world
   - Spawn the Franka FR3 robot with hand
   - Launch RViz for visualization
   - Start the ROS 2 control stack

2. **Send joint trajectories:**
   ```bash
   ros2 launch gazebo_practise send_trajectory.launch.py
   ```
   This launches a trajectory publisher to control the robot joints.

### Controllers

The project includes several controllers configured in `controller_config/controller.yaml`:

- **Joint State Broadcaster**: Publishes joint states
- **Joint Trajectory Controller**: Executes joint trajectories
- **Force Torque Sensor Broadcaster**: Publishes force/torque sensor data
- **Custom Robot Controller**: Custom C++ controller (currently commented out)

### Topics and Services

- **Joint States**: `/joint_states`
- **Force/Torque Sensor**: `/force_torque_sensor_broadcaster/force_torque_sensor`
- **Trajectory Commands**: `/joint_trajectory_controller/joint_trajectory`

### Customization

- **Robot Description**: Modify `franka_description` package for different Franka models
- **World Files**: Edit `worlds/world.sdf` for different environments
- **Controller Parameters**: Adjust `controller_config/controller.yaml`
- **Launch Files**: Customize `launch/` files for different scenarios

### MoveIt Integration

The `franka_moveit_config` package provides MoveIt 2 configuration for motion planning:

```bash
ros2 launch franka_moveit_config moveit.launch.py
```

## Project Structure

```
RobotArmSim/
├── src/
│   ├── franka_description/     # URDF/SDF robot descriptions
│   ├── franka_moveit_config/   # MoveIt 2 configuration
│   └── gazebo_practise/        # Main simulation package
│       ├── controller/         # Custom ROS 2 controllers
│       ├── launch/             # Launch files
│       ├── worlds/             # Gazebo world files
│       └── config/             # RViz configurations
├── franka_robot.urdf           # Generated URDF file
├── franka_robot.sdf            # Generated SDF file
└── BuildScript.sh              # Build script (currently empty)
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Franka Emika for robot descriptions
- ROS 2 and Gazebo communities
- MoveIt 2 for motion planning framework

