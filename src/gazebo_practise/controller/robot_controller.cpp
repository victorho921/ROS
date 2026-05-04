// Copyright 2026, victorho921
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Phrase 1
// Target: Set a target position -> Monitor Force -> If Force > Threshold, hold position

#include "include/robot_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

// #include "rclcpp/qos.hpp"
// #include "rclcpp/time.hpp"
// #include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
// #include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace Custom_Franka_Controller
{
HybridFTController::HybridFTController() : controller_interface::ControllerInterface() {}


// Read out joint name, command and state interface
controller_interface::CallbackReturn HybridFTController::on_init()
{
  // should have error handling
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ =
    auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ =
    auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

  // Set the initial point of trajectory to be 0 (both joint position and joint velocity)
  point_interp_.positions.assign(joint_names_.size(), 0);
  point_interp_.velocities.assign(joint_names_.size(), 0);

  // Set default virtual stiffness and damping gains for end effector
  const double default_kp = 100.0;  // N/m
  const double default_kd = 20.0;   // N·s/m
  auto_declare<double>("stiffness.trans_x", default_kp);
  auto_declare<double>("stiffness.trans_y", default_kp);
  auto_declare<double>("stiffness.trans_z", default_kp);
  auto_declare<double>("stiffness.rot_x", default_kp);
  auto_declare<double>("stiffness.rot_y", default_kp);
  auto_declare<double>("stiffness.rot_z", default_kp);
  return CallbackReturn::SUCCESS;
}


// Both command & state interface method is to declare variable (similar) of every joint with command or state
// e.g: joint1/command, joint2/state
controller_interface::InterfaceConfiguration HybridFTController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : command_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::InterfaceConfiguration HybridFTController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : state_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

// defining a callback 
controller_interface::CallbackReturn HybridFTController::on_configure(const rclcpp_lifecycle::State &)
{
  // Get list of joints from parameters
  auto joints = get_node()->get_parameter("joints").as_string_array();
  num_joints_ = joints.size();

  q_.resize(num_joints_);
  dq_.resize(num_joints_);

  // Initialize control gains and positions
  kp_.resize(num_joints_, 100.0);
  kd_.resize(num_joints_, 20.0);
  kp_hold_.resize(num_joints_, 200.0);
  target_pos_.resize(num_joints_, 0.0);
  hold_pos_.resize(num_joints_, 0.0);

  // Initialize sensor vectors
  forces_.resize(3, 0.0);
  torques_.resize(3, 0.0);

  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();

  // pass the message from the subscription to the control loop
  // bascially the input of the motion
  auto callback =
    [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg) -> void
  {
    traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
    new_msg_ = true;
  };

  // create subscription to the trajectory node
  // listen to the message from that node
  joint_command_subscriber_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HybridFTController::on_activate(const rclcpp_lifecycle::State &)
{
  // clear out vectors in case of restart
  joint_position_command_interface_.clear();  
  joint_velocity_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();
  // sensor_interfaces_.clear();



  // assign command interfaces
  // for (auto & interface : command_interfaces_)
  // {

    // command_interface_map_[interface.get_interface_name()]->push_back(interface);
  // }
  // I DOUBT THIS WORK
  for (auto & interface : command_interfaces_)
    {
        const std::string interface_type = interface.get_interface_name();  // "position", "velocity", etc.
        auto it = command_interface_map_.find(interface_type);
        if (it != command_interface_map_.end())
        {
            it->second->push_back(interface);   // push into the correct vector
        }
    }

  // assign state interfaces
  // for (auto & interface : state_interfaces_)
  // {
  //   if (interface.get_name().find("force_torque_sensor/") == 0) {
  //     // Sensor interface
  //     sensor_interfaces_[interface.get_interface_name()] = std::ref(interface);
  //   } else {
  //     // Joint interface
  //     state_interface_map_[interface.get_interface_name()]->push_back(interface);
  //   }
  // }

  for (auto & interface : state_interfaces_)
  {
      const std::string interface_name = interface.get_name();
      const std::string interface_type = interface.get_interface_name();  // position/velocity/effort
      auto it = state_interface_map_.find(interface_type);
      // state_interfaces_[interface.get_interface_name()] = std::ref(interface);
      if (it != state_interface_map_.end())
      {
          it->second->push_back(interface);   // push into the correct vector
      }
  }

  auto wrench_callback = [this](const geometry_msgs::msg::Wrench::SharedPtr wrench) 
  {
      latest_wrench_ = wrench;
  };

  ft_sensor_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Wrench>(
    "~/ft_sensor_data", rclcpp::SystemDefaultsQoS(), wrench_callback);
    // [this](const geometry_msgs::msg::Wrench::SharedPtr msg) {
    //   wrench_raw_(0) = msg->force.x;
    //   wrench_raw_(1) = msg->force.y;
    //   wrench_raw_(2) = msg->force.z;
    //   wrench_raw_(3) = msg->torque.x;
    //   wrench_raw_(4) = msg->torque.y;
    //   wrench_raw_(5) = msg->torque.z;
    //   const double alpha = 0.1; // Low-pass filter coefficient
    //   wrench_filtered_ = alpha * wrench_raw_ + (1 - alpha) * wrench_filtered_;
    // });


  // Safty check for the interfaces
  if (joint_position_command_interface_.empty() && joint_position_state_interface_.empty())
  {
      RCLCPP_ERROR(get_node()->get_logger(), "No position interfaces found!");
      return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

void interpolate_point(
  const trajectory_msgs::msg::JointTrajectoryPoint & point_1,
  const trajectory_msgs::msg::JointTrajectoryPoint & point_2,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp, double delta)
{
  for (size_t i = 0; i < point_1.positions.size(); i++)
  {
    point_interp.positions[i] = delta * point_2.positions[i] + (1.0 - delta) * point_2.positions[i];
  }
  for (size_t i = 0; i < point_1.positions.size(); i++)
  {
    point_interp.velocities[i] =
      delta * point_2.velocities[i] + (1.0 - delta) * point_2.velocities[i];
  }
}

void interpolate_trajectory_point(
  const trajectory_msgs::msg::JointTrajectory & traj_msg, const rclcpp::Duration & cur_time,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp)
{
  double traj_len = traj_msg.points.size();
  auto last_time = traj_msg.points[traj_len - 1].time_from_start;
  double total_time = last_time.sec + last_time.nanosec * 1E-9;

  size_t ind = cur_time.seconds() * (traj_len / total_time);
  ind = std::min(static_cast<double>(ind), traj_len - 2);
  double delta = cur_time.seconds() - ind * (total_time / traj_len);
  interpolate_point(traj_msg.points[ind], traj_msg.points[ind + 1], point_interp, delta);
}

controller_interface::return_type HybridFTController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Read current joint position and velocity from the state interfaces
  // for (size_t i = 0; i < num_joints_; ++i)
  // {
  //   q_(i) = joint_position_state_interface_[i].get().get_value();
  //   dq_(i) = joint_velocity_state_interface_[i].get().get_value();
  // }

  // Filter the z-force data
  if (latest_wrench_)
  {
      forces_[0] = latest_wrench_->force.x;
      forces_[1] = latest_wrench_->force.y;
      forces_[2] = latest_wrench_->force.z;

      torques_[0] = latest_wrench_->torque.x;
      torques_[1] = latest_wrench_->torque.y;
      torques_[2] = latest_wrench_->torque.z;
  }

  double filtered_force_z_ = 0.1 * forces_[2] + 0.9 * filtered_force_z_; // Simple low-pass filter
  double force_threshold_ = 5.0; // Threshold for contact detection (adjust as needed)

  // === Force filtering (Z-axis) ===
  // force_buffer_.push_back(forces_[2]);
  // if (force_buffer_.size() > buffer_size_)
  //     force_buffer_.pop_front();

  // double filtered_force_z = 0.0;
  // if (!force_buffer_.empty())
  // {
  //     filtered_force_z = std::accumulate(force_buffer_.begin(), force_buffer_.end(), 0.0) 
  //                         / force_buffer_.size();
  // }

  // Check the current state
  switch (current_state_) {
        case State::MOVING:
            // Calculate Movement Torques (PD Control: Kp*error + Kd*d_error)
            // This moves the robot toward the target_position_
            for (size_t i = 0; i < joint_position_state_interface_[i].get().get_value(); ++i) {
                double pos_err = target_pos_[i] - joint_position_state_interface_[i].get().get_value();
                double vel_err = 0.0 - joint_velocity_state_interface_[i].get().get_value();
                // double effort = kp_[i] * pos_err + kd_[i] * vel_err;
                
                // joint_effort_command_interface_[i].get().set_value(effort);
            }

            // Check for Contact on z-axis
            if (std::abs(filtered_force_z_) > force_threshold_) {
                current_state_ = State::CONTACT;
                // Lock the joints at their current position upon contact
                for (size_t i = 0; i < joint_position_state_interface_.size(); ++i) {
                    hold_pos_[i] = joint_position_state_interface_[i].get().get_value(); 
                }
            }
            break;

        case State::CONTACT:
            // Holding Logic: Keep the joints at the position where they made contact
            break;
    }


  // Trajectory interpolation logic (if needed)
  if (new_msg_)
  {
    trajectory_msg_ = *traj_msg_external_point_ptr_.readFromRT();
    start_time_ = time;
    new_msg_ = false;
  }
  if (trajectory_msg_ != nullptr)
  {
    interpolate_trajectory_point(*trajectory_msg_, time - start_time_, point_interp_);
    for (size_t i = 0; i < joint_position_command_interface_.size(); i++)
    {
      joint_position_command_interface_[i].get().set_value(point_interp_.positions[i]);
      target_pos_[i] = point_interp_.positions[i]; // Update target position for hybrid control
    }
    for (size_t i = 0; i < joint_velocity_command_interface_.size(); i++)
    {
      joint_velocity_command_interface_[i].get().set_value(point_interp_.velocities[i]);
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn HybridFTController::on_deactivate(const rclcpp_lifecycle::State &)
{
  release_interfaces();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HybridFTController::on_cleanup(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HybridFTController::on_error(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HybridFTController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}




// void JointImpedanceExampleController::updateJointStates()
// {
//   for (size_t i = 0; i < num_joints_; ++i)
//   {
//     q_(i)  = joint_position_interfaces_[i].get().get_value();
//     dq_(i) = joint_velocity_interfaces_[i].get().get_value();
//   }
// }


}  // namespace Custom_Franka_Controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  Custom_Franka_Controller::HybridFTController, controller_interface::ControllerInterface)
  