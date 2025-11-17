// Copyright 2023 ros2_control Development Team
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

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Create a node called send_trajectory
  auto node = std::make_shared<rclcpp::Node>("send_trajectory");
  // Create a publisher that send message to the controller node 
  auto pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint_trajectory_controller/joint_trajectory", 10); // The controller you wanna use
  RCLCPP_INFO(node->get_logger(), "Initization");

  // get robot description
  auto robot_param = rclcpp::Parameter();
  node->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
  node->get_parameter("robot_description", robot_param);
  auto robot_description = robot_param.as_string();
  RCLCPP_INFO(node->get_logger(), "Successfully retrieve robot description");

  // create kinematic chain
  KDL::Tree robot_tree;
  KDL::Chain chain;
  kdl_parser::treeFromString(robot_description, robot_tree);
  // DEBUG
  // bool tree_ok = kdl_parser::treeFromString(robot_description, robot_tree);
  // RCLCPP_INFO(node->get_logger(), "treeFromString success: %s", tree_ok ? "true" : "false");

  robot_tree.getChain("fr3_link0", "fr3_link7", chain); // make sure the name of the first link same as the one in URDF
  // DEBUG
  // bool chain_ok = robot_tree.getChain("fr3_link0", "fr3_link7", chain);
  // RCLCPP_INFO(node->get_logger(), "getChain success: %s", chain_ok ? "true" : "false");

  auto joint_positions = KDL::JntArray(chain.getNrOfJoints());
  auto joint_velocities = KDL::JntArray(chain.getNrOfJoints());
  auto twist = KDL::Twist();
  RCLCPP_INFO(node->get_logger(), "Successfully create kinematic chain");
  // RCLCPP_INFO(node->get_logger(), "Number of joint from chain is %u", chain.getNrOfJoints()); // It returns 7 so OK!!! 

  // create KDL solvers
  auto ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain, 0.0000001);

  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = node->now();
  for (unsigned int i = 0; i < chain.getNrOfSegments(); i++)
  {
    auto joint = chain.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::Fixed)
    {
      // RCLCPP_INFO(node->get_logger(),"%s",joint.getName().c_str());
  //     trajectory_msg.joint_names.push_back(joint.getName());
    }
  }
  trajectory_msg.joint_names = {
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7"
  };
  
  // DEBUG
  // print joint name 
  // RCLCPP_INFO(node->get_logger(), "Joint names in trajectory message:");
  // for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
  //   RCLCPP_INFO(node->get_logger(), "No of segement = %u", chain.getNrOfSegments());
  //   const auto& joint = chain.getSegment(i).getJoint();
  //   auto joint_name = joint.getName();
  //   RCLCPP_INFO(node->get_logger(), "Joint [%u]: '%s' (length: %zu), type: %d", 
  //             i, joint_name.c_str(), joint_name.size(), joint.getType());
  // }


  trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
  trajectory_point_msg.positions.resize(chain.getNrOfJoints());
  trajectory_point_msg.velocities.resize(chain.getNrOfJoints());

  double total_time = 3.0;
  int trajectory_len = 200;
  double dt = total_time / static_cast<double>(trajectory_len - 1);

  for (int i = 0; i < trajectory_len; i++)
  {
    // set endpoint twist
    double t = i / (static_cast<double>(trajectory_len - 1));
    twist.vel.x(2 * 0.3 * cos(2 * M_PI * t));
    twist.vel.y(-0.3 * sin(2 * M_PI * t));

    // convert cart to joint velocities
    // Input: Current joint position and twist
    // Output: Joint velocity
    ik_vel_solver_->CartToJnt(joint_positions, twist, joint_velocities);

    // copy to trajectory_point_msg
    std::memcpy(
      trajectory_point_msg.positions.data(), joint_positions.data.data(),
      trajectory_point_msg.positions.size() * sizeof(double));
    std::memcpy(
      trajectory_point_msg.velocities.data(), joint_velocities.data.data(),
      trajectory_point_msg.velocities.size() * sizeof(double));

    // integrate joint velocities
    joint_positions.data += joint_velocities.data * dt;

    // set timing information
    double time_point = total_time * t;
    double time_point_sec = std::floor(time_point);
    trajectory_point_msg.time_from_start.sec = static_cast<int>(time_point_sec);
    trajectory_point_msg.time_from_start.nanosec =
      static_cast<int>((time_point - time_point_sec) * 1E9);
    trajectory_msg.points.push_back(trajectory_point_msg);
  }

  // send zero velocities in the end
  auto & last_point_msg = trajectory_msg.points.back();
  std::fill(last_point_msg.velocities.begin(), last_point_msg.velocities.end(), 0.0);

  // DEBUG
  // RCLCPP_INFO(node->get_logger(), "Publishing trajectory with %zu points", trajectory_msg.points.size());

  pub->publish(trajectory_msg);

  // trajectory_msgs::msg::JointTrajectoryPoint point;
  // point.positions = {0.1, 0.2, 0.1, 0.0, -0.1, -0.2, 0.0};  // 7 values
  // point.time_from_start = rclcpp::Duration::from_seconds(2.0);
  // trajectory_msg.points.push_back(point);

  // pub->publish(trajectory_msg);

  
  // // Simple Trajectory
  // trajectory_msgs::msg::JointTrajectory traj;
  // traj.joint_names = {
  //   "fr3_joint1",
  //   "fr3_joint2",
  //   "fr3_joint3",
  //   "fr3_joint4",
  //   "fr3_joint5",
  //   "fr3_joint6",
  //   "fr3_joint7"
  // };

  // trajectory_msgs::msg::JointTrajectoryPoint point;
  // point.positions = {0.1, 0.2, 0.1, 0.0, -0.1, -0.2, 0.0};  // 7 values
  // point.time_from_start = rclcpp::Duration::from_seconds(2.0);
  // traj.points.push_back(point);
  // pub->publish(traj);

  
  while (rclcpp::ok())
  {
  }

  return 0;
}