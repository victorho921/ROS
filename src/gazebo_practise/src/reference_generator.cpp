// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/chainiksolvervel_pinv.hpp>
// #include <kdl/jntarray.hpp>
// #include <kdl/tree.hpp>
// #include <kdl_parser/kdl_parser.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <trajectory_msgs/msg/joint_trajectory.hpp>
// #include <trajectory_msgs/msg/joint_trajectory_point.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>


// class TrajectoryGenerator : public rclcpp::Node
// {
// public:
//   TrajectoryGenerator() : Node("send_trajectory")
//   {
//     // Create a publisher that send message to the controller node
//     pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
//       "/joint_trajectory_controller/JointTrajectoryController", 10); // The controller you wanna use
//     RCLCPP_INFO(this->get_logger(), "Initialization");

//     // Subscribe to joint states to get current positions
//     joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
//       "/joint_states", 10, std::bind(&TrajectoryGenerator::jointStateCallback, this, std::placeholders::_1));

//     // get robot description
//     this->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
//     auto robot_param = this->get_parameter("robot_description");
//     auto robot_description = robot_param.as_string();
//     RCLCPP_INFO(this->get_logger(), "Successfully retrieve robot description");

//     // create kinematic chain
//     KDL::Tree robot_tree;
//     KDL::Chain chain;
//     kdl_parser::treeFromString(robot_description, robot_tree);
//     robot_tree.getChain("fr3_link0", "fr3_link7", chain); // make sure the name of the first link same as the one in URDF

//     joint_positions_ = KDL::JntArray(chain.getNrOfJoints());
//     joint_velocities_ = KDL::JntArray(chain.getNrOfJoints());
//     twist_ = KDL::Twist();
//     RCLCPP_INFO(this->get_logger(), "Successfully create kinematic chain");

//     // create KDL solvers
//     ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain, 0.0000001);

//     trajectory_msg_.joint_names = {
//       "fr3_joint1",
//       "fr3_joint2",
//       "fr3_joint3",
//       "fr3_joint4",
//       "fr3_joint5",
//       "fr3_joint6",
//       "fr3_joint7"
//     };

//     // Wait for initial joint states
//     while (!initial_positions_received_ && rclcpp::ok()) {
//       rclcpp::spin_some(this->get_node_base_interface());
//       rclcpp::sleep_for(std::chrono::milliseconds(100));
//     }

//     if (!rclcpp::ok()) return;

//     generateAndPublishTrajectory();
//   }

// private:
//   void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
//   {
//     if (initial_positions_received_) return;

//     // Map joint names to positions
//     for (size_t i = 0; i < trajectory_msg_.joint_names.size(); ++i) {
//       auto it = std::find(msg->name.begin(), msg->name.end(), trajectory_msg_.joint_names[i]);
//       if (it != msg->name.end()) {
//         size_t idx = std::distance(msg->name.begin(), it);
//         joint_positions_(i) = msg->position[idx];
//       }
//     }
//     initial_positions_received_ = true;
//     RCLCPP_INFO(this->get_logger(), "Received initial joint positions");
//   }

//   void generateAndPublishTrajectory()
//   {
//     trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
//     trajectory_point_msg.positions.resize(joint_positions_.rows());
//     trajectory_point_msg.velocities.resize(joint_positions_.rows());

//     double total_time = 3.0;
//     int trajectory_len = 200;
//     double dt = total_time / static_cast<double>(trajectory_len - 1);

//     trajectory_msg_.header.stamp = this->now();

//     for (int i = 0; i < trajectory_len; i++)
//     {
//       // set endpoint twist
//       double t = i / (static_cast<double>(trajectory_len - 1));
//       twist_.vel.x(2 * 0.3 * cos(2 * M_PI * t));
//       twist_.vel.y(-0.3 * sin(2 * M_PI * t));

//       // convert cart to joint velocities
//       ik_vel_solver_->CartToJnt(joint_positions_, twist_, joint_velocities_);

//       // copy to trajectory_point_msg
//       std::memcpy(
//         trajectory_point_msg.positions.data(), joint_positions_.data.data(),
//         trajectory_point_msg.positions.size() * sizeof(double));
//       std::memcpy(
//         trajectory_point_msg.velocities.data(), joint_velocities_.data.data(),
//         trajectory_point_msg.velocities.size() * sizeof(double));

//       // integrate joint velocities
//       joint_positions_.data += joint_velocities_.data * dt;

//       // set timing information
//       double time_point = total_time * t;
//       double time_point_sec = std::floor(time_point);
//       trajectory_point_msg.time_from_start.sec = static_cast<int>(time_point_sec);
//       trajectory_point_msg.time_from_start.nanosec =
//         static_cast<int>((time_point - time_point_sec) * 1E9);
//       trajectory_msg_.points.push_back(trajectory_point_msg);
//     }

//     // send zero velocities in the end
//     auto & last_point_msg = trajectory_msg_.points.back();
//     std::fill(last_point_msg.velocities.begin(), last_point_msg.velocities.end(), 0.0);

//     pub_->publish(trajectory_msg_);
//     RCLCPP_INFO(this->get_logger(), "Published trajectory with %zu points", trajectory_msg_.points.size());
//   }

//   rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
//   rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
//   std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
//   KDL::JntArray joint_positions_;
//   KDL::JntArray joint_velocities_;
//   KDL::Twist twist_;
//   trajectory_msgs::msg::JointTrajectory trajectory_msg_;
//   bool initial_positions_received_ = false;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<TrajectoryGenerator>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }



#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


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
      trajectory_msg.joint_names.push_back(joint.getName());
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

  // ====================== Wait for real joint states ======================
  std::vector<double> current_positions;
  bool received_state = false;

  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
          if (!received_state && msg->position.size() >= 7) {
              current_positions = msg->position;
              received_state = true;
              RCLCPP_INFO(node->get_logger(), "Received joint states with %zu joints", msg->position.size());
          }
      });

  RCLCPP_INFO(node->get_logger(), "Waiting for joint states from /joint_states...");
  while (rclcpp::ok() && !received_state) {
      rclcpp::spin_some(node);
      rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_INFO(node->get_logger(), "Successfully received initial joint state!");

  trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
  trajectory_point_msg.positions.resize(chain.getNrOfJoints());
  trajectory_point_msg.velocities.resize(chain.getNrOfJoints());

  double total_time = 1.0;
  int trajectory_len = 200;
  double dt = total_time / static_cast<double>(trajectory_len - 1);

  // for (int i = 0; i < trajectory_len; i++)
  // {
  //   // set endpoint twist
  //   double t = i / (static_cast<double>(trajectory_len - 1));
  //   twist.vel.x(2 * 0.3 * cos(2 * M_PI * t));
  //   twist.vel.y(-0.3 * sin(2 * M_PI * t));

  //   // convert cart to joint velocities
  //   // Input: Current joint position and twist
  //   // Output: Joint velocity
  //   ik_vel_solver_->CartToJnt(joint_positions, twist, joint_velocities);

  //   // copy to trajectory_point_msg
  //   // std::memcpy(
  //   //   trajectory_point_msg.positions.data(), joint_positions.data.data(),
  //   //   trajectory_point_msg.positions.size() * sizeof(double));
  //   // std::memcpy(
  //   //   trajectory_point_msg.velocities.data(), joint_velocities.data.data(),
  //   //   trajectory_point_msg.velocities.size() * sizeof(double));

  //   for (unsigned int j = 0; j < chain.getNrOfJoints(); j++) 
  //   {
  //     trajectory_point_msg.positions[j] = joint_positions.data(j);
  //     trajectory_point_msg.velocities[j] = joint_velocities.data(j);
  //   }

  //   // integrate joint velocities
  //   joint_positions.data += joint_velocities.data * dt;

  //   // set timing information
  //   double time_point = total_time * t;
  //   double time_point_sec = std::floor(time_point);
  //   trajectory_point_msg.time_from_start = rclcpp::Duration::from_seconds(i * dt);
  //   // trajectory_point_msg.time_from_start.sec = static_cast<int>(time_point_sec);
  //   trajectory_point_msg.time_from_start.nanosec = static_cast<int>((time_point - time_point_sec) * 1E9);
  //   trajectory_msg.points.push_back(trajectory_point_msg);
  // }

  // For Debug
  // Clear and add first point at current position
trajectory_msg.points.clear();

trajectory_msgs::msg::JointTrajectoryPoint point;
point.positions.resize(chain.getNrOfJoints());
point.velocities.resize(chain.getNrOfJoints(), 0.0);
for (unsigned int j = 0; j < chain.getNrOfJoints(); ++j) {
    point.positions[j] = joint_positions.data(j);  // current position
}
// first_point.velocities.assign(chain.getNrOfJoints(), 0.0);
point.time_from_start = rclcpp::Duration::from_seconds(0.0);
trajectory_msg.points.push_back(point);

RCLCPP_INFO(node->get_logger(), "=== FIRST POINT (Current Position) ===");
RCLCPP_INFO(node->get_logger(), "Time from start: 0.0 s");
for (unsigned int j = 0; j < chain.getNrOfJoints(); ++j) {
    RCLCPP_INFO(node->get_logger(), "  Joint %d (%s): position = %.4f rad, velocity = %.4f rad/s",
                j, 
                trajectory_msg.joint_names[j].c_str(),
                point.positions[j],
                point.velocities[j]);
}
RCLCPP_INFO(node->get_logger(), "==================================================");

// Then your loop for the actual motion (i=1 to trajectory_len)
for (int i = 1; i < trajectory_len; ++i) {
    double t_normalized = static_cast<double>(i) / (trajectory_len - 1);

    twist.vel.x(0.6 * cos(2 * M_PI * t_normalized));  // reduced speed
    twist.vel.y(-0.3 * sin(2 * M_PI * t_normalized));

    ik_vel_solver_->CartToJnt(joint_positions, twist, joint_velocities);
    joint_positions.data += joint_velocities.data * dt;

    point.positions.resize(chain.getNrOfJoints());
    point.velocities.assign(chain.getNrOfJoints(), 0.0);  // or keep the solved vel

    for (unsigned int j = 0; j < chain.getNrOfJoints(); ++j) {
        point.positions[j] = joint_positions.data(j);
    }
    point.time_from_start = rclcpp::Duration::from_seconds(i * dt);

    trajectory_msg.points.push_back(point);
}

  // send zero velocities in the end
  auto & last_point_msg = trajectory_msg.points.back();
  std::fill(last_point_msg.velocities.begin(), last_point_msg.velocities.end(), 0.0);

  // // DEBUG
  // RCLCPP_INFO(node->get_logger(), "Publishing trajectory with %zu points", trajectory_msg.points.size());

  pub->publish(trajectory_msg);


  
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

  rclcpp::spin_some(node); 
  rclcpp::sleep_for(std::chrono::seconds(2)); // Give it time to hand off the message
  rclcpp::shutdown();

  return 0;
}