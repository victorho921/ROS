import os
import launch
import xacro
import re
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
# from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue

from xml.etree.ElementTree import fromstring, ParseError
import sys
# from xacro import process_file

def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    # Get packet form share directory
    pkg_gazebo = get_package_share_directory('ros_gz_sim')
    pkg_robot = get_package_share_directory('gazebo_practise')
    pkg_franka_robot = get_package_share_directory('franka_description')


    # Get the urdf.xacro file
    xacro_file = os.path.join(pkg_franka_robot, 'robots', 'fr3', 'fr3.urdf.xacro')

    # Convert Xacro to URDF 
    # robot_description_config = xacro.process_file(
    #     xacro_file 
    #     # mappings={
    #     #     # 'arm_id': arm_id_str, 
    #     #     'arm_id': 'fr3', 
    #     #     # 'hand': load_gripper_str, 
    #     #     'hand': 'true', 
    #     #     'ros2_control': 'false', 
    #     #     'gazebo': 'true', 
    #     #     # 'ee_id': franka_hand_str
    #     #     'ee_id': 'franka_hand'
    #     # }
    # )
    # robot_description = {'robot_description': robot_description_config.toxml()}
    
    robot_description = xacro.process_file(
        xacro_file, 
        mappings={'ros2_control': 'true', 'gazebo': 'true'}
    ).toxml()
    robot_description = remove_comments(robot_description)
    
    output_file = os.path.join(os.getcwd(), "franka_robot.urdf")
    
    with open(output_file, "w") as f:
        f.write(robot_description)  # write the string  
    # print(f"URDF written to: {output_file}")

    # robot_description = ParameterValue(robot_description, value_type=None)

    # try:
    #     with open(urdf_file, 'r') as f:
    #          robot_description = f.read()
    # except FileNotFoundError:
    #     raise RuntimeError(f"URDF file not found at: {urdf_file}")

    # Get Rviz config file path
    rviz_config_path = os.path.join(pkg_robot, 'config', 'my_robot_config.rviz')

    # For publishing robot information
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        
    )

    # # For publishing joint information
    # Only when using Rviz
    # joint_state_publisher = Node(
    #         package='joint_state_publisher_gui',
    #         executable='joint_state_publisher_gui',
    #         name="joint_state_publisher_gui",
    # )

    # Get the sdf file
    world_path = os.path.join(pkg_robot,'worlds','world.sdf')

    # For launching gazebo
    launch_gazebo = ExecuteProcess(
            cmd=['ign', 'gazebo','-v', '4','-r',world_path],    
            output='screen'
    )

    # For launching Rviz
    launch_rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
    )

    # For spawning robot node in gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=['-topic', '/robot_description'],
        # arguments=['-string', sdf_path, '-name', 'fr3'],
        output='screen',
    )

    # For spawning TF node in Rviz

    # For spawning controller manager node 
    # controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[robot_description, os.path.join(pkg_robot, 'controller_config', 'controller.yaml')],
    #     output='screen'
    # )

    # # For launching the custom controller  
    # load_custom_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller',
    #         '--set-state', 'active',
    #         'controller'],
    #     output='screen'
    # )

    # For launching the joint state broadcaster
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_custom_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    ign_bridge_config = os.path.join(pkg_robot, 'config', 'ign_bridge.yaml')
    ft_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ft_sensor_bridge',
        output='screen',
        parameters=[{'config_file': ign_bridge_config}],
    )


    return LaunchDescription([
        robot_state_publisher,
        # joint_state_publisher,
        launch_gazebo,
        spawn_robot,
        launch_rviz,
        ft_bridge,

        load_joint_state_broadcaster,
        load_custom_controller,
    ])

# Single Command to move the robot in gazebo
# ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
#   joint_names: ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7'],
#   points: [
#     {
#       positions: [1.0, -0.785, 1.0, -2.35, 1.0, 1.57, 0.785],
#       time_from_start: {sec: 2, nanosec: 0}
#     }
#   ]
# }"

# If cannot not find visual
# export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:~/RobotArmSim/src



