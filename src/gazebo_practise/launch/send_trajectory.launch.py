# Copyright 2023 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import re
import os
import xacro
import launch
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    # Get URDF via xacro
    pkg_robot = get_package_share_directory('gazebo_practise')
    pkg_franka_robot = get_package_share_directory('franka_description')

    xacro_file = os.path.join(pkg_franka_robot, 'robots', 'fr3', 'fr3.urdf.xacro')

    robot_description = xacro.process_file(
        xacro_file, 
        mappings={'ros2_control': 'true', 'gazebo': 'true'}
    ).toxml()

    robot_description = remove_comments(robot_description)

    send_trajectory_node = Node(
        package="gazebo_practise",
        executable="reference_generator",
        name="send_trajectory_node",
        parameters=[{'robot_description': robot_description}]
    )

    nodes_to_start = [send_trajectory_node]
    return LaunchDescription(nodes_to_start)