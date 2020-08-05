# Copyright (c) 2018 Intel Corporation
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

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    social_bringup_dir = get_package_share_directory('social_navigation_bringup')
    launch_dir = os.path.join(social_bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    
    social_nav_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'tb3_house_simulation_launch.py')))

    dummytf2_cmd = Node(
        package='measuring_tools',
        node_executable='dummy_tf2_node',
        node_name='dummy_tf2_node',
        output='screen')

    distance_to_agent_cmd = Node(
        package='measuring_tools',
        node_executable='distance_to_agent_node',
        node_name='distance_to_agent_node',
        output='screen',
        arguments=["agent_3"])

    robot_cost_cmd = Node(
        package='measuring_tools',
        node_executable='robot_cost_node',
        node_name='robot_cost_node',
        output='screen')
    
    robot_path_cmd = Node(
        package='measuring_tools',
        node_executable='path_pub_node',
        node_name='path_pub_node',
        output='screen')
    
    topics_2_csv_cmd = Node(
        package='social_navigation_csv',
        node_executable='topics_2_csv',
        node_name='topics_2_csv',
        output='screen')
    
    agent_spawner_cmd = Node(
        package='pedsim_gazebo_plugin',
        node_executable='spawn_single_agent.py',
        node_name='spawn_single_agent',
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(social_nav_bringup_cmd)
    ld.add_action(dummytf2_cmd)
    ld.add_action(robot_cost_cmd)
    ld.add_action(distance_to_agent_cmd)
    ld.add_action(robot_path_cmd)
    # ld.add_action(topics_2_csv_cmd)
    ld.add_action(agent_spawner_cmd)
    return ld
