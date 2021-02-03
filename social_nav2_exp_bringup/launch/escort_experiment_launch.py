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
    # ros2 run --prefix 'gdb -ex run --args' nav2_planner planner_server --ros-args -r __node:=planner_server --params-file /home/ubuntu/social_nav2_ws/src/social_navigation2/social_navigation_bringup/params/nav2_params.yaml

    # Get the launch directory
    exp_bringup_dir = get_package_share_directory('social_navigation_exp_bringup')

    scene_file = LaunchConfiguration('scene_file')
    simulation_factor = LaunchConfiguration('simulation_factor')

    social_bringup_dir = get_package_share_directory('social_navigation_bringup')
    launch_dir = os.path.join(social_bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    frame_id = LaunchConfiguration('frame_id')
    declare_scene_file_cmd = DeclareLaunchArgument(
        'scene_file', 
        default_value=os.path.join(exp_bringup_dir, 'scenarios', 'escorting.xml'),
        description='')
    declare_simulation_factor_cmd = DeclareLaunchArgument(
        'simulation_factor', default_value='0.15',
        description='Simulator factor. 0.0 to get static agents')
    declare_frame_id_cmd = DeclareLaunchArgument(
        'frame_id', default_value='map', description='Reference frame')

    social_nav_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'social_nav_launch.py')))

    #escort_controller_cmd = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(os.path.join(
    #        get_package_share_directory('social_navigation_actions'),
    #        'launch',
    #        'escort_controller.py'))
    #    )
    
    social_goal_updater_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('social_navigation_actions'),
            'launch',
            'social_goal_updaters_launch.py'))
        )

    escort_controller_cmd  = Node(
        package='social_nav2_experiments',
        executable='social_nav2_hri',
        name='social_nav2_hri',
        output='screen')
        
    escort_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            exp_bringup_dir, 
            'launch',
            'escort_sim_launch.py')),
        launch_arguments={'scene_file': scene_file,
                          'simulation_factor': simulation_factor}.items())

    distance_to_agent_cmd = Node(
        package='measuring_tools',
        executable='distance_to_agent_node',
        name='distance_to_agent_node',
        output='screen',
        arguments=["agent_1"])

    robot_cost_cmd = Node(
        package='measuring_tools',
        executable='robot_cost_node',
        name='robot_cost_node',
        output='screen')
    
    path_cmd = Node(
        package='measuring_tools',
        executable='path_pub_node',
        name='path_pub_node',
        output='screen')
    
    topics_2_csv_cmd = Node(
        package='social_navigation_csv',
        executable='exp2_topics_2_csv',
        name='exp2_topics_2_csv',
        output='screen')
    
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_scene_file_cmd)
    ld.add_action(declare_simulation_factor_cmd)
    ld.add_action(declare_frame_id_cmd)

    ld.add_action(distance_to_agent_cmd)
    ld.add_action(path_cmd)
    #ld.add_action(topics_2_csv_cmd)    
    #ld.add_action(escort_controller_cmd)
    ld.add_action(social_goal_updater_cmd)

    ld.add_action(escort_sim)
    ld.add_action(social_nav_bringup_cmd)
    return ld
