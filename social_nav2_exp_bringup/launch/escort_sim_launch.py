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
    # ros2 run --prefix 'gdb -ex run --args' nav2_planner planner_server --ros-args -r __node:=planner_server --params-file /home/ubuntu/social_nav2_ws/src/social_nav2/social_nav2_bringup/params/nav2_params.yaml

    # Get the launch directory
    exp_bringup_dir = get_package_share_directory('social_nav2_exp_bringup')

    scene_file = LaunchConfiguration('scene_file')
    simulation_factor = LaunchConfiguration('simulation_factor')

    social_bringup_dir = get_package_share_directory('social_nav2_bringup')
    launch_dir = os.path.join(social_bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    frame_id = LaunchConfiguration('frame_id')
    declare_scene_file_cmd = DeclareLaunchArgument(
        'scene_file', 
        default_value=os.path.join(exp_bringup_dir, 'scenarios', 'escorting.xml'),
        description='')

    declare_simulation_factor_cmd = DeclareLaunchArgument(
        'simulation_factor', default_value='0.1',
        description='Simulator factor. 0.0 to get static agents')
    declare_frame_id_cmd = DeclareLaunchArgument(
        'frame_id', default_value='map', description='Reference frame')

    sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(exp_bringup_dir, 'launch', 'sim_launch.py')),
        launch_arguments={'headless': "True"}.items()
    )

    pedsim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('pedsim_simulator'), 
            'launch',
            'simulator_launch.py')),
        launch_arguments={'scene_file': scene_file,
                          'simulation_factor': simulation_factor}.items())

    pedsim_visualizer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pedsim_visualizer'), 'launch', 'visualizer_launch.py')),
        launch_arguments={'frame_id': frame_id}.items()
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_scene_file_cmd)
    ld.add_action(declare_simulation_factor_cmd)
    ld.add_action(declare_frame_id_cmd)

    ld.add_action(pedsim_cmd)
    ld.add_action(pedsim_visualizer_cmd)
    ld.add_action(sim_cmd)

    return ld
