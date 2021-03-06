# Copyright 2019 Intelligent Robotics Lab
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

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    social_nav2_experiment1_cmd = Node(
        package='social_nav2_behavior_tree',
        node_executable='social_nav2_hri',
        node_name='social_nav2_hri',
        output='screen',
        arguments=["social_nav2_hri_bt.xml"])

    ld = LaunchDescription()
    ld.add_action(social_nav2_experiment1_cmd)
    return ld
