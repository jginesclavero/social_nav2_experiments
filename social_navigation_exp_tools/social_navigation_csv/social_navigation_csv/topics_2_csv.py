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
 
import os
import os.path

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from std_msgs.msg import Float32, Int16, Empty
from geometry_msgs.msg import Twist

import csv
from datetime import datetime

class Topics2csv(Node):
    def __init__(self, last_contexts=None):
        super().__init__('topics_2_csv')
        self.path_increment_sub_ = self.create_subscription(
          Float32,
          "/social_nav_exp/path_increment",
          self.path_increment_cb, 1)
        self.task_finished_sub_ = self.create_subscription(
          Empty,
          "/social_nav_exp/task_finished",
          self.task_finished_cb, 1)
        
        self.cost_sub_ = self.create_subscription(
          Int16,
          "/social_nav_exp/robot_cost",
          self.cost_cb, 1)

        self.agent_distance_sub_ = self.create_subscription(
          Float32,
          "/social_nav_exp/distance_to_agent",
          self.agent_distance_cb, 1)

        self.distance_ = 0.0
        self.min_agent_distance_ = 0.0
        self.write_ = ''
        self.cost_samples_ = 0.0
        self.personal_z_ = 0.0
        self.intimate_z_ = 0.0
        self.time_ = 0.0

        #self.get_logger().info("DF_CLIENT: Ready!")
        self.fieldnames_ = ['time', 'distance', 'dmin', 'psi_personal', 'psi_intimate']

        username = os.environ['USER']
        self.path = "/home/" + username + "/waf2020_data/run/"
        filename_list = []
        for file in os.listdir(self.path):
          filename_list.append(int(file[:-4]))
        filename_list_sorted = sorted(filename_list)

        if len(filename_list_sorted) == 0:
          self.csv_filename = str(1) + ".csv"
        else: 
          self.last_file_number = filename_list_sorted[-1]
          self.csv_filename = str(self.last_file_number + 1) + ".csv"

        with open(self.path + self.csv_filename , mode='w+') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames_)
            writer.writeheader()

    def destroy(self):
        super().destroy_node()

    def path_increment_cb(self, msg):
        if self.time_ == 0.0:
            self.time_ = self.get_clock().now().to_msg().sec
        self.distance_ += msg.data
    
    def agent_distance_cb(self, msg):
        if self.min_agent_distance_ == 0.0 or msg.data < self.min_agent_distance_:
            self.min_agent_distance_ = msg.data

    def task_finished_cb(self, msg):
        self.step()

    def cost_cb(self, msg):
        self.cost_samples_ += 1.0
        if msg.data > 0.0 and msg.data < 253.0:
            self.personal_z_ += 1.0
        elif msg.data >= 253.0:
            self.intimate_z_ += 1.0

    def step(self):
        with open(self.path + self.csv_filename, mode='a+') as csv_file:
          personal_percent = self.personal_z_ / self.cost_samples_
          intimate_percent = self.intimate_z_ / self.cost_samples_
          writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames_)
          writer.writerow({
              'time': self.get_clock().now().to_msg().sec - self.time_,
              'distance': self.distance_,
              'dmin': self.min_agent_distance_,
              'psi_personal': personal_percent,
              'psi_intimate': intimate_percent})
          self.write_ = ''
          self.distance_ = 0.0
          self.cost_samples_ = 0.0
          self.personal_z_ = 0.0
          self.intimate_z_ = 0.0
          self.min_agent_distance_ = 0.0
          self.time_ = self.get_clock().now().to_msg().sec


def main(args=None):
    rclpy.init(args=args)
    node = Topics2csv()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()