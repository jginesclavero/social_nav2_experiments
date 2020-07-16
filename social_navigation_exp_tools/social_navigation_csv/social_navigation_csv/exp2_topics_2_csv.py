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
        self.agent_distance_sub_ = self.create_subscription(
          Float32,
          "/social_nav_exp/distance_to_agent",
          self.agent_distance_cb, 1)
        self.cost_sub_ = self.create_subscription(
          Int16,
          "/social_nav_exp/robot_cost",
          self.cost_cb, 1)

        self.distance_ = 0.0
        self.cost_ = 0.0
        #self.get_logger().info("DF_CLIENT: Ready!")
        self.fieldnames_ = ['distance', 'robot_cost']

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
        timer_period = 1.0 # seconds
        timer = self.create_timer(timer_period, self.step)

    def destroy(self):
        super().destroy_node()
    
    def agent_distance_cb(self, msg):
        self.distance_ = msg.data

    def cost_cb(self, msg):
        self.cost_ = msg.data

    def step(self):
      if not(self.distance_ == 0.0 and self.cost_ == 0.0):
          with open(self.path + self.csv_filename, mode='a+') as csv_file:
              writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames_)
              writer.writerow({
                  'distance': self.distance_,
                  'robot_cost': self.cost_})
              self.distance_ = 0.0
              self.cost_ = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = Topics2csv()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()