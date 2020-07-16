// Copyright 2020 Intelligent Robotics Lab
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
/* Author: jginesclavero jonatan.gines@urjc.es */

/* Mantainer: jginesclavero jonatan.gines@urjc.es */

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "std_msgs/msg/int16.hpp"
#include <algorithm>
#include <math.h>

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace geometry_msgs::msg;

class RobotCost : public rclcpp::Node
{
public:
  RobotCost(const std::string & name)
  : Node(name)
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = create_publisher<std_msgs::msg::Int16>("social_nav_exp/robot_cost", 1);
  }

  void init()
  {
    costmap_sub_ =
      std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
      shared_from_this(), "global_costmap/social_layer/costmap_raw");
  }

  void step()
  {
    float pos_x, pos_y;
    unsigned int mx, my;
    std_msgs::msg::Int16 msg;
    TransformStamped global2agent;
    try {
      // Check if the transform is available
      auto costmap = costmap_sub_->getCostmap();
      global2agent = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);

      pos_x = global2agent.transform.translation.x;
      pos_y = global2agent.transform.translation.y;

      if (costmap->worldToMap(pos_x, pos_y, mx, my)) {
        msg.data = costmap->getCost(mx, my);
        pub_->publish(msg);
      }
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "%s", e.what());
      return;
    } catch (const std::runtime_error & e) {
      RCLCPP_WARN(get_logger(), "%s", e.what());
      return;
    }

  }

private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto robot_cost_node = 
    std::make_shared<RobotCost>("social_nav_exp_robot_cost_node");
  robot_cost_node->init();
  rclcpp::WallRate loop_rate(1000ms);
  while (rclcpp::ok()) 
  {
    robot_cost_node->step();
    rclcpp::spin_some(robot_cost_node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}