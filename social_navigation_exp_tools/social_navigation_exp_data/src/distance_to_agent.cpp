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
#include <algorithm>
#include <math.h>

#include <std_msgs/msg/float32.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace geometry_msgs::msg;

class DistanceNode : public rclcpp::Node
{
public:
  DistanceNode(const std::string & name, const std::string & agent) : Node(name)
  {
    pub_ = create_publisher<std_msgs::msg::Float32>("social_nav_exp/distance_to_agent", 1);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    agent_id_ = agent;
  }

  void step()
  {
    TransformStamped map2agent, bf2map;
    tf2::Transform map2agent_tf2, bf2map_tf2;
    try {
      // Check if the transform is available
      bf2map = tf_buffer_->lookupTransform("base_footprint", "map", tf2::TimePointZero);
      map2agent = tf_buffer_->lookupTransform("map", agent_id_, tf2::TimePointZero);
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "%s", e.what());
      return;
    }
    tf2::impl::Converter<true, false>::convert(map2agent.transform, map2agent_tf2);
    tf2::impl::Converter<true, false>::convert(bf2map.transform, bf2map_tf2);

    auto bf2agent = bf2map_tf2 * map2agent_tf2;
    
    std_msgs::msg::Float32 distance;
    distance.data = bf2agent.getOrigin().length();
    // RCLCPP_INFO(get_logger(), "%f", distance.data);
    pub_->publish(distance);
  }

private:

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string agent_id_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::string agent;
  agent = argv[1];
  auto distance_node = 
    std::make_shared<DistanceNode>("social_nav_exp_distance_node", agent);
  rclcpp::WallRate loop_rate(1000ms);

  while (rclcpp::ok()) 
  {
    distance_node->step();
    rclcpp::spin_some(distance_node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}