
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
#ifndef DUMMYTF2NODE__H
#define DUMMYTF2NODE__H

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/empty.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "pedsim_msgs/msg/agent_states.hpp"
#include "pedsim_msgs/msg/agent_state.hpp"

using namespace geometry_msgs::msg;

namespace dummy_tf2
{
class DummyTF2 : public rclcpp::Node
{
public:

  DummyTF2(const std::string & name);
  void step();

private:

  void approachTFCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void updateGazeboAgent(geometry_msgs::msg::Twist t);

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<pedsim_msgs::msg::AgentStates>::SharedPtr update_agent_pub_;
  bool update_tf;
  float rand_angle;
};
};  // namespace pedsim

#endif
