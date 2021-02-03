// Copyright (c) 2021 Intelligent Robotics Lab
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

#ifndef social_nav2_behavior_tree__BEHAVIOR_TREE_NODES__STOP_CONDITION_HPP_
#define social_nav2_behavior_tree__BEHAVIOR_TREE_NODES__STOP_CONDITION_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "std_msgs/msg/float32.hpp"

namespace social_nav2_behavior_tree
{

class Chrono : public BT::ConditionNode
{
public:
  Chrono(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  bool initialized_;
  rclcpp::Time first_tick_;

  void initialize();

};

}  // namespace social_nav2_behavior_tree

#endif  // social_nav2_behavior_tree__BEHAVIOR_TREE_NODES__STOP_CONDITION_HPP_
