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
#include "std_msgs/msg/empty.hpp"

namespace social_nav2_behavior_tree
{

class StopCondition : public BT::ConditionNode
{
public:
  StopCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  StopCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("stop_topic", "Topic to stop a BTAction"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_;
  std::string topic_;
  bool initialized_;
  bool stop_trigger_;

  void callback(const std_msgs::msg::Empty::SharedPtr msg);
  void initialize();

};

}  // namespace social_nav2_behavior_tree

#endif  // social_nav2_behavior_tree__BEHAVIOR_TREE_NODES__STOP_CONDITION_HPP_
