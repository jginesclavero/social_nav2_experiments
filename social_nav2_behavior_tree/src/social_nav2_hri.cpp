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

/* Author: Jonatan Ginés Clavero jonatan.gines@urjc.es */

/* Mantainer: Jonatan Ginés Clavero jonatan.gines@urjc.es */


#include <math.h>
#include <iostream>
#include <memory>
#include <string>
#include <map>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/blackboard.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ros2_knowledge_graph/GraphNode.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  std::string xml_name = argv[1];

  std::string pkgpath = ament_index_cpp::get_package_share_directory("social_nav2_behavior_tree");
  std::string xml_file = pkgpath + "/behavior_trees/" + xml_name;

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;
  factory.registerFromPlugin(loader.getOSName("social_nav2_behavior_tree_navigate_to_wp_bt_node"));
  factory.registerFromPlugin(loader.getOSName("social_nav2_behavior_tree_social_nav2_action_bt_node"));
  factory.registerFromPlugin(loader.getOSName("social_nav2_behavior_tree_dialog_bt_node"));
  factory.registerFromPlugin(loader.getOSName("social_nav2_behavior_tree_stop_condition_bt_node"));
  factory.registerFromPlugin(loader.getOSName("social_nav2_behavior_tree_chrono_decorator_bt_node"));
  factory.registerFromPlugin(loader.getOSName("social_nav2_behavior_tree_turn_agent_decorator_bt_node"));
  factory.registerFromPlugin(loader.getOSName("social_nav2_behavior_tree_simple_goal_reached_condition_bt_node"));
  factory.registerFromPlugin(loader.getOSName("social_nav2_behavior_tree_write_csv_decorator_bt_node"));

  auto blackboard = BT::Blackboard::create();
  auto node = rclcpp::Node::make_shared("social_nav2_behavior_tree_node");
  auto graph = std::make_shared<ros2_knowledge_graph::GraphNode>("social_nav2_behavior_tree_graph");
  graph->start();
  blackboard->set("node", node);
  blackboard->set("social_nav2_behavior_tree_graph", graph);  

  std::unordered_map<std::string, geometry_msgs::msg::Pose> wp_map;
  geometry_msgs::msg::Pose wp;
  wp.position.x = -1.0;
  wp.position.y = -3.0;
  wp.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.0);
  wp_map.insert(std::pair<std::string, geometry_msgs::msg::Pose>("home", wp));

  wp.position.x = 3.06;
  wp.position.y = -2.69;
  wp.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.0);
  wp_map.insert(std::pair<std::string, geometry_msgs::msg::Pose>("return_wp_hri", wp));

  blackboard->set("wp_map", wp_map);  
  blackboard->set<std::string>("df_intent", "");

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);
  rclcpp::Rate rate(1);
  bool finished = false;
  while (rclcpp::ok() && !finished) {
    finished = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "Social nav2 experiment execution finished");

  rclcpp::shutdown();

  return 0;
}
