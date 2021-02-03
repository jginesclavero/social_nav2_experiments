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
#include "measuring_tools/dummy_tf2_node.hpp"
#include <algorithm>
#include <math.h>


using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace geometry_msgs::msg;

namespace dummy_tf2
{
DummyTF2::DummyTF2(const std::string & name) : Node(name)
{
  sub_ = create_subscription<std_msgs::msg::Empty>(
      "social_navigation/update_approach_tf", rclcpp::SystemDefaultsQoS(),
      std::bind(&DummyTF2::approachTFCallback, this, std::placeholders::_1));
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  update_agent_pub_ = create_publisher<pedsim_msgs::msg::AgentStates>(
    "pedsim_simulator/simulated_agents", rclcpp::SensorDataQoS());
  rotation = geometry_msgs::msg::Twist();
}

void DummyTF2::approachTFCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  update_tf = true;
}

void DummyTF2::updateGazeboAgent(geometry_msgs::msg::Twist t)
{
  auto msg = pedsim_msgs::msg::AgentStates();
  std::vector<pedsim_msgs::msg::AgentState> agents;
  pedsim_msgs::msg::AgentState agent;
  auto model_pose = Pose();
  model_pose.position.x = 2.5;
  model_pose.position.y = -3.0;
  model_pose.position.z = 0.0;
  agent.pose = model_pose;
  agent.twist = t;
  agent.id = 3;
  agents.push_back(agent);
  msg.agent_states = agents;
  update_agent_pub_->publish(msg);
}

void DummyTF2::step()
{
  tf2::Quaternion qt;
  if (update_tf)
  {
    //srand (time(NULL));
    rand_angle = 2 * M_PI * rand() / (RAND_MAX);
    rotation.linear.x = cos(rand_angle);
    rotation.linear.y = sin(rand_angle);
    update_tf = false;
  }
  
  updateGazeboAgent(rotation);
  TransformStamped tf_msg;
  tf_msg.transform.translation.x = 2.5;
  tf_msg.transform.translation.y = -3.0;
  tf_msg.transform.translation.z = 0.0;
  qt.setRPY(0.0, 0.0, rand_angle);
  qt.normalize();
  tf_msg.transform.rotation = tf2::toMsg(qt);
  tf_msg.header.frame_id = "map";
  tf_msg.header.stamp = now();
  tf_msg.child_frame_id = "agent_3";
  tf_broadcaster_->sendTransform(tf_msg);
}

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto dummy_tf2 = std::make_shared<dummy_tf2::DummyTF2>("dummy_tf2_node");
  rclcpp::Rate loop_rate(200ms); 
  while (rclcpp::ok())
  {
    dummy_tf2->step();
    rclcpp::spin_some(dummy_tf2);
    loop_rate.sleep();
  }
  rclcpp::shutdown();

  return 0;
}