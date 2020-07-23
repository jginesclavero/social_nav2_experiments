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
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float32.hpp"
#include <algorithm>
#include <math.h>

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

class PathPub : public rclcpp::Node
{
public:
  PathPub(const std::string & name)
  : Node(name)
  {
    amcl_sub_ = create_subscription<PoseWithCovarianceStamped>(
      "amcl_pose",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&PathPub::amclCb, this, std::placeholders::_1));
    robot_path_pub_ = create_publisher<nav_msgs::msg::Path>("measuring_tools/robot_path", 1);
    agent_path_pub_ = create_publisher<nav_msgs::msg::Path>("measuring_tools/agent_path", 1);
    path_increment_pub_ = create_publisher<std_msgs::msg::Float32>("measuring_tools/path_increment", 1);
    frame_id_ = "map";

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void amclCb(const PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (last_pose_.pose.position.x == 0.0 &&
        last_pose_.pose.position.y == 0.0)
    {
      last_pose_.pose = msg->pose.pose;
    }

    nav_msgs::msg::Path path;
    PoseStamped p;
    p.pose = msg->pose.pose;
    r_poses_.push_back(p);
    
    path.header.frame_id = frame_id_;
    path.poses = r_poses_;
    robot_path_pub_->publish(path);

    std_msgs::msg::Float32 increment;
    increment.data = sqrt(
      pow((p.pose.position.x - last_pose_.pose.position.x), 2) +
      pow((p.pose.position.y - last_pose_.pose.position.y), 2)
    );
    path_increment_pub_->publish(increment);
    last_pose_.pose = p.pose;

    tf2::Transform global2agent_tf2;
    if (!getAgentTF("agent_1", global2agent_tf2)) {return;}
    publishAgentPath(global2agent_tf2);
  }

private:
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr robot_path_pub_, agent_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr path_increment_pub_;
  std::vector<PoseStamped> r_poses_, a_poses_;
  PoseStamped last_pose_;
  std::string frame_id_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  bool getAgentTF(std::string id, tf2::Transform & tf)
  {
    geometry_msgs::msg::TransformStamped global2agent;
    try {
      // Check if the transform is available
      global2agent = tf_buffer_->lookupTransform("map", id, tf2::TimePointZero);
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "%s", e.what());
      return false;
    }
    tf2::impl::Converter<true, false>::convert(global2agent.transform, tf);
    return true;
  }

  void publishAgentPath(tf2::Transform & tf)
  {
    nav_msgs::msg::Path path;
    PoseStamped p;
    p.pose.position.x = tf.getOrigin().getX();
    p.pose.position.y = tf.getOrigin().getY();
    a_poses_.push_back(p);
    
    path.header.frame_id = frame_id_;
    path.poses = a_poses_;
    agent_path_pub_->publish(path);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto robot_path_node = 
    std::make_shared<PathPub>("social_nav_exp_robot_path_node");
  rclcpp::spin(robot_path_node);
  rclcpp::shutdown();
  return 0;
}