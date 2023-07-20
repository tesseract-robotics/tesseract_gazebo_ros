// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef ROS_GZ_BRIDGE__BRIDGE_HPP_
#define ROS_GZ_BRIDGE__BRIDGE_HPP_

#include <memory>
#include <string>

// include ROS
#include <ros/console.h>
#include <ros/node_handle.h>

// include Gazebo Transport
#include <gz/transport/Node.hh>

#include "factories.hpp"

namespace ros_gz_bridge
{

struct BridgeRosToGzHandles
{
  ros::Subscriber ros_subscriber;
  gz::transport::Node::Publisher gz_publisher;
};

struct BridgeGzToRosHandles
{
  std::shared_ptr<gz::transport::Node> gz_subscriber;
  ros::Publisher ros_publisher;
};

struct BridgeHandles
{
  BridgeRosToGzHandles bridgeRosToGz;
  BridgeGzToRosHandles bridgeGzToRos;
};

BridgeRosToGzHandles
create_bridge_from_ros_to_gz(
  ros::NodeHandle ros_node,
  std::shared_ptr<gz::transport::Node> gz_node,
  const std::string & ros_type_name,
  const std::string & ros_topic_name,
  size_t subscriber_queue_size,
  const std::string & gz_type_name,
  const std::string & gz_topic_name,
  size_t publisher_queue_size)
{
  auto factory = get_factory(ros_type_name, gz_type_name);
  auto gz_pub = factory->create_gz_publisher(
    gz_node, gz_topic_name, publisher_queue_size);

  auto ros_sub = factory->create_ros_subscriber(
    ros_node, ros_topic_name, subscriber_queue_size, gz_pub);

  BridgeRosToGzHandles handles;
  handles.ros_subscriber = ros_sub;
  handles.gz_publisher = gz_pub;
  return handles;
}

BridgeGzToRosHandles
create_bridge_from_gz_to_ros(
  std::shared_ptr<gz::transport::Node> gz_node,
  ros::NodeHandle ros_node,
  const std::string & gz_type_name,
  const std::string & gz_topic_name,
  size_t subscriber_queue_size,
  const std::string & ros_type_name,
  const std::string & ros_topic_name,
  size_t publisher_queue_size)
{
  auto factory = get_factory(ros_type_name, gz_type_name);
  auto ros_pub = factory->create_ros_publisher(
    ros_node, ros_topic_name, publisher_queue_size);

  factory->create_gz_subscriber(
    gz_node, gz_topic_name, subscriber_queue_size, ros_pub);

  BridgeGzToRosHandles handles;
  handles.gz_subscriber = gz_node;
  handles.ros_publisher = ros_pub;
  return handles;
}

BridgeHandles
create_bidirectional_bridge(
  ros::NodeHandle ros_node,
  std::shared_ptr<gz::transport::Node> gz_node,
  const std::string & ros_type_name,
  const std::string & gz_type_name,
  const std::string & topic_name,
  size_t queue_size = 10)
{
  ROS_DEBUG_STREAM("Creating bidirectional bridge for topic" << topic_name
      << " with ROS type [" << ros_type_name << "] and Gazebo Transport"
      << " type [" << gz_type_name << "]");

  BridgeHandles handles;
  handles.bridgeRosToGz = create_bridge_from_ros_to_gz(
   ros_node, gz_node,
   ros_type_name, topic_name, queue_size, gz_type_name, topic_name, queue_size);
  handles.bridgeGzToRos = create_bridge_from_gz_to_ros(
    gz_node, ros_node,
    gz_type_name, topic_name, queue_size, ros_type_name, topic_name, queue_size);
  return handles;
}

}  // namespace ros_gz_bridge

#endif  // ROS_BRIDGE__BRIDGE_HPP_
