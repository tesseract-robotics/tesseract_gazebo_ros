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

#ifndef ROS_GZ_BRIDGE__FACTORY_HPP_
#define ROS_GZ_BRIDGE__FACTORY_HPP_

#include <functional>
#include <memory>
#include <string>

#include <gz/transport/Node.hh>

// include ROS message event
#include <ros/console.h>
#include <ros/message.h>
#include <ros/ros.h>

#include "factory_interface.hpp"

namespace ros_gz_bridge
{

template<typename ROS_T, typename GZ_T>
class Factory : public FactoryInterface
{
public:
  Factory(
    const std::string & ros_type_name, const std::string & gz_type_name)
  : ros_type_name_(ros_type_name),
    gz_type_name_(gz_type_name)
  {}

  ros::Publisher
  create_ros_publisher(
    ros::NodeHandle node,
    const std::string & topic_name,
    size_t queue_size)
  {
    return node.advertise<ROS_T>(topic_name, queue_size);
  }

  gz::transport::Node::Publisher
  create_gz_publisher(
    std::shared_ptr<gz::transport::Node> gz_node,
    const std::string & topic_name,
    size_t /*queue_size*/)
  {
    return gz_node->Advertise<GZ_T>(topic_name);
  }

  ros::Subscriber
  create_ros_subscriber(
    ros::NodeHandle node,
    const std::string & topic_name,
    size_t queue_size,
    gz::transport::Node::Publisher & gz_pub)
  {
    // workaround for https://github.com/ros/roscpp_core/issues/22 to get the
    // connection header
    ros::SubscribeOptions ops;
    ops.topic = topic_name;
    ops.queue_size = queue_size;
    ops.md5sum = ros::message_traits::md5sum<ROS_T>();
    ops.datatype = ros::message_traits::datatype<ROS_T>();
    ops.helper = ros::SubscriptionCallbackHelperPtr(
      new ros::SubscriptionCallbackHelperT
        <const ros::MessageEvent<ROS_T const> &>(
          boost::bind(
            &Factory<ROS_T, GZ_T>::ros_callback,
            _1, gz_pub, ros_type_name_, gz_type_name_)));
    return node.subscribe(ops);
  }

  void
  create_gz_subscriber(
    std::shared_ptr<gz::transport::Node> node,
    const std::string & topic_name,
    size_t /*queue_size*/,
    ros::Publisher ros_pub)
  {

    std::function<void(const GZ_T&,
                       const gz::transport::MessageInfo &)> subCb =
    [this, ros_pub](const GZ_T &_msg,
                     const gz::transport::MessageInfo &_info)
    {
      // Ignore messages that are published from this bridge.
      if (!_info.IntraProcess())
        this->gz_callback(_msg, ros_pub);
    };

    node->Subscribe(topic_name, subCb);
  }

protected:

  static
  void ros_callback(
    const ros::MessageEvent<ROS_T const> & ros_msg_event,
    gz::transport::Node::Publisher & gz_pub,
    const std::string &ros_type_name,
    const std::string &gz_type_name)
  {
    const boost::shared_ptr<ros::M_string> & connection_header =
      ros_msg_event.getConnectionHeaderPtr();
    if (!connection_header) {
      ROS_ERROR("Dropping message %s without connection header",
          ros_type_name.c_str());
      return;
    }

    std::string key = "callerid";
    if (connection_header->find(key) != connection_header->end()) {
      if (connection_header->at(key) == ros::this_node::getName()) {
        return;
      }
    }

    const boost::shared_ptr<ROS_T const> & ros_msg =
      ros_msg_event.getConstMessage();

    GZ_T gz_msg;
    convert_ros_to_gz(*ros_msg, gz_msg);
    gz_pub.Publish(gz_msg);
    ROS_INFO_ONCE("Passing message from ROS %s to Gazebo %s (showing msg"\
        " only once per type)", ros_type_name.c_str(), gz_type_name.c_str());
  }

  static
  void gz_callback(
    const GZ_T & gz_msg,
    ros::Publisher ros_pub)
  {
    ROS_T ros_msg;
    convert_gz_to_ros(gz_msg, ros_msg);
    ros_pub.publish(ros_msg);
  }

public:
  // since convert functions call each other for sub messages they must be
  // public defined outside of the class
  static
  void
  convert_ros_to_gz(
    const ROS_T & ros_msg,
    GZ_T & gz_msg);
  static
  void
  convert_gz_to_ros(
    const GZ_T & gz_msg,
    ROS_T & ros_msg);

  std::string ros_type_name_;
  std::string gz_type_name_;
};

}  // namespace ros_gz_bridge

#endif  // ROS_GZ_BRIDGE__FACTORY_HPP_
