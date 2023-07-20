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

#include <iostream>
#include <list>
#include <memory>
#include <string>

// include ROS
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include <ros/ros.h>
#include <ros/console.h>
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include Gazebo Transport
#include <gz/transport/Node.hh>

#include "bridge.hpp"

// Direction of bridge.
enum Direction
{
  // Both directions.
  BIDIRECTIONAL = 0,
  // Only from GZ to ROS
  FROM_GZ_TO_ROS = 1,
  // Only from ROS to GZ
  FROM_ROS_TO_GZ = 2,
};

//////////////////////////////////////////////////
void usage()
{
  ROS_INFO_STREAM(
      "Bridge a collection of ROS and Gazebo Transport topics.\n\n"
      << "  parameter_bridge <topic@ROS_type(@,[,])gz_type> .. "
      << " <topic@ROS_type(@,[,])gz_type>\n\n"
      << "The first @ symbol delimits the topic name from the message types.\n"
      << "Following the first @ symbol is the ROS message type.\n"
      << "The ROS message type is followed by an @, [, or ] symbol where\n"
      << "    @  == a bidirectional bridge, \n"
      << "    [  == a bridge from Gazebo to ROS,\n"
      << "    ]  == a bridge from ROS to Gazebo.\n"
      << "Following the direction symbol is the Gazebo Transport message "
      << "type.\n\n"
      << "A bidirectional bridge example:\n"
      << "    parameter_bridge /chatter@std_msgs/String@gz.msgs"
      << ".StringMsg\n\n"
      << "A bridge from Gazebo to ROS example:\n"
      << "    parameter_bridge /chatter@std_msgs/String[gz.msgs"
      << ".StringMsg\n\n"
      << "A bridge from ROS to Gazebo example:\n"
      << "    parameter_bridge /chatter@std_msgs/String]gz.msgs"
      << ".StringMsg" << std::endl);
}

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  if (argc < 2)
  {
    usage();
    return -1;
  }

  // ROS node
  ros::init(argc, argv, "ros_gz_bridge");
  ros::NodeHandle ros_node;

  // Gazebo node
  auto gz_node = std::make_shared<gz::transport::Node>();

  std::list<ros_gz_bridge::BridgeHandles> bidirectional_handles;
  std::list<ros_gz_bridge::BridgeGzToRosHandles> gz_to_ros_handles;
  std::list<ros_gz_bridge::BridgeRosToGzHandles> ros_to_gz_handles;

  // Parse all arguments.
  const std::string delim = "@";
  const size_t queue_size = 10;
  for (auto i = 1; i < argc; ++i)
  {
    std::string arg = std::string(argv[i]);
    auto delimPos = arg.find(delim);
    if (delimPos == std::string::npos || delimPos == 0)
    {
     usage();
     return -1;
    }
    std::string topic_name = arg.substr(0, delimPos);
    arg.erase(0, delimPos + delim.size());

    // Get the direction delimeter, which should be one of:
    //   @ == bidirectional, or
    //   [ == only from GZ to ROS, or
    //   ] == only from ROS to GZ.
    delimPos = arg.find("@");
    Direction direction = BIDIRECTIONAL;
    if (delimPos == std::string::npos || delimPos == 0)
    {
      delimPos = arg.find("[");
      if (delimPos == std::string::npos || delimPos == 0)
      {
        delimPos = arg.find("]");
        if (delimPos == std::string::npos || delimPos == 0)
        {
          usage();
          return -1;
        }
        else
        {
          direction = FROM_ROS_TO_GZ;
        }
      }
      else
      {
        direction = FROM_GZ_TO_ROS;
      }
    }
    std::string ros_type_name = arg.substr(0, delimPos);
    arg.erase(0, delimPos + delim.size());

    delimPos = arg.find(delim);
    if (delimPos != std::string::npos || arg.empty())
    {
      usage();
      return -1;
    }
    std::string gz_type_name = arg;

    try
    {
      switch (direction)
      {
        default:
        case BIDIRECTIONAL:
          bidirectional_handles.push_back(
              ros_gz_bridge::create_bidirectional_bridge(
                ros_node, gz_node,
                ros_type_name, gz_type_name,
                topic_name, queue_size));
          break;
        case FROM_GZ_TO_ROS:
          gz_to_ros_handles.push_back(
              ros_gz_bridge::create_bridge_from_gz_to_ros(
                gz_node, ros_node,
                gz_type_name, topic_name, queue_size,
                ros_type_name, topic_name, queue_size));
          break;
        case FROM_ROS_TO_GZ:
          ros_to_gz_handles.push_back(
              ros_gz_bridge::create_bridge_from_ros_to_gz(
                ros_node, gz_node,
                ros_type_name, topic_name, queue_size,
                gz_type_name, topic_name, queue_size));
          break;
      }
    }
    catch (std::runtime_error &_e)
    {
      ROS_ERROR_STREAM("Failed to create a bridge for topic ["
          << topic_name << "] "
          << "with ROS type [" << ros_type_name << "] and "
          << "Gazebo Transport type [" << gz_type_name << "]"
          << std::endl);
    }
  }

  // ROS asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // Zzzzzz.
  gz::transport::waitForShutdown();

  return 0;
}
