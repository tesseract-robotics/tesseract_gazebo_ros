/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include "../test_utils.h"

/// \brief Flag used to break the publisher loop and terminate the program.
static std::atomic<bool> g_terminatePub(false);

//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to break the infinite loop that publishes messages
/// and exit the program smoothly.
void signal_handler(int _signal)
{
  if (_signal == SIGINT || _signal == SIGTERM)
    g_terminatePub = true;
}

//////////////////////////////////////////////////
int main(int /*argc*/, char **/*argv*/)
{
  // Install a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT,  signal_handler);
  std::signal(SIGTERM, signal_handler);

  // Create a transport node and advertise a topic.
  gz::transport::Node node;

  // gz::msgs::Boolean.
  auto bool_pub = node.Advertise<gz::msgs::Boolean>("bool");
  gz::msgs::Boolean bool_msg;
  ros_gz_bridge::testing::createTestMsg(bool_msg);

  // gz::msgs::Color.
  auto color_pub = node.Advertise<gz::msgs::Color>("color");
  gz::msgs::Color color_msg;
  ros_gz_bridge::testing::createTestMsg(color_msg);

  // gz::msgs::Empty.
  auto empty_pub = node.Advertise<gz::msgs::Empty>("empty");
  gz::msgs::Empty empty_msg;

  // gz::msgs::Int32.
  auto int32_pub = node.Advertise<gz::msgs::Int32>("int32");
  gz::msgs::Int32 int32_msg;
  ros_gz_bridge::testing::createTestMsg(int32_msg);

  // gz::msgs::Float.
  auto float_pub = node.Advertise<gz::msgs::Float>("float");
  gz::msgs::Float float_msg;
  ros_gz_bridge::testing::createTestMsg(float_msg);

  // gz::msgs::Double.
  auto double_pub = node.Advertise<gz::msgs::Double>("double");
  gz::msgs::Double double_msg;
  ros_gz_bridge::testing::createTestMsg(double_msg);

  // gz::msgs::Header.
  auto header_pub = node.Advertise<gz::msgs::Header>("header");
  gz::msgs::Header header_msg;
  ros_gz_bridge::testing::createTestMsg(header_msg);

  // gz::msgs::StringMsg.
  auto string_pub = node.Advertise<gz::msgs::StringMsg>("string");
  gz::msgs::StringMsg string_msg;
  ros_gz_bridge::testing::createTestMsg(string_msg);

  // gz::msgs::Quaternion.
  auto quaternion_pub =
    node.Advertise<gz::msgs::Quaternion>("quaternion");
  gz::msgs::Quaternion quaternion_msg;
  ros_gz_bridge::testing::createTestMsg(quaternion_msg);

  // gz::msgs::Vector3d.
  auto vector3_pub = node.Advertise<gz::msgs::Vector3d>("vector3");
  gz::msgs::Vector3d vector3_msg;
  ros_gz_bridge::testing::createTestMsg(vector3_msg);

  // gz::msgs::Clock.
  auto clock_pub = node.Advertise<gz::msgs::Clock>("clock");
  gz::msgs::Clock clock_msg;
  ros_gz_bridge::testing::createTestMsg(clock_msg);

  // gz::msgs::Point.
  auto point_pub = node.Advertise<gz::msgs::Vector3d>("point");
  gz::msgs::Vector3d point_msg;
  ros_gz_bridge::testing::createTestMsg(point_msg);

  // gz::msgs::Pose.
  auto pose_pub = node.Advertise<gz::msgs::Pose>("pose");
  gz::msgs::Pose pose_msg;
  ros_gz_bridge::testing::createTestMsg(pose_msg);

  // gz::msgs::PoseStamped.
  auto pose_stamped_pub = node.Advertise<gz::msgs::Pose>("pose_stamped");
  gz::msgs::Pose pose_stamped_msg;
  ros_gz_bridge::testing::createTestMsg(pose_stamped_msg);

  // gz::msgs::Pose_V.
  auto pose_v_pub = node.Advertise<gz::msgs::Pose_V>("pose_array");
  gz::msgs::Pose_V pose_v_msg;
  ros_gz_bridge::testing::createTestMsg(pose_v_msg);

  // gz::msgs::Transform.
  auto transform_pub =
      node.Advertise<gz::msgs::Pose>("transform");
  gz::msgs::Pose transform_msg;
  ros_gz_bridge::testing::createTestMsg(transform_msg);

  // gz::msgs::TransformStamped.
  auto transform_stamped_pub =
      node.Advertise<gz::msgs::Pose>("transform_stamped");
  gz::msgs::Pose transform_stamped_msg;
  ros_gz_bridge::testing::createTestMsg(transform_stamped_msg);

  // gz::msgs::Pose_V.
  auto tf2_message_pub =
      node.Advertise<gz::msgs::Pose_V>("tf2_message");
  gz::msgs::Pose_V tf2_msg;
  ros_gz_bridge::testing::createTestMsg(tf2_msg);

  // gz::msgs::Image.
  auto image_pub = node.Advertise<gz::msgs::Image>("image");
  gz::msgs::Image image_msg;
  ros_gz_bridge::testing::createTestMsg(image_msg);

  // gz::msgs::CameraInfo.
  auto camera_info_pub = node.Advertise<gz::msgs::CameraInfo>("camera_info");
  gz::msgs::CameraInfo camera_info_msg;
  ros_gz_bridge::testing::createTestMsg(camera_info_msg);

  // gz::msgs::FluidPressure.
  auto fluid_pressure_pub = node.Advertise<gz::msgs::FluidPressure>("fluid_pressure");
  gz::msgs::FluidPressure fluid_pressure_msg;
  ros_gz_bridge::testing::createTestMsg(fluid_pressure_msg);

  // gz::msgs::IMU.
  auto imu_pub = node.Advertise<gz::msgs::IMU>("imu");
  gz::msgs::IMU imu_msg;
  ros_gz_bridge::testing::createTestMsg(imu_msg);

  // gz::msgs::LaserScan.
  auto laserscan_pub = node.Advertise<gz::msgs::LaserScan>("laserscan");
  gz::msgs::LaserScan laserscan_msg;
  ros_gz_bridge::testing::createTestMsg(laserscan_msg);

  // gz::msgs::Magnetometer.
  auto magnetic_pub = node.Advertise<gz::msgs::Magnetometer>("magnetic");
  gz::msgs::Magnetometer magnetometer_msg;
  ros_gz_bridge::testing::createTestMsg(magnetometer_msg);

  // gz::msgs::NavSat.
  auto navsat_pub = node.Advertise<gz::msgs::NavSat>("navsat");
  gz::msgs::NavSat navsat_msg;
  ros_gz_bridge::testing::createTestMsg(navsat_msg);

  // gz::msgs::Actuators.
  auto actuators_pub = node.Advertise<gz::msgs::Actuators>("actuators");
  gz::msgs::Actuators actuators_msg;
  ros_gz_bridge::testing::createTestMsg(actuators_msg);

  // gz::msgs::OccupancyGrid
  auto map_pub = node.Advertise<gz::msgs::OccupancyGrid>("map");
  gz::msgs::OccupancyGrid map_msg;
  ros_gz_bridge::testing::createTestMsg(map_msg);

  // gz::msgs::Odometry.
  auto odometry_pub = node.Advertise<gz::msgs::Odometry>("odometry");
  gz::msgs::Odometry odometry_msg;
  ros_gz_bridge::testing::createTestMsg(odometry_msg);

  // gz::msgs::Model.
  auto joint_states_pub = node.Advertise<gz::msgs::Model>("joint_states");
  gz::msgs::Model joint_states_msg;
  ros_gz_bridge::testing::createTestMsg(joint_states_msg);

  // gz::msgs::Twist.
  auto twist_pub = node.Advertise<gz::msgs::Twist>("twist");
  gz::msgs::Twist twist_msg;
  ros_gz_bridge::testing::createTestMsg(twist_msg);

  // gz::msgs::PointCloudPacked.
  auto pointcloudpacked_pub = node.Advertise<gz::msgs::PointCloudPacked>(
      "pointcloud2");
  gz::msgs::PointCloudPacked pointcloudpacked_msg;
  ros_gz_bridge::testing::createTestMsg(pointcloudpacked_msg);

  // gz::msgs::BatteryState.
  auto battery_state_pub = node.Advertise<gz::msgs::BatteryState>("battery_state");
  gz::msgs::BatteryState battery_state_msg;
  ros_gz_bridge::testing::createTestMsg(battery_state_msg);

  auto marker_pub = node.Advertise<gz::msgs::Marker>("marker");
  gz::msgs::Marker marker_msg;
  ros_gz_bridge::testing::createTestMsg(marker_msg);

  auto marker_array_pub = node.Advertise<gz::msgs::Marker_V>("marker_array");
  gz::msgs::Marker_V marker_array_msg;
  ros_gz_bridge::testing::createTestMsg(marker_array_msg);

  // Publish messages at 1Hz.
  while (!g_terminatePub)
  {
    bool_pub.Publish(bool_msg);
    color_pub.Publish(color_msg);
    empty_pub.Publish(empty_msg);
    int32_pub.Publish(int32_msg);
    float_pub.Publish(float_msg);
    double_pub.Publish(double_msg);
    header_pub.Publish(header_msg);
    string_pub.Publish(string_msg);
    quaternion_pub.Publish(quaternion_msg);
    vector3_pub.Publish(vector3_msg);
    clock_pub.Publish(clock_msg);
    point_pub.Publish(point_msg);
    pose_pub.Publish(pose_msg);
    pose_v_pub.Publish(pose_v_msg);
    pose_stamped_pub.Publish(pose_stamped_msg);
    transform_pub.Publish(transform_msg);
    transform_stamped_pub.Publish(transform_stamped_msg);
    tf2_message_pub.Publish(tf2_msg);
    image_pub.Publish(image_msg);
    camera_info_pub.Publish(camera_info_msg);
    fluid_pressure_pub.Publish(fluid_pressure_msg);
    imu_pub.Publish(imu_msg);
    laserscan_pub.Publish(laserscan_msg);
    magnetic_pub.Publish(magnetometer_msg);
    navsat_pub.Publish(navsat_msg);
    actuators_pub.Publish(actuators_msg);
    map_pub.Publish(map_msg);
    odometry_pub.Publish(odometry_msg);
    joint_states_pub.Publish(joint_states_msg);
    twist_pub.Publish(twist_msg);
    pointcloudpacked_pub.Publish(pointcloudpacked_msg);
    battery_state_pub.Publish(battery_state_msg);
    marker_pub.Publish(marker_msg);
    marker_array_pub.Publish(marker_array_msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
