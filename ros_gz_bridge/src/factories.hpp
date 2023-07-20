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

#ifndef ROS_GZ_BRIDGE__BUILTIN_INTERFACES_FACTORIES_HPP_
#define ROS_GZ_BRIDGE__BUILTIN_INTERFACES_FACTORIES_HPP_

// ROS messages
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
// #include <mav_msgs/Actuators.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Gazebo messages
#include <gz/msgs.hh>

#include "factory.hpp"

#include <memory>
#include <string>

namespace ros_gz_bridge
{
std::shared_ptr<FactoryInterface>
get_factory(const std::string & ros_type_name,
            const std::string & gz_type_name);

// conversion functions for available interfaces

// std_msgs
template<>
void
Factory<
  std_msgs::Bool,
  gz::msgs::Boolean
>::convert_ros_to_gz(
  const std_msgs::Bool & ros_msg,
  gz::msgs::Boolean & gz_msg);

template<>
void
Factory<
  std_msgs::Bool,
  gz::msgs::Boolean
>::convert_gz_to_ros(
  const gz::msgs::Boolean & gz_msg,
  std_msgs::Bool & ros_msg);

template<>
void
Factory<
  std_msgs::ColorRGBA,
  gz::msgs::Color
>::convert_ros_to_gz(
  const std_msgs::ColorRGBA & ros_msg,
  gz::msgs::Color & gz_msg);

template<>
void
Factory<
  std_msgs::ColorRGBA,
  gz::msgs::Color
>::convert_gz_to_ros(
  const gz::msgs::Color & gz_msg,
  std_msgs::ColorRGBA & ros_msg);

template<>
void
Factory<
  std_msgs::Empty,
  gz::msgs::Empty
>::convert_ros_to_gz(
  const std_msgs::Empty & ros_msg,
  gz::msgs::Empty & gz_msg);

template<>
void
Factory<
  std_msgs::Empty,
  gz::msgs::Empty
>::convert_gz_to_ros(
  const gz::msgs::Empty & gz_msg,
  std_msgs::Empty & ros_msg);

template<>
void
Factory<
  std_msgs::Int32,
  gz::msgs::Int32
>::convert_ros_to_gz(
  const std_msgs::Int32 & ros_msg,
  gz::msgs::Int32 & gz_msg);

template<>
void
Factory<
  std_msgs::Int32,
  gz::msgs::Int32
>::convert_gz_to_ros(
  const gz::msgs::Int32 & gz_msg,
  std_msgs::Int32 & ros_msg);

template<>
void
Factory<
  std_msgs::Float32,
  gz::msgs::Float
>::convert_ros_to_gz(
  const std_msgs::Float32 & ros_msg,
  gz::msgs::Float & gz_msg);

template<>
void
Factory<
  std_msgs::Float32,
  gz::msgs::Float
>::convert_gz_to_ros(
  const gz::msgs::Float & gz_msg,
  std_msgs::Float32 & ros_msg);

template<>
void
Factory<
  std_msgs::Float64,
  gz::msgs::Double
>::convert_ros_to_gz(
  const std_msgs::Float64 & ros_msg,
  gz::msgs::Double & gz_msg);

template<>
void
Factory<
  std_msgs::Float64,
  gz::msgs::Double
>::convert_gz_to_ros(
  const gz::msgs::Double & gz_msg,
  std_msgs::Float64 & ros_msg);

template<>
void
Factory<
  std_msgs::Header,
  gz::msgs::Header
>::convert_ros_to_gz(
  const std_msgs::Header & ros_msg,
  gz::msgs::Header & gz_msg);

template<>
void
Factory<
  std_msgs::Header,
  gz::msgs::Header
>::convert_gz_to_ros(
  const gz::msgs::Header & gz_msg,
  std_msgs::Header & ros_msg);

template<>
void
Factory<
  std_msgs::String,
  gz::msgs::StringMsg
>::convert_ros_to_gz(
  const std_msgs::String & ros_msg,
  gz::msgs::StringMsg & gz_msg);

template<>
void
Factory<
  std_msgs::String,
  gz::msgs::StringMsg
>::convert_gz_to_ros(
  const gz::msgs::StringMsg & gz_msg,
  std_msgs::String & ros_msg);

// rosgraph_msgs
template<>
void
Factory<
  rosgraph_msgs::Clock,
  gz::msgs::Clock
>::convert_ros_to_gz(
  const rosgraph_msgs::Clock & ros_msg,
  gz::msgs::Clock & gz_msg);

template<>
void
Factory<
  rosgraph_msgs::Clock,
  gz::msgs::Clock
>::convert_gz_to_ros(
  const gz::msgs::Clock & gz_msg,
  rosgraph_msgs::Clock & ros_msg);

// geometry_msgs
template<>
void
Factory<
  geometry_msgs::Quaternion,
  gz::msgs::Quaternion
>::convert_ros_to_gz(
  const geometry_msgs::Quaternion & ros_msg,
  gz::msgs::Quaternion & gz_msg);

template<>
void
Factory<
  geometry_msgs::Quaternion,
  gz::msgs::Quaternion
>::convert_gz_to_ros(
  const gz::msgs::Quaternion & gz_msg,
  geometry_msgs::Quaternion & ros_msg);

template<>
void
Factory<
  geometry_msgs::Vector3,
  gz::msgs::Vector3d
>::convert_ros_to_gz(
  const geometry_msgs::Vector3 & ros_msg,
  gz::msgs::Vector3d & gz_msg);

template<>
void
Factory<
  geometry_msgs::Vector3,
  gz::msgs::Vector3d
>::convert_gz_to_ros(
  const gz::msgs::Vector3d & gz_msg,
  geometry_msgs::Vector3 & ros_msg);

template<>
void
Factory<
  geometry_msgs::Point,
  gz::msgs::Vector3d
>::convert_ros_to_gz(
  const geometry_msgs::Point & ros_msg,
  gz::msgs::Vector3d & gz_msg);

template<>
void
Factory<
  geometry_msgs::Point,
  gz::msgs::Vector3d
>::convert_gz_to_ros(
  const gz::msgs::Vector3d & gz_msg,
  geometry_msgs::Point & ros_msg);

template<>
void
Factory<
  geometry_msgs::Pose,
  gz::msgs::Pose
>::convert_ros_to_gz(
  const geometry_msgs::Pose & ros_msg,
  gz::msgs::Pose & gz_msg);

template<>
void
Factory<
  geometry_msgs::Pose,
  gz::msgs::Pose
>::convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::Pose & ros_msg);

template<>
void
Factory<
  geometry_msgs::PoseArray,
  gz::msgs::Pose_V
>::convert_ros_to_gz(
  const geometry_msgs::PoseArray & ros_msg,
  gz::msgs::Pose_V & gz_msg);

template<>
void
Factory<
  geometry_msgs::PoseArray,
  gz::msgs::Pose_V
>::convert_gz_to_ros(
  const gz::msgs::Pose_V & gz_msg,
  geometry_msgs::PoseArray & ros_msg);

template<>
void
Factory<
  geometry_msgs::PoseStamped,
  gz::msgs::Pose
>::convert_ros_to_gz(
  const geometry_msgs::PoseStamped & ros_msg,
  gz::msgs::Pose & gz_msg);

template<>
void
Factory<
  geometry_msgs::PoseStamped,
  gz::msgs::Pose
>::convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::PoseStamped & ros_msg);

template<>
void
Factory<
  geometry_msgs::Transform,
  gz::msgs::Pose
>::convert_ros_to_gz(
  const geometry_msgs::Transform & ros_msg,
  gz::msgs::Pose & gz_msg);

template<>
void
Factory<
  geometry_msgs::Transform,
  gz::msgs::Pose
>::convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::Transform & ros_msg);

template<>
void
Factory<
  geometry_msgs::TransformStamped,
  gz::msgs::Pose
>::convert_ros_to_gz(
  const geometry_msgs::TransformStamped & ros_msg,
  gz::msgs::Pose & gz_msg);

template<>
void
Factory<
  geometry_msgs::TransformStamped,
  gz::msgs::Pose
>::convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::TransformStamped & ros_msg);

template<>
void
Factory<
  tf2_msgs::TFMessage,
  gz::msgs::Pose_V
>::convert_ros_to_gz(
  const tf2_msgs::TFMessage & ros_msg,
  gz::msgs::Pose_V & gz_msg);

template<>
void
Factory<
  tf2_msgs::TFMessage,
  gz::msgs::Pose_V
>::convert_gz_to_ros(
  const gz::msgs::Pose_V & gz_msg,
  tf2_msgs::TFMessage & ros_msg);

template<>
void
Factory<
  geometry_msgs::Twist,
  gz::msgs::Twist
>::convert_ros_to_gz(
  const geometry_msgs::Twist & ros_msg,
  gz::msgs::Twist & gz_msg);

template<>
void
Factory<
  geometry_msgs::Twist,
  gz::msgs::Twist
>::convert_gz_to_ros(
  const gz::msgs::Twist & gz_msg,
  geometry_msgs::Twist & ros_msg);

// mav_msgs
// template<>
// void
// Factory<
//   mav_msgs::Actuators,
//   gz::msgs::Actuators
// >::convert_ros_to_gz(
//   const mav_msgs::Actuators & ros_msg,
//   gz::msgs::Actuators & gz_msg);
//
// template<>
// void
// Factory<
//   mav_msgs::Actuators,
//   gz::msgs::Actuators
// >::convert_gz_to_ros(
//   const gz::msgs::Actuators & gz_msg,
//   mav_msgs::Actuators & ros_msg);

// nav_msgs
template<>
void
Factory<
  nav_msgs::OccupancyGrid,
  gz::msgs::OccupancyGrid
>::convert_ros_to_gz(
  const nav_msgs::OccupancyGrid & ros_msg,
  gz::msgs::OccupancyGrid & gz_msg);

template<>
void
Factory<
  nav_msgs::OccupancyGrid,
  gz::msgs::OccupancyGrid
>::convert_gz_to_ros(
  const gz::msgs::OccupancyGrid & gz_msg,
  nav_msgs::OccupancyGrid & ros_msg);

template<>
void
Factory<
  nav_msgs::Odometry,
  gz::msgs::Odometry
>::convert_ros_to_gz(
  const nav_msgs::Odometry & ros_msg,
  gz::msgs::Odometry & gz_msg);

template<>
void
Factory<
  nav_msgs::Odometry,
  gz::msgs::Odometry
>::convert_gz_to_ros(
  const gz::msgs::Odometry & gz_msg,
  nav_msgs::Odometry & ros_msg);

// sensor_msgs
template<>
void
Factory<
  sensor_msgs::FluidPressure,
  gz::msgs::FluidPressure
>::convert_ros_to_gz(
  const sensor_msgs::FluidPressure & ros_msg,
  gz::msgs::FluidPressure & gz_msg);

template<>
void
Factory<
  sensor_msgs::FluidPressure,
  gz::msgs::FluidPressure
>::convert_gz_to_ros(
  const gz::msgs::FluidPressure & gz_msg,
  sensor_msgs::FluidPressure & ros_msg);

template<>
void
Factory<
  sensor_msgs::Image,
  gz::msgs::Image
>::convert_ros_to_gz(
  const sensor_msgs::Image & ros_msg,
  gz::msgs::Image & gz_msg);

template<>
void
Factory<
  sensor_msgs::Image,
  gz::msgs::Image
>::convert_gz_to_ros(
  const gz::msgs::Image & gz_msg,
  sensor_msgs::Image & ros_msg);

template<>
void
Factory<
  sensor_msgs::CameraInfo,
  gz::msgs::CameraInfo
>::convert_ros_to_gz(
  const sensor_msgs::CameraInfo & ros_msg,
  gz::msgs::CameraInfo & gz_msg);

template<>
void
Factory<
  sensor_msgs::CameraInfo,
  gz::msgs::CameraInfo
>::convert_gz_to_ros(
  const gz::msgs::CameraInfo & gz_msg,
  sensor_msgs::CameraInfo & ros_msg);

template<>
void
Factory<
  sensor_msgs::Imu,
  gz::msgs::IMU
>::convert_ros_to_gz(
  const sensor_msgs::Imu & ros_msg,
  gz::msgs::IMU & gz_msg);

template<>
void
Factory<
  sensor_msgs::Imu,
  gz::msgs::IMU
>::convert_gz_to_ros(
  const gz::msgs::IMU & gz_msg,
  sensor_msgs::Imu & ros_msg);

template<>
void
Factory<
  sensor_msgs::JointState,
  gz::msgs::Model
>::convert_ros_to_gz(
  const sensor_msgs::JointState & ros_msg,
  gz::msgs::Model & gz_msg);

template<>
void
Factory<
  sensor_msgs::JointState,
  gz::msgs::Model
>::convert_gz_to_ros(
  const gz::msgs::Model & gz_msg,
  sensor_msgs::JointState & ros_msg);

template<>
void
Factory<
  sensor_msgs::LaserScan,
  gz::msgs::LaserScan
>::convert_ros_to_gz(
  const sensor_msgs::LaserScan & ros_msg,
  gz::msgs::LaserScan & gz_msg);

template<>
void
Factory<
  sensor_msgs::LaserScan,
  gz::msgs::LaserScan
>::convert_gz_to_ros(
  const gz::msgs::LaserScan & gz_msg,
  sensor_msgs::LaserScan & ros_msg);

template<>
void
Factory<
  sensor_msgs::MagneticField,
  gz::msgs::Magnetometer
>::convert_ros_to_gz(
  const sensor_msgs::MagneticField & ros_msg,
  gz::msgs::Magnetometer & gz_msg);

template<>
void
Factory<
  sensor_msgs::MagneticField,
  gz::msgs::Magnetometer
>::convert_gz_to_ros(
  const gz::msgs::Magnetometer & gz_msg,
  sensor_msgs::MagneticField & ros_msg);

template<>
void
Factory<
  sensor_msgs::NavSatFix,
  gz::msgs::NavSat
>::convert_ros_to_gz(
  const sensor_msgs::NavSatFix & ros_msg,
  gz::msgs::NavSat & gz_msg);

template<>
void
Factory<
  sensor_msgs::NavSatFix,
  gz::msgs::NavSat
>::convert_gz_to_ros(
  const gz::msgs::NavSat & gz_msg,
  sensor_msgs::NavSatFix & ros_msg);

template<>
void
Factory<
  sensor_msgs::PointCloud2,
  gz::msgs::PointCloudPacked
>::convert_ros_to_gz(
  const sensor_msgs::PointCloud2 & ros_msg,
  gz::msgs::PointCloudPacked & gz_msg);

template<>
void
Factory<
  sensor_msgs::PointCloud2,
  gz::msgs::PointCloudPacked
>::convert_gz_to_ros(
  const gz::msgs::PointCloudPacked & gz_msg,
  sensor_msgs::PointCloud2 & ros_msg);

template<>
void
Factory<
  sensor_msgs::BatteryState,
  gz::msgs::BatteryState
>::convert_ros_to_gz(
  const sensor_msgs::BatteryState & ros_msg,
  gz::msgs::BatteryState & gz_msg);

template<>
void
Factory<
  sensor_msgs::BatteryState,
  gz::msgs::BatteryState
>::convert_gz_to_ros(
  const gz::msgs::BatteryState & gz_msg,
  sensor_msgs::BatteryState & ros_msg);

template<>
void
Factory<
  visualization_msgs::Marker,
  gz::msgs::Marker
>::convert_ros_to_gz(
    const visualization_msgs::Marker & ros_msg,
    gz::msgs::Marker & gz_msg);

template<>
void
Factory<
  visualization_msgs::Marker,
  gz::msgs::Marker
>::convert_gz_to_ros(
    const gz::msgs::Marker & gz_msg,
    visualization_msgs::Marker & ros_msg);

template<>
void
Factory<
  visualization_msgs::MarkerArray,
  gz::msgs::Marker_V
>::convert_ros_to_gz(
    const visualization_msgs::MarkerArray & ros_msg,
    gz::msgs::Marker_V & gz_msg);

template<>
void
Factory<
  visualization_msgs::MarkerArray,
  gz::msgs::Marker_V
>::convert_gz_to_ros(
    const gz::msgs::Marker_V & gz_msg,
    visualization_msgs::MarkerArray & ros_msg);

}  // namespace ros_gz_bridge

#endif  // ROS_gz_BRIDGE__BUILTIN_INTERFACES_FACTORIES_HPP_
