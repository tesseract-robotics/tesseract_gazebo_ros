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

#include <algorithm>
#include <exception>
#include <ros/console.h>

#include "ros_gz_bridge/convert.hpp"

namespace ros_gz_bridge
{

// This can be used to replace `::` with `/` to make frame_id compatible with TF
std::string replace_delimiter(const std::string &input,
                              const std::string &old_delim,
                              const std::string new_delim)
{
  std::string output;
  output.reserve(input.size());

  std::size_t last_pos = 0;

  while (last_pos < input.size())
  {
    std::size_t pos = input.find(old_delim, last_pos);
    output += input.substr(last_pos, pos - last_pos);
    if (pos != std::string::npos)
    {
      output += new_delim;
      pos += old_delim.size();
    }

    last_pos = pos;
  }

  return output;
}

// Frame id from ROS to Gazebo is not supported right now
// std::string frame_id_ros_to_gz(const std::string &frame_id)
// {
//   return replace_delimiter(frame_id, "/", "::");
// }

std::string frame_id_gz_to_ros(const std::string &frame_id)
{
  return replace_delimiter(frame_id, "::", "/");
}

template<>
void
convert_ros_to_gz(
  const std_msgs::Bool & ros_msg,
  gz::msgs::Boolean & gz_msg)
{
  gz_msg.set_data(ros_msg.data);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Boolean & gz_msg,
  std_msgs::Bool & ros_msg)
{
  ros_msg.data = gz_msg.data();
}

template<>
void
convert_ros_to_gz(
  const std_msgs::ColorRGBA & ros_msg,
  gz::msgs::Color & gz_msg)
{
  gz_msg.set_r(ros_msg.r);
  gz_msg.set_g(ros_msg.g);
  gz_msg.set_b(ros_msg.b);
  gz_msg.set_a(ros_msg.a);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Color & gz_msg,
  std_msgs::ColorRGBA & ros_msg)
{
  ros_msg.r = gz_msg.r();
  ros_msg.g = gz_msg.g();
  ros_msg.b = gz_msg.b();
  ros_msg.a = gz_msg.a();
}

template<>
void
convert_ros_to_gz(
  const std_msgs::Empty &,
  gz::msgs::Empty &)
{
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Empty &,
  std_msgs::Empty &)
{
}

template<>
void
convert_ros_to_gz(
  const std_msgs::Int32 & ros_msg,
  gz::msgs::Int32 & gz_msg)
{
  gz_msg.set_data(ros_msg.data);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Int32 & gz_msg,
  std_msgs::Int32 & ros_msg)
{
  ros_msg.data = gz_msg.data();
}

template<>
void
convert_ros_to_gz(
  const std_msgs::Float32 & ros_msg,
  gz::msgs::Float & gz_msg)
{
  gz_msg.set_data(ros_msg.data);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Float & gz_msg,
  std_msgs::Float32 & ros_msg)
{
  ros_msg.data = gz_msg.data();
}

template<>
void
convert_ros_to_gz(
  const std_msgs::Float64 & ros_msg,
  gz::msgs::Double & gz_msg)
{
  gz_msg.set_data(ros_msg.data);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Double & gz_msg,
  std_msgs::Float64 & ros_msg)
{
  ros_msg.data = gz_msg.data();
}

template<>
void
convert_ros_to_gz(
  const std_msgs::Header & ros_msg,
  gz::msgs::Header & gz_msg)
{
  gz_msg.mutable_stamp()->set_sec(ros_msg.stamp.sec);
  gz_msg.mutable_stamp()->set_nsec(ros_msg.stamp.nsec);
  auto newPair = gz_msg.add_data();
  newPair->set_key("seq");
  newPair->add_value(std::to_string(ros_msg.seq));
  newPair = gz_msg.add_data();
  newPair->set_key("frame_id");
  newPair->add_value(ros_msg.frame_id);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Header & gz_msg,
  std_msgs::Header & ros_msg)
{
  ros_msg.stamp = ros::Time(gz_msg.stamp().sec(), gz_msg.stamp().nsec());
  for (auto i = 0; i < gz_msg.data_size(); ++i)
  {
    auto aPair = gz_msg.data(i);
    if (aPair.key() == "seq" && aPair.value_size() > 0)
    {
      std::string value = aPair.value(0);
      try
      {
        unsigned long ul = std::stoul(value, nullptr);
        ros_msg.seq = ul;
      }
      catch (std::exception & e)
      {
        ROS_ERROR_STREAM("Exception converting [" << value << "] to an "
                  << "unsigned int" << std::endl);
      }
    }
    else if (aPair.key() == "frame_id" && aPair.value_size() > 0)
    {
      ros_msg.frame_id = frame_id_gz_to_ros(aPair.value(0));
    }
  }
}

template<>
void
convert_ros_to_gz(
  const std_msgs::String & ros_msg,
  gz::msgs::StringMsg & gz_msg)
{
  gz_msg.set_data(ros_msg.data);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::StringMsg & gz_msg,
  std_msgs::String & ros_msg)
{
  ros_msg.data = gz_msg.data();
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Clock & gz_msg,
  rosgraph_msgs::Clock & ros_msg)
{
  ros_msg.clock = ros::Time(gz_msg.sim().sec(), gz_msg.sim().nsec());
}

template<>
void
convert_ros_to_gz(
  const rosgraph_msgs::Clock & ros_msg,
  gz::msgs::Clock & gz_msg)
{
  gz_msg.mutable_sim()->set_sec(ros_msg.clock.sec);
  gz_msg.mutable_sim()->set_nsec(ros_msg.clock.nsec);
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::Quaternion & ros_msg,
  gz::msgs::Quaternion & gz_msg)
{
  gz_msg.set_x(ros_msg.x);
  gz_msg.set_y(ros_msg.y);
  gz_msg.set_z(ros_msg.z);
  gz_msg.set_w(ros_msg.w);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Quaternion & gz_msg,
  geometry_msgs::Quaternion & ros_msg)
{
  ros_msg.x = gz_msg.x();
  ros_msg.y = gz_msg.y();
  ros_msg.z = gz_msg.z();
  ros_msg.w = gz_msg.w();
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::Vector3 & ros_msg,
  gz::msgs::Vector3d & gz_msg)
{
  gz_msg.set_x(ros_msg.x);
  gz_msg.set_y(ros_msg.y);
  gz_msg.set_z(ros_msg.z);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Vector3d & gz_msg,
  geometry_msgs::Vector3 & ros_msg)
{
  ros_msg.x = gz_msg.x();
  ros_msg.y = gz_msg.y();
  ros_msg.z = gz_msg.z();
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::Point & ros_msg,
  gz::msgs::Vector3d & gz_msg)
{
  gz_msg.set_x(ros_msg.x);
  gz_msg.set_y(ros_msg.y);
  gz_msg.set_z(ros_msg.z);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Vector3d & gz_msg,
  geometry_msgs::Point & ros_msg)
{
  ros_msg.x = gz_msg.x();
  ros_msg.y = gz_msg.y();
  ros_msg.z = gz_msg.z();
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::Pose & ros_msg,
  gz::msgs::Pose & gz_msg)
{
  convert_ros_to_gz(ros_msg.position, *gz_msg.mutable_position());
  convert_ros_to_gz(ros_msg.orientation, *gz_msg.mutable_orientation());
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::Pose & ros_msg)
{
  convert_gz_to_ros(gz_msg.position(), ros_msg.position);
  convert_gz_to_ros(gz_msg.orientation(), ros_msg.orientation);
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::PoseArray & ros_msg,
  gz::msgs::Pose_V & gz_msg)
{
  gz_msg.clear_pose();
  for (auto const &t : ros_msg.poses)
  {
    auto p = gz_msg.add_pose();
    convert_ros_to_gz(t, *p);
  }

  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Pose_V & gz_msg,
  geometry_msgs::PoseArray & ros_msg)
{
  ros_msg.poses.clear();
  for (auto const &p : gz_msg.pose())
  {
    geometry_msgs::Pose pose;
    convert_gz_to_ros(p, pose);
    ros_msg.poses.push_back(pose);
  }
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::PoseStamped & ros_msg,
  gz::msgs::Pose & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  convert_ros_to_gz(ros_msg.pose, gz_msg);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::PoseStamped & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  convert_gz_to_ros(gz_msg, ros_msg.pose);
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::Transform & ros_msg,
  gz::msgs::Pose & gz_msg)
{
  convert_ros_to_gz(ros_msg.translation , *gz_msg.mutable_position());
  convert_ros_to_gz(ros_msg.rotation, *gz_msg.mutable_orientation());
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::Transform & ros_msg)
{
  convert_gz_to_ros(gz_msg.position(), ros_msg.translation);
  convert_gz_to_ros(gz_msg.orientation(), ros_msg.rotation);
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::TransformStamped & ros_msg,
  gz::msgs::Pose & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  convert_ros_to_gz(ros_msg.transform, gz_msg);

  auto newPair = gz_msg.mutable_header()->add_data();
  newPair->set_key("child_frame_id");
  newPair->add_value(ros_msg.child_frame_id);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::TransformStamped & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  convert_gz_to_ros(gz_msg, ros_msg.transform);
  for (auto i = 0; i < gz_msg.header().data_size(); ++i)
  {
    auto aPair = gz_msg.header().data(i);
    if (aPair.key() == "child_frame_id" && aPair.value_size() > 0)
    {
      ros_msg.child_frame_id = frame_id_gz_to_ros(aPair.value(0));
      break;
    }
  }
}

template<>
void
convert_ros_to_gz(
  const tf2_msgs::TFMessage & ros_msg,
  gz::msgs::Pose_V & gz_msg)
{
  gz_msg.clear_pose();
  for (auto const &t : ros_msg.transforms)
  {
    auto p = gz_msg.add_pose();
    convert_ros_to_gz(t, *p);
  }

  if (!ros_msg.transforms.empty())
  {
    convert_ros_to_gz(ros_msg.transforms[0].header,
        (*gz_msg.mutable_header()));
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Pose_V & gz_msg,
  tf2_msgs::TFMessage & ros_msg)
{
  ros_msg.transforms.clear();
  for (auto const &p : gz_msg.pose())
  {
    geometry_msgs::TransformStamped tf;
    convert_gz_to_ros(p, tf);
    ros_msg.transforms.push_back(tf);
  }
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::Twist & ros_msg,
  gz::msgs::Twist & gz_msg)
{
  convert_ros_to_gz(ros_msg.linear,  (*gz_msg.mutable_linear()));
  convert_ros_to_gz(ros_msg.angular, (*gz_msg.mutable_angular()));
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Twist & gz_msg,
  geometry_msgs::Twist & ros_msg)
{
  convert_gz_to_ros(gz_msg.linear(), ros_msg.linear);
  convert_gz_to_ros(gz_msg.angular(), ros_msg.angular);
}

// template<>
// void
// convert_ros_to_gz(
//   const mav_msgs::Actuators & ros_msg,
//   gz::msgs::Actuators & gz_msg)
// {
//   convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
//
//   for (auto i = 0u; i < ros_msg.angles.size(); ++i)
//     gz_msg.add_position(ros_msg.angles[i]);
//   for (auto i = 0u; i < ros_msg.angular_velocities.size(); ++i)
//     gz_msg.add_velocity(ros_msg.angular_velocities[i]);
//   for (auto i = 0u; i < ros_msg.normalized.size(); ++i)
//     gz_msg.add_normalized(ros_msg.normalized[i]);
// }
//
// template<>
// void
// convert_gz_to_ros(
//   const gz::msgs::Actuators & gz_msg,
//   mav_msgs::Actuators & ros_msg)
// {
//   convert_gz_to_ros(gz_msg.header(), ros_msg.header);
//
//   for (auto i = 0; i < gz_msg.position_size(); ++i)
//     ros_msg.angles.push_back(gz_msg.position(i));
//   for (auto i = 0; i < gz_msg.velocity_size(); ++i)
//     ros_msg.angular_velocities.push_back(gz_msg.velocity(i));
//   for (auto i = 0; i < gz_msg.normalized_size(); ++i)
//     ros_msg.normalized.push_back(gz_msg.normalized(i));
// }

template<>
void
convert_ros_to_gz(
  const nav_msgs::OccupancyGrid & ros_msg,
  gz::msgs::OccupancyGrid & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  gz_msg.mutable_info()->mutable_map_load_time()->set_sec(
      ros_msg.info.map_load_time.sec);
  gz_msg.mutable_info()->mutable_map_load_time()->set_nsec(
      ros_msg.info.map_load_time.nsec);

  gz_msg.mutable_info()->set_resolution(
      ros_msg.info.resolution);
  gz_msg.mutable_info()->set_width(
      ros_msg.info.width);
  gz_msg.mutable_info()->set_height(
      ros_msg.info.height);

  convert_ros_to_gz(ros_msg.info.origin,
      (*gz_msg.mutable_info()->mutable_origin()));

  gz_msg.set_data(&ros_msg.data[0], ros_msg.data.size());
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::OccupancyGrid & gz_msg,
  nav_msgs::OccupancyGrid & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  ros_msg.info.map_load_time.sec = gz_msg.info().map_load_time().sec();
  ros_msg.info.map_load_time.nsec = gz_msg.info().map_load_time().nsec();
  ros_msg.info.resolution = gz_msg.info().resolution();
  ros_msg.info.width = gz_msg.info().width();
  ros_msg.info.height = gz_msg.info().height();

  convert_gz_to_ros(gz_msg.info().origin(), ros_msg.info.origin);

  ros_msg.data.resize(gz_msg.data().size());
  memcpy(&ros_msg.data[0], gz_msg.data().c_str(), gz_msg.data().size());
}

template<>
void
convert_ros_to_gz(
  const nav_msgs::Odometry & ros_msg,
  gz::msgs::Odometry & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  convert_ros_to_gz(ros_msg.pose.pose, (*gz_msg.mutable_pose()));
  convert_ros_to_gz(ros_msg.twist.twist, (*gz_msg.mutable_twist()));

  auto childFrame = gz_msg.mutable_header()->add_data();
  childFrame->set_key("child_frame_id");
  childFrame->add_value(ros_msg.child_frame_id);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Odometry & gz_msg,
  nav_msgs::Odometry & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  convert_gz_to_ros(gz_msg.pose(), ros_msg.pose.pose);
  convert_gz_to_ros(gz_msg.twist(), ros_msg.twist.twist);

  for (auto i = 0; i < gz_msg.header().data_size(); ++i)
  {
    auto aPair = gz_msg.header().data(i);
    if (aPair.key() == "child_frame_id" && aPair.value_size() > 0)
    {
      ros_msg.child_frame_id = frame_id_gz_to_ros(aPair.value(0));
      break;
    }
  }
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::FluidPressure & ros_msg,
  gz::msgs::FluidPressure & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  gz_msg.set_pressure(ros_msg.fluid_pressure);
  gz_msg.set_variance(ros_msg.variance);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::FluidPressure & gz_msg,
  sensor_msgs::FluidPressure & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  ros_msg.fluid_pressure = gz_msg.pressure();
  ros_msg.variance = gz_msg.variance();
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::Image & ros_msg,
  gz::msgs::Image & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  gz_msg.set_width(ros_msg.width);
  gz_msg.set_height(ros_msg.height);

  unsigned int num_channels;
  unsigned int octets_per_channel;

  if (ros_msg.encoding == "mono8")
  {
    gz_msg.set_pixel_format_type(
      gz::msgs::PixelFormatType::L_INT8);
    num_channels = 1;
    octets_per_channel = 1u;
  }
  else if (ros_msg.encoding == "mono16")
  {
    gz_msg.set_pixel_format_type(
      gz::msgs::PixelFormatType::L_INT16);
    num_channels = 1;
    octets_per_channel = 2u;
  }
  else if (ros_msg.encoding == "rgb8")
  {
    gz_msg.set_pixel_format_type(
      gz::msgs::PixelFormatType::RGB_INT8);
    num_channels = 3;
    octets_per_channel = 1u;
  }
  else if (ros_msg.encoding == "rgba8")
  {
    gz_msg.set_pixel_format_type(
      gz::msgs::PixelFormatType::RGBA_INT8);
    num_channels = 4;
    octets_per_channel = 1u;
  }
  else if (ros_msg.encoding == "bgra8")
  {
    gz_msg.set_pixel_format_type(
      gz::msgs::PixelFormatType::BGRA_INT8);
    num_channels = 4;
    octets_per_channel = 1u;
  }
  else if (ros_msg.encoding == "rgb16")
  {
    gz_msg.set_pixel_format_type(
      gz::msgs::PixelFormatType::RGB_INT16);
    num_channels = 3;
    octets_per_channel = 2u;
  }
  else if (ros_msg.encoding == "bgr8")
  {
    gz_msg.set_pixel_format_type(
      gz::msgs::PixelFormatType::BGR_INT8);
    num_channels = 3;
    octets_per_channel = 1u;
  }
  else if (ros_msg.encoding == "bgr16")
  {
    gz_msg.set_pixel_format_type(
      gz::msgs::PixelFormatType::BGR_INT16);
    num_channels = 3;
    octets_per_channel = 2u;
  }
  else if (ros_msg.encoding == "32FC1")
  {
    gz_msg.set_pixel_format_type(
      gz::msgs::PixelFormatType::R_FLOAT32);
    num_channels = 1;
    octets_per_channel = 4u;
  }
  else
  {
    gz_msg.set_pixel_format_type(
      gz::msgs::PixelFormatType::UNKNOWN_PIXEL_FORMAT);
    ROS_ERROR_STREAM("Unsupported pixel format [" << ros_msg.encoding << "]"
              << std::endl);
    return;
  }

  gz_msg.set_step(gz_msg.width() * num_channels * octets_per_channel);

  gz_msg.set_data(&(ros_msg.data[0]), gz_msg.step() * gz_msg.height());
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Image & gz_msg,
  sensor_msgs::Image & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  ros_msg.height = gz_msg.height();
  ros_msg.width = gz_msg.width();

  unsigned int num_channels;
  unsigned int octets_per_channel;

  if (gz_msg.pixel_format_type() ==
      gz::msgs::PixelFormatType::L_INT8)
  {
    ros_msg.encoding = "mono8";
    num_channels = 1;
    octets_per_channel = 1u;
  }
  else if (gz_msg.pixel_format_type() ==
      gz::msgs::PixelFormatType::L_INT16)
  {
    ros_msg.encoding = "mono16";
    num_channels = 1;
    octets_per_channel = 2u;
  }
  else if (gz_msg.pixel_format_type() ==
      gz::msgs::PixelFormatType::RGB_INT8)
  {
    ros_msg.encoding = "rgb8";
    num_channels = 3;
    octets_per_channel = 1u;
  }
  else if (gz_msg.pixel_format_type() ==
      gz::msgs::PixelFormatType::RGBA_INT8)
  {
    ros_msg.encoding = "rgba8";
    num_channels = 4;
    octets_per_channel = 1u;
  }
  else if (gz_msg.pixel_format_type() ==
      gz::msgs::PixelFormatType::BGRA_INT8)
  {
    ros_msg.encoding = "bgra8";
    num_channels = 4;
    octets_per_channel = 1u;
  }
  else if (gz_msg.pixel_format_type() ==
      gz::msgs::PixelFormatType::RGB_INT16)
  {
    ros_msg.encoding = "rgb16";
    num_channels = 3;
    octets_per_channel = 2u;
  }
  else if (gz_msg.pixel_format_type() ==
      gz::msgs::PixelFormatType::BGR_INT8)
  {
    ros_msg.encoding = "bgr8";
    num_channels = 3;
    octets_per_channel = 1u;
  }
  else if (gz_msg.pixel_format_type() ==
      gz::msgs::PixelFormatType::BGR_INT16)
  {
    ros_msg.encoding = "bgr16";
    num_channels = 3;
    octets_per_channel = 2u;
  }
  else if (gz_msg.pixel_format_type() ==
      gz::msgs::PixelFormatType::R_FLOAT32)
  {
    ros_msg.encoding = "32FC1";
    num_channels = 1;
    octets_per_channel = 4u;
  }
  else
  {
    ROS_ERROR_STREAM("Unsupported pixel format ["
        << gz_msg.pixel_format_type() << "]" << std::endl);
    return;
  }

  ros_msg.is_bigendian = false;
  ros_msg.step = ros_msg.width * num_channels * octets_per_channel;

  auto count = ros_msg.step * ros_msg.height;
  ros_msg.data.resize(ros_msg.step * ros_msg.height);
  std::copy(
    gz_msg.data().begin(),
    gz_msg.data().begin() + count,
    ros_msg.data.begin());
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::CameraInfo & ros_msg,
  gz::msgs::CameraInfo & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  gz_msg.set_width(ros_msg.width);
  gz_msg.set_height(ros_msg.height);

  auto distortion = gz_msg.mutable_distortion();
  if (ros_msg.distortion_model == "plumb_bob")
  {
    distortion->set_model(gz::msgs::CameraInfo::Distortion::PLUMB_BOB);
  }
  else if (ros_msg.distortion_model == "rational_polynomial")
  {
    distortion->set_model(gz::msgs::CameraInfo::Distortion::RATIONAL_POLYNOMIAL);
  }
  else if (ros_msg.distortion_model == "equidistant")
  {
    distortion->set_model(gz::msgs::CameraInfo::Distortion::EQUIDISTANT);
  }
  else
  {
    ROS_ERROR_STREAM("Unsupported distortion model ["
        << ros_msg.distortion_model << "]" << std::endl);
  }
  for (auto i = 0u; i < ros_msg.D.size(); ++i)
  {
    distortion->add_k(ros_msg.D[i]);
  }

  auto intrinsics = gz_msg.mutable_intrinsics();
  for (auto i = 0u; i < ros_msg.K.size(); ++i)
  {
    intrinsics->add_k(ros_msg.K[i]);
  }

  auto projection = gz_msg.mutable_projection();
  for (auto i = 0u; i < ros_msg.P.size(); ++i)
  {
    projection->add_p(ros_msg.P[i]);
  }

  for (auto i = 0u; i < ros_msg.R.size(); ++i)
  {
    gz_msg.add_rectification_matrix(ros_msg.R[i]);
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::CameraInfo & gz_msg,
  sensor_msgs::CameraInfo & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  ros_msg.height = gz_msg.height();
  ros_msg.width = gz_msg.width();

  auto distortion = gz_msg.distortion();
  if (gz_msg.has_distortion())
  {
    auto distortion = gz_msg.distortion();
    if (distortion.model() ==
        gz::msgs::CameraInfo::Distortion::PLUMB_BOB)
    {
      ros_msg.distortion_model = "plumb_bob";
    }
    else if (distortion.model() ==
        gz::msgs::CameraInfo::Distortion::RATIONAL_POLYNOMIAL)
    {
      ros_msg.distortion_model = "rational_polynomial";
    }
    else if (distortion.model() ==
        gz::msgs::CameraInfo::Distortion::EQUIDISTANT)
    {
      ros_msg.distortion_model = "equidistant";
    }
    else
    {
      ROS_ERROR_STREAM("Unsupported distortion model ["
                << distortion.model() << "]" << std::endl);
    }

    ros_msg.D.resize(distortion.k_size());
    for (auto i = 0; i < distortion.k_size(); ++i)
    {
      ros_msg.D[i] = distortion.k(i);
    }
  }

  auto intrinsics = gz_msg.intrinsics();
  if (gz_msg.has_intrinsics())
  {
    auto intrinsics = gz_msg.intrinsics();

    for (auto i = 0; i < intrinsics.k_size(); ++i)
    {
      ros_msg.K[i] = intrinsics.k(i);
    }
  }

  auto projection = gz_msg.projection();
  if (gz_msg.has_projection())
  {
    auto projection = gz_msg.projection();

    for (auto i = 0; i < projection.p_size(); ++i)
    {
      ros_msg.P[i] = projection.p(i);
    }
  }

  for (auto i = 0; i < gz_msg.rectification_matrix_size(); ++i)
  {
    ros_msg.R[i] = gz_msg.rectification_matrix(i);
  }
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::Imu & ros_msg,
  gz::msgs::IMU & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  // ToDo: Verify that this is the expected value (probably not).
  gz_msg.set_entity_name(ros_msg.header.frame_id);

  if (!gz::math::equal(ros_msg.orientation_covariance[0], -1.0))
  {
    // -1 in orientation covariance matrix means there are no orientation
    // values, see
    // http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html
    convert_ros_to_gz(ros_msg.orientation, (*gz_msg.mutable_orientation()));
  }

  convert_ros_to_gz(ros_msg.angular_velocity,
                   (*gz_msg.mutable_angular_velocity()));
  convert_ros_to_gz(ros_msg.linear_acceleration,
                   (*gz_msg.mutable_linear_acceleration()));
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::IMU & gz_msg,
  sensor_msgs::Imu & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  if (gz_msg.has_orientation())
  {
    convert_gz_to_ros(gz_msg.orientation(), ros_msg.orientation);
  }
  else
  {
    // ign may not publish orientation values.
    // So set 1st element of orientation covariance matrix to -1 to indicate
    // there are no orientation estimates, see ROS imu msg documentation:
    // http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html
    ros_msg.orientation_covariance[0] = -1.0f;
  }

  convert_gz_to_ros(gz_msg.angular_velocity(), ros_msg.angular_velocity);
  convert_gz_to_ros(gz_msg.linear_acceleration(), ros_msg.linear_acceleration);

  // Covariances not supported in gz::msgs::IMU
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::JointState & ros_msg,
  gz::msgs::Model & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  const auto nan = std::numeric_limits<double>::quiet_NaN();
  for (auto i = 0u; i < ros_msg.name.size(); ++i)
  {
    auto newJoint = gz_msg.add_joint();
    newJoint->set_name(ros_msg.name[i]);

    if (ros_msg.position.size() > i)
      newJoint->mutable_axis1()->set_position(ros_msg.position[i]);
    else
      newJoint->mutable_axis1()->set_position(nan);

    if (ros_msg.velocity.size() > i)
      newJoint->mutable_axis1()->set_velocity(ros_msg.velocity[i]);
    else
      newJoint->mutable_axis1()->set_velocity(nan);

    if (ros_msg.effort.size() > i)
      newJoint->mutable_axis1()->set_force(ros_msg.effort[i]);
    else
      newJoint->mutable_axis1()->set_force(nan);
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Model & gz_msg,
  sensor_msgs::JointState & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  for (auto i = 0; i < gz_msg.joint_size(); ++i)
  {
    ros_msg.name.push_back(gz_msg.joint(i).name());
    ros_msg.position.push_back(gz_msg.joint(i).axis1().position());
    ros_msg.velocity.push_back(gz_msg.joint(i).axis1().velocity());
    ros_msg.effort.push_back(gz_msg.joint(i).axis1().force());
  }
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::LaserScan & ros_msg,
  gz::msgs::LaserScan & gz_msg)
{
  const unsigned int num_readings =
    (ros_msg.angle_max - ros_msg.angle_min) / ros_msg.angle_increment;

  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  gz_msg.set_frame(ros_msg.header.frame_id);
  gz_msg.set_angle_min(ros_msg.angle_min);
  gz_msg.set_angle_max(ros_msg.angle_max);
  gz_msg.set_angle_step(ros_msg.angle_increment);
  gz_msg.set_range_min(ros_msg.range_min);
  gz_msg.set_range_max(ros_msg.range_max);
  gz_msg.set_count(num_readings);

  // Not supported in sensor_msgs::LaserScan.
  gz_msg.set_vertical_angle_min(0.0);
  gz_msg.set_vertical_angle_max(0.0);
  gz_msg.set_vertical_angle_step(0.0);
  gz_msg.set_vertical_count(0u);

  for (auto i = 0u; i < gz_msg.count(); ++i)
  {
    gz_msg.add_ranges(ros_msg.ranges[i]);
    gz_msg.add_intensities(ros_msg.intensities[i]);
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::LaserScan & gz_msg,
  sensor_msgs::LaserScan & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  ros_msg.header.frame_id = frame_id_gz_to_ros(gz_msg.frame());

  ros_msg.angle_min = gz_msg.angle_min();
  ros_msg.angle_max = gz_msg.angle_max();
  ros_msg.angle_increment = gz_msg.angle_step();

  // Not supported in gz::msgs::LaserScan.
  ros_msg.time_increment = 0.0;
  ros_msg.scan_time = 0.0;

  ros_msg.range_min = gz_msg.range_min();
  ros_msg.range_max = gz_msg.range_max();

  auto count = gz_msg.count();
  auto vertical_count = gz_msg.vertical_count();

  // If there are multiple vertical beams, use the one in the middle.
  size_t start = (vertical_count / 2) * count;

  // Copy ranges into ROS message.
  ros_msg.ranges.resize(count);
  std::copy(
    gz_msg.ranges().begin() + start,
    gz_msg.ranges().begin() + start + count,
    ros_msg.ranges.begin());

  // Copy intensities into ROS message.
  ros_msg.intensities.resize(count);
  std::copy(
    gz_msg.intensities().begin() + start,
    gz_msg.intensities().begin() + start + count,
    ros_msg.intensities.begin());
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::MagneticField & ros_msg,
  gz::msgs::Magnetometer & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  convert_ros_to_gz(ros_msg.magnetic_field, (*gz_msg.mutable_field_tesla()));
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Magnetometer & gz_msg,
  sensor_msgs::MagneticField & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  convert_gz_to_ros(gz_msg.field_tesla(), ros_msg.magnetic_field);

  // magnetic_field_covariance is not supported in gz::Msgs::Magnetometer.
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::NavSatFix & ros_msg,
  gz::msgs::NavSat & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  gz_msg.set_latitude_deg(ros_msg.latitude);
  gz_msg.set_longitude_deg(ros_msg.longitude);
  gz_msg.set_altitude(ros_msg.altitude);
  gz_msg.set_frame_id(ros_msg.header.frame_id);

  // Not supported in sensor_msgs::NavSatFix.
  gz_msg.set_velocity_east(0.0);
  gz_msg.set_velocity_north(0.0);
  gz_msg.set_velocity_up(0.0);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::NavSat & gz_msg,
  sensor_msgs::NavSatFix & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  ros_msg.header.frame_id = frame_id_gz_to_ros(gz_msg.frame_id());
  ros_msg.latitude = gz_msg.latitude_deg();
  ros_msg.longitude = gz_msg.longitude_deg();
  ros_msg.altitude = gz_msg.altitude();

  // position_covariance is not supported in gz::Msgs::NavSat.
  ros_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  ros_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::PointCloud2 & ros_msg,
  gz::msgs::PointCloudPacked &gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  gz_msg.set_height(ros_msg.height);
  gz_msg.set_width(ros_msg.width);
  gz_msg.set_is_bigendian(ros_msg.is_bigendian);
  gz_msg.set_point_step(ros_msg.point_step);
  gz_msg.set_row_step(ros_msg.row_step);
  gz_msg.set_is_dense(ros_msg.is_dense);
  gz_msg.mutable_data()->resize(ros_msg.data.size());
  memcpy(gz_msg.mutable_data()->data(), ros_msg.data.data(),
         ros_msg.data.size());

  for (unsigned int i = 0; i < ros_msg.fields.size(); ++i)
  {
    gz::msgs::PointCloudPacked::Field *pf = gz_msg.add_field();
    pf->set_name(ros_msg.fields[i].name);
    pf->set_count(ros_msg.fields[i].count);
    pf->set_offset(ros_msg.fields[i].offset);
    switch (ros_msg.fields[i].datatype)
    {
      default:
      case sensor_msgs::PointField::INT8:
        pf->set_datatype(gz::msgs::PointCloudPacked::Field::INT8);
        break;
      case sensor_msgs::PointField::UINT8:
        pf->set_datatype(gz::msgs::PointCloudPacked::Field::UINT8);
        break;
      case sensor_msgs::PointField::INT16:
        pf->set_datatype(gz::msgs::PointCloudPacked::Field::INT16);
        break;
      case sensor_msgs::PointField::UINT16:
        pf->set_datatype(gz::msgs::PointCloudPacked::Field::UINT16);
        break;
      case sensor_msgs::PointField::INT32:
        pf->set_datatype(gz::msgs::PointCloudPacked::Field::INT32);
        break;
      case sensor_msgs::PointField::UINT32:
        pf->set_datatype(gz::msgs::PointCloudPacked::Field::UINT32);
        break;
      case sensor_msgs::PointField::FLOAT32:
        pf->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
        break;
      case sensor_msgs::PointField::FLOAT64:
        pf->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT64);
        break;
    };
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::PointCloudPacked & gz_msg,
  sensor_msgs::PointCloud2 & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  ros_msg.height = gz_msg.height();
  ros_msg.width = gz_msg.width();
  ros_msg.is_bigendian = gz_msg.is_bigendian();
  ros_msg.point_step = gz_msg.point_step();
  ros_msg.row_step = gz_msg.row_step();
  ros_msg.is_dense = gz_msg.is_dense();
  ros_msg.data.resize(gz_msg.data().size());
  memcpy(ros_msg.data.data(), gz_msg.data().c_str(), gz_msg.data().size());

  for (int i = 0; i < gz_msg.field_size(); ++i)
  {
    sensor_msgs::PointField pf;
    pf.name = gz_msg.field(i).name();
    pf.count = gz_msg.field(i).count();
    pf.offset = gz_msg.field(i).offset();
    switch (gz_msg.field(i).datatype())
    {
      default:
      case gz::msgs::PointCloudPacked::Field::INT8:
        pf.datatype = sensor_msgs::PointField::INT8;
        break;
      case gz::msgs::PointCloudPacked::Field::UINT8:
        pf.datatype = sensor_msgs::PointField::UINT8;
        break;
      case gz::msgs::PointCloudPacked::Field::INT16:
        pf.datatype = sensor_msgs::PointField::INT16;
        break;
      case gz::msgs::PointCloudPacked::Field::UINT16:
        pf.datatype = sensor_msgs::PointField::UINT16;
        break;
      case gz::msgs::PointCloudPacked::Field::INT32:
        pf.datatype = sensor_msgs::PointField::INT32;
        break;
      case gz::msgs::PointCloudPacked::Field::UINT32:
        pf.datatype = sensor_msgs::PointField::UINT32;
        break;
      case gz::msgs::PointCloudPacked::Field::FLOAT32:
        pf.datatype = sensor_msgs::PointField::FLOAT32;
        break;
      case gz::msgs::PointCloudPacked::Field::FLOAT64:
        pf.datatype = sensor_msgs::PointField::FLOAT64;
        break;
    };
    ros_msg.fields.push_back(pf);
  }
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::BatteryState & ros_msg,
  gz::msgs::BatteryState & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  gz_msg.set_voltage(ros_msg.voltage);
  gz_msg.set_current(ros_msg.current);
  gz_msg.set_charge(ros_msg.charge);
  gz_msg.set_capacity(ros_msg.capacity);
  gz_msg.set_percentage(ros_msg.percentage);

  if (ros_msg.power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN)
  {
    gz_msg.set_power_supply_status(gz::msgs::BatteryState::UNKNOWN);
  }
  else if (ros_msg.power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING)
  {
    gz_msg.set_power_supply_status(gz::msgs::BatteryState::CHARGING);
  }
  else if (ros_msg.power_supply_status ==
      sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING)
  {
    gz_msg.set_power_supply_status(gz::msgs::BatteryState::DISCHARGING);
  }
  else if (ros_msg.power_supply_status ==
      sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING)
  {
    gz_msg.set_power_supply_status(gz::msgs::BatteryState::NOT_CHARGING);
  }
  else if (ros_msg.power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL)
  {
    gz_msg.set_power_supply_status(gz::msgs::BatteryState::FULL);
  }
  else
  {
    ROS_ERROR_STREAM("Unsupported power supply status ["
        << ros_msg.power_supply_status << "]" << std::endl);
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::BatteryState & gz_msg,
  sensor_msgs::BatteryState & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  ros_msg.voltage = gz_msg.voltage();
  ros_msg.current = gz_msg.current();
  ros_msg.charge = gz_msg.charge();
  ros_msg.capacity = gz_msg.capacity();
  ros_msg.design_capacity = std::numeric_limits<double>::quiet_NaN();
  ros_msg.percentage = gz_msg.percentage();

  if (gz_msg.power_supply_status() ==
      gz::msgs::BatteryState::UNKNOWN)
  {
    ros_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  }
  else if (gz_msg.power_supply_status() ==
      gz::msgs::BatteryState::CHARGING)
  {
    ros_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  }
  else if (gz_msg.power_supply_status() ==
      gz::msgs::BatteryState::DISCHARGING)
  {
    ros_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  }
  else if (gz_msg.power_supply_status() ==
      gz::msgs::BatteryState::NOT_CHARGING)
  {
    ros_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
  }
  else if (gz_msg.power_supply_status() ==
      gz::msgs::BatteryState::FULL)
  {
    ros_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL;
  }
  else
  {
    ROS_ERROR_STREAM("Unsupported power supply status ["
              << gz_msg.power_supply_status() << "]" << std::endl);
  }

  ros_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  ros_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  ros_msg.present = true;
}

template<>
void
convert_ros_to_gz(
    const visualization_msgs::Marker & ros_msg,
    gz::msgs::Marker & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  // Note, in ROS's Marker message ADD and MODIFY both map to a value of "0",
  // so that case is not needed here.
  switch(ros_msg.action)
  {
    case visualization_msgs::Marker::ADD:
      gz_msg.set_action(gz::msgs::Marker::ADD_MODIFY);
      break;
    case visualization_msgs::Marker::DELETE:
      gz_msg.set_action(gz::msgs::Marker::DELETE_MARKER);
      break;
    case visualization_msgs::Marker::DELETEALL:
      gz_msg.set_action(gz::msgs::Marker::DELETE_ALL);
      break;
    default:
      ROS_ERROR_STREAM("Unknown visualization_msgs::Marker action [" <<
          ros_msg.action << "]\n");
      break;
  }

  gz_msg.set_ns(ros_msg.ns);
  gz_msg.set_id(ros_msg.id);
  // gz_msg.set_layer();  // No "layer" concept in ROS

  // Type
  switch(ros_msg.type)
  {
    case visualization_msgs::Marker::ARROW:
      gz_msg.set_type(gz::msgs::Marker::ARROW);
      break;
    case visualization_msgs::Marker::CUBE:
      gz_msg.set_type(gz::msgs::Marker::BOX);
      break;
    case visualization_msgs::Marker::SPHERE:
      gz_msg.set_type(gz::msgs::Marker::SPHERE);
      break;
    case visualization_msgs::Marker::CYLINDER:
      gz_msg.set_type(gz::msgs::Marker::CYLINDER);
      break;
    case visualization_msgs::Marker::LINE_STRIP:
      gz_msg.set_type(gz::msgs::Marker::LINE_STRIP);
      break;
    case visualization_msgs::Marker::LINE_LIST:
      gz_msg.set_type(gz::msgs::Marker::LINE_LIST);
      break;
    case visualization_msgs::Marker::CUBE_LIST:
      ROS_ERROR_STREAM("Unsupported visualization_msgs::Marker type" <<
          "[CUBE_LIST]\n");
      break;
    case visualization_msgs::Marker::SPHERE_LIST:
      ROS_ERROR_STREAM("Unsupported visualization_msgs::Marker type" <<
          "[SPHERE_LIST]\n");
      break;
    case visualization_msgs::Marker::POINTS:
      gz_msg.set_type(gz::msgs::Marker::POINTS);
      break;
    case visualization_msgs::Marker::TEXT_VIEW_FACING:
      gz_msg.set_type(gz::msgs::Marker::TEXT);
      break;
    case visualization_msgs::Marker::MESH_RESOURCE:
      ROS_ERROR_STREAM("Unsupported visualization_msgs::Marker type" <<
          "[MESH_RESOURCE]\n");
      break;
    case visualization_msgs::Marker::TRIANGLE_LIST:
      gz_msg.set_type(gz::msgs::Marker::TRIANGLE_LIST);
      break;
    default:
      ROS_ERROR_STREAM("Unknown visualization_msgs::Marker type [" <<
          ros_msg.type << "]\n");
      break;
  }

  // Lifetime
  gz_msg.mutable_lifetime()->set_sec(ros_msg.lifetime.sec);
  gz_msg.mutable_lifetime()->set_nsec(ros_msg.lifetime.nsec);

  // Pose
  convert_ros_to_gz(ros_msg.pose, *gz_msg.mutable_pose());

  // Scale
  convert_ros_to_gz(ros_msg.scale, *gz_msg.mutable_scale());

  // Material
  convert_ros_to_gz(ros_msg.color, *gz_msg.mutable_material()->mutable_ambient());
  convert_ros_to_gz(ros_msg.color, *gz_msg.mutable_material()->mutable_diffuse());
  convert_ros_to_gz(ros_msg.color, *gz_msg.mutable_material()->mutable_specular());

  // Point
  gz_msg.clear_point();
  for (auto const &pt : ros_msg.points)
  {
    auto p = gz_msg.add_point();
    convert_ros_to_gz(pt, *p);
  }

  gz_msg.set_text(ros_msg.text);

  // gz_msg.set_parent();  // No "parent" concept in ROS
  // gz_msg.set_visibility();  // No "visibility" concept in ROS
}

template<>
void
convert_gz_to_ros(
    const gz::msgs::Marker & gz_msg,
    visualization_msgs::Marker & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  switch(gz_msg.action())
  {
    case gz::msgs::Marker::ADD_MODIFY:
      ros_msg.action = visualization_msgs::Marker::ADD;
      break;
    case gz::msgs::Marker::DELETE_MARKER:
      ros_msg.action = visualization_msgs::Marker::DELETE;
      break;
    case gz::msgs::Marker::DELETE_ALL:
      ros_msg.action = visualization_msgs::Marker::DELETEALL;
      break;
    default:
      ROS_ERROR_STREAM("Unknown gz.msgs.marker action [" <<
          gz_msg.action() << "]\n");
      break;
  }

  ros_msg.ns = gz_msg.ns();
  ros_msg.id = gz_msg.id();

  switch(gz_msg.type())
  {
    case gz::msgs::Marker::ARROW:
      ros_msg.type = visualization_msgs::Marker::TRIANGLE_LIST;
      break;
    case gz::msgs::Marker::AXIS:
      ROS_ERROR_STREAM("Unsupported gz.msgs.marker type " <<
          "[AXIS]\n");
      break;
    case gz::msgs::Marker::CONE:
      ROS_ERROR_STREAM("Unsupported gz.msgs.marker type " <<
          "[CONE]\n");
      break;
    case gz::msgs::Marker::NONE:
      ROS_ERROR_STREAM("Unsupported gz.msgs.marker type " <<
          "[NONE]\n");
      break;
    case gz::msgs::Marker::BOX:
      ros_msg.type = visualization_msgs::Marker::CUBE;
      break;
    case gz::msgs::Marker::CYLINDER:
      ros_msg.type = visualization_msgs::Marker::CYLINDER;
      break;
    case gz::msgs::Marker::LINE_LIST:
      ros_msg.type = visualization_msgs::Marker::LINE_LIST;
      break;
    case gz::msgs::Marker::LINE_STRIP:
      ros_msg.type = visualization_msgs::Marker::LINE_STRIP;
      break;
    case gz::msgs::Marker::POINTS:
      ros_msg.type = visualization_msgs::Marker::POINTS;
      break;
    case gz::msgs::Marker::SPHERE:
      ros_msg.type = visualization_msgs::Marker::SPHERE;
      break;
    case gz::msgs::Marker::TEXT:
      ros_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      break;
    case gz::msgs::Marker::TRIANGLE_FAN:
      ROS_ERROR_STREAM("Unsupported gz.msgs.marker type " <<
          "[TRIANGLE_FAN]\n");
      break;
    case gz::msgs::Marker::TRIANGLE_LIST:
      ros_msg.type = visualization_msgs::Marker::TRIANGLE_LIST;
      break;
    case gz::msgs::Marker::TRIANGLE_STRIP:
      ROS_ERROR_STREAM("Unsupported gz.msgs.marker type " <<
          "[TRIANGLE_STRIP]\n");
      break;
    default:
      ROS_ERROR_STREAM("Unknown gz.msgs.marker type " <<
          "[" << gz_msg.type() << "]\n");
      break;
  }

  // Lifetime
  ros_msg.lifetime.sec = gz_msg.lifetime().sec();
  ros_msg.lifetime.nsec = gz_msg.lifetime().nsec();

  // Pose
  convert_gz_to_ros(gz_msg.pose(), ros_msg.pose);

  // Scale
  convert_gz_to_ros(gz_msg.scale(), ros_msg.scale);

  // Material
  convert_gz_to_ros(gz_msg.material().ambient(), ros_msg.color);
  ros_msg.colors.clear();

  // Points
  ros_msg.points.clear();
  ros_msg.points.reserve(gz_msg.point_size());
  for (auto const & pt: gz_msg.point())
  {
    geometry_msgs::Point p;
    convert_gz_to_ros(pt, p);
    ros_msg.points.push_back(p);
  }

  ros_msg.text = gz_msg.text();
}

template<>
void
convert_ros_to_gz(
    const visualization_msgs::MarkerArray & ros_msg,
    gz::msgs::Marker_V & gz_msg)
{
  gz_msg.clear_header();
  gz_msg.clear_marker();
  for (const auto &marker : ros_msg.markers)
  {
    auto m = gz_msg.add_marker();
    convert_ros_to_gz(marker, *m);
  }
}

template<>
void
convert_gz_to_ros(
    const gz::msgs::Marker_V & gz_msg,
    visualization_msgs::MarkerArray & ros_msg)
{
  ros_msg.markers.clear();
  ros_msg.markers.reserve(gz_msg.marker_size());

  for (auto const &marker : gz_msg.marker())
  {
      visualization_msgs::Marker m;
      convert_gz_to_ros(marker, m);
      ros_msg.markers.push_back(m);
  }
}

}  // namespace ros_gz_bridge
