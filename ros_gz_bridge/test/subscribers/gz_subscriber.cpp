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

#include <gtest/gtest.h>
#include <chrono>
#include <string>
#include <gz/transport.hh>
#include "../test_utils.h"

//////////////////////////////////////////////////
/// \brief A class for testing Gazebo Transport topic subscription.
template <typename GZ_T>
class MyTestClass
{
  /// \brief Class constructor.
  /// \param[in] _topic Topic to subscribe.
  public: MyTestClass(const std::string &_topic)
  {
    EXPECT_TRUE(this->node.Subscribe(_topic, &MyTestClass::Cb, this));
  }

  /// \brief Member function called each time a topic update is received.
  public: void Cb(const GZ_T &_msg)
  {
    ros_gz_bridge::testing::compareTestMsg(_msg);
    this->callbackExecuted = true;
  };

  /// \brief Member variables that flag when the actions are executed.
  public: bool callbackExecuted = false;

  /// \brief Transport node;
  private: gz::transport::Node node;
};

/////////////////////////////////////////////////
TEST(GzSubscriberTest, Boolean)
{
  MyTestClass<gz::msgs::Boolean> client("bool");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, Color)
{
  MyTestClass<gz::msgs::Color> client("color");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, Empty)
{
  MyTestClass<gz::msgs::Empty> client("empty");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, Int32)
{
  MyTestClass<gz::msgs::Int32> client("int32");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, Float)
{
  MyTestClass<gz::msgs::Float> client("float");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, Header)
{
  MyTestClass<gz::msgs::Header> client("header");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, String)
{
  MyTestClass<gz::msgs::StringMsg> client("string");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, Quaternion)
{
  MyTestClass<gz::msgs::Quaternion> client("quaternion");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, Vector3)
{
  MyTestClass<gz::msgs::Vector3d> client("vector3");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, Clock)
{
  MyTestClass<gz::msgs::Clock> client("clock");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, Point)
{
  MyTestClass<gz::msgs::Vector3d> client("point");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, Pose)
{
  MyTestClass<gz::msgs::Pose> client("pose");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, Pose_V)
{
  MyTestClass<gz::msgs::Pose_V> client("pose_array");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, PoseStamped)
{
  MyTestClass<gz::msgs::Pose> client("pose_stamped");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, Transform)
{
  MyTestClass<gz::msgs::Pose> client("transform");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, TransformStamped)
{
  MyTestClass<gz::msgs::Pose> client("transform_stamped");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, TF2Message)
{
  MyTestClass<gz::msgs::Pose_V> client("tf2_message");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, Twist)
{
  MyTestClass<gz::msgs::Twist> client("twist");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, Image)
{
  MyTestClass<gz::msgs::Image> client("image");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, CameraInfo)
{
  MyTestClass<gz::msgs::CameraInfo> client("camera_info");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, FluidPressure)
{
  MyTestClass<gz::msgs::FluidPressure> client("fluid_pressure");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, Imu)
{
  MyTestClass<gz::msgs::IMU> client("imu");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, LaserScan)
{
  MyTestClass<gz::msgs::LaserScan> client("laserscan");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, Magnetometer)
{
  MyTestClass<gz::msgs::Magnetometer> client("magnetic");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, NavSat)
{
  MyTestClass<gz::msgs::NavSat> client("navsat");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
//TEST(GzSubscriberTest, Actuators)
//{
//  MyTestClass<gz::msgs::Actuators> client("actuators");
//
//  using namespace std::chrono_literals;
//  ros_gz_bridge::testing::waitUntilBoolVar(
//    client.callbackExecuted, 10ms, 200);
//
//  EXPECT_TRUE(client.callbackExecuted);
//}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, OccupancyGrid)
{
  MyTestClass<gz::msgs::OccupancyGrid> client("map");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, Odometry)
{
  MyTestClass<gz::msgs::Odometry> client("odometry");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, JointStates)
{
  MyTestClass<gz::msgs::Model> client("joint_states");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, PointCloudPacked)
{
  MyTestClass<gz::msgs::PointCloudPacked> client("pointcloud2");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, BatteryState)
{
  MyTestClass<gz::msgs::BatteryState> client("battery_state");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, Marker)
{
  MyTestClass<gz::msgs::Marker> client("marker");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
TEST(GzSubscriberTest, MarkerArray)
{
  MyTestClass<gz::msgs::Marker_V> client("marker_array");

  using namespace std::chrono_literals;
  ros_gz_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 10ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "gz_string_subscriber");

  return RUN_ALL_TESTS();
}
