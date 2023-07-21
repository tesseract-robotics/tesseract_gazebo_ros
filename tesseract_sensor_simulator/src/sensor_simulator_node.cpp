/**
 * @file sensor_simulator_node.cpp
 * @brief Sensor simulator leveraging Gazebo Sensors Node
 *
 * @author Levi Armstrong
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2023, Levi Armstrong
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <ros/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_sensor_simulator/sensor_simulator.h>

using namespace tesseract_environment;
using namespace tesseract_sensor_simulator;
using namespace tesseract_monitoring;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tesseract_environment_monitor");
  ros::NodeHandle pnh("~");
  std::string robot_description;
  std::string joint_state_topic;
  std::string monitor_namespace;
  std::string monitored_namespace;
  bool publish_environment{ false };

  if (!pnh.getParam("monitor_namespace", monitor_namespace))
  {
    ROS_ERROR("Missing required parameter monitor_namespace!");
    return 1;
  }

  pnh.param<std::string>("monitored_namespace", monitored_namespace, "");
  pnh.param<std::string>("robot_description", robot_description, ROBOT_DESCRIPTION_PARAM);
  pnh.param<std::string>("joint_state_topic", joint_state_topic, "");
  pnh.param<bool>("publish_environment", publish_environment, publish_environment);

  auto monitor = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(robot_description, monitor_namespace);

  if (publish_environment)
    monitor->startPublishingEnvironment();

  if (!monitored_namespace.empty())
    monitor->startMonitoringEnvironment(monitored_namespace);

  bool publish_tf = monitored_namespace.empty();
  if (joint_state_topic.empty())
    monitor->startStateMonitor(DEFAULT_JOINT_STATES_TOPIC, publish_tf);
  else
    monitor->startStateMonitor(joint_state_topic, publish_tf);

  SensorSimulatorProperties properties;
  properties.engine_name = "ogre";
  properties.scene_name = "tesseract_sensor_simulator";
  auto sensor_simulator = std::make_shared<SensorSimulator>(monitor, properties);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    sensor_simulator->update();
    loop_rate.sleep();
  }

  return 0;
}
