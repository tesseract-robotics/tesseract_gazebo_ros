/**
 * @file sensor_simulator.h
 * @brief Sensor simulator leveraging Gazebo Sensors
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
#ifndef TESSERACT_SENSOR_SIMULATOR_SENSOR_SIMULATOR_H
#define TESSERACT_SENSOR_SIMULATOR_SENSOR_SIMULATOR_H

#include <tesseract_environment/environment_monitor.h>

#include <tesseract_qt/rendering/gazebo_utils.h>
#include <tesseract_qt/common/entity_manager.h>
#include <tesseract_qt/common/entity_container.h>

#include <gz/rendering/Scene.hh>
#include <gz/sensors/Manager.hh>
#include <gz/math/Color.hh>
#include <gz/msgs/image.pb.h>
#include <gz/common/events/Types.hh>

namespace tesseract_sensor_simulator
{
struct SensorSimulatorProperties
{
  /** @brief Render engine to use */
  std::string engine_name{ "ogre2" };

  /** @brief Unique scene name */
  std::string scene_name{ "scene" };

  /** @brief Scene background color */
  gz::math::Color background_color{ 0.8f, 0.8f, 0.8f, 1.0f };

  /** @brief Ambient color */
  gz::math::Color ambient_light{ 0.4f, 0.4f, 0.4f, 1.0f };

  /** @brief True if sky is enabled */
  bool sky_enable{ false };
};

class SensorSimulator
{
public:
SensorSimulator(tesseract_environment::EnvironmentMonitor::Ptr env_monitor,
                SensorSimulatorProperties scene_properties);

/** @brief Call update on the sensors */
void update();

private:
  tesseract_environment::EnvironmentMonitor::Ptr env_monitor_;
  SensorSimulatorProperties scene_properties_;
  tesseract_gui::EntityManager::Ptr entity_manager_;
  tesseract_gui::EntityContainer::Ptr entity_container_;
  gz::rendering::ScenePtr scene_;
  gz::sensors::Manager sensor_manager_;
  std::chrono::steady_clock::time_point start_time_;
  int current_revision_{0};
  std::string prefix_;
  std::mutex mutex_;

  bool loadSensors();

  void tesseractEventFilter(const tesseract_environment::Event& event);
};
}
#endif // TESSERACT_SENSOR_SIMULATOR_SENSOR_SIMULATOR_H
