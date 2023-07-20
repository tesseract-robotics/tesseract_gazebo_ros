/**
 * @author Levi Armstrong <levi.armstrong@gmail.com>
 *
 * @copyright Copyright (C) 2023 Levi Armstrong <levi.armstrong@gmail.com>
 *
 * @par License
 * GNU Lesser General Public License Version 3, 29 June 2007
 * @par
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 * @par
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * @par
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
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

  bool loadSensors();

  void tesseractEventFilter(const tesseract_environment::Event& event);
};
}
#endif // TESSERACT_SENSOR_SIMULATOR_SENSOR_SIMULATOR_H
