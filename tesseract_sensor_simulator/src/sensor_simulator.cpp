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

#include <tesseract_sensor_simulator/sensor_simulator.h>

#include <gz/rendering/Scene.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/sensors/CameraSensor.hh>
#include <gz/sensors/DepthCameraSensor.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/Manager.hh>
#include <gz/math/eigen3/Conversions.hh>

namespace tesseract_sensor_simulator
{

SensorSimulator::SensorSimulator(tesseract_environment::EnvironmentMonitor::Ptr env_monitor,
                                 SensorSimulatorProperties scene_properties)
  : env_monitor_(std::move(env_monitor))
  , scene_properties_(std::move(scene_properties))
  , entity_manager_(std::make_shared<tesseract_gui::EntityManager>())
  , entity_container_(std::make_shared<tesseract_gui::EntityContainer>(entity_manager_, scene_properties_.scene_name))
{
}

void SensorSimulator::update()
{
  std::scoped_lock render_lock(mutex_);
  if (scene_ == nullptr && env_monitor_->environment().isInitialized())
  {
    auto* engine = gz::rendering::engine(scene_properties_.engine_name);
    if (engine == nullptr)
    {
      CONSOLE_BRIDGE_logError("Internal error: failed to load engine [%s]", scene_properties_.engine_name.c_str());
      return;
    }
    scene_ = engine->SceneByName(scene_properties_.scene_name);
    assert(scene_ == nullptr);

    // Create scene
    scene_ = engine->CreateScene(scene_properties_.scene_name);
    scene_->SetAmbientLight(scene_properties_.ambient_light);
    scene_->SetBackgroundColor(scene_properties_.background_color);
    scene_->SetSkyEnabled(scene_properties_.sky_enable);

    // Get root node
    gz::rendering::VisualPtr root = scene_->RootVisual();

    // create directional light
    gz::rendering::DirectionalLightPtr light0 = scene_->CreateDirectionalLight();
    light0->SetDirection(1, 0, 0);
    light0->SetDiffuseColor(0.8, 0.8, 0.8);
    light0->SetSpecularColor(0.5, 0.5, 0.5);
    Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
    pose.translation() = Eigen::Vector3d(-1.5, 0, 1);
    light0->SetLocalPose(gz::math::eigen3::convert(pose));
    root->AddChild(light0);

    // Load scene
    {
      auto lock = env_monitor_->environment().lockRead();
      tesseract_gui::loadSceneGraph(*scene_, *entity_container_, *env_monitor_->environment().getSceneGraph());

      // Set the initial state
      tesseract_scene_graph::SceneState scene_state = env_monitor_->environment().getState();
      tesseract_gui::setSceneState(*scene_, *entity_container_, scene_state.link_transforms);

      // Set the tesseract event callback
      current_revision_ = env_monitor_->environment().getRevision();
    }
    // Add event callback
    std::size_t uuid = std::hash<SensorSimulator*>()(this);
    env_monitor_->environment().addEventCallback(uuid,[this](const tesseract_environment::Event& event){ tesseractEventFilter(event); });

    // load sensors
    loadSensors();
    start_time_ = std::chrono::steady_clock::now();
    sensor_manager_.RunOnce(std::chrono::steady_clock::now().time_since_epoch(), true);
  }

  sensor_manager_.RunOnce(std::chrono::steady_clock::now().time_since_epoch());
}

bool SensorSimulator::loadSensors()
{
  // Create SDF describing a camera sensor

  {
    const double hz = 30;
    const std::size_t width = 1920;
    const std::size_t height = 1080;
    const std::size_t hfov = GZ_DTOR(60);
    const double near = 0.1;
    const double far = 100;
    const auto format = sdf::PixelFormatType::RGB_INT8;
    const std::string name = "ExampleCamera";
    const std::string topic = "/gz/sensors/examples/camera_sensor";
    const std::string info_topic = "/gz/sensors/examples/camera_sensor/camera_info";
    sdf::Camera cameraSdf;
    cameraSdf.SetImageWidth(width);
    cameraSdf.SetImageHeight(height);
    cameraSdf.SetHorizontalFov(hfov);
    cameraSdf.SetNearClip(near);
    cameraSdf.SetFarClip(far);
    cameraSdf.SetPixelFormat(format);
    cameraSdf.SetOpticalFrameId("tool0");
    cameraSdf.SetCameraInfoTopic(info_topic);

    sdf::Sensor sensorSdf;
    sensorSdf.SetType(sdf::SensorType::CAMERA);
    sensorSdf.SetName(name);
    sensorSdf.SetTopic(topic);
    sensorSdf.SetUpdateRate(hz);
    sensorSdf.SetCameraSensor(cameraSdf);

    // Create a CameraSensor
    auto* cameraSensor = sensor_manager_.CreateSensor<gz::sensors::CameraSensor>(sensorSdf);

    if (cameraSensor == nullptr)
    {
      gzerr << "Unable to load camera sensor\n";
      return false;
    }
    cameraSensor->SetScene(scene_);
    Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
    pose.translation() = Eigen::Vector3d(-1.5, 0, 1);
    cameraSensor->SetPose(gz::math::eigen3::convert(pose));
  }

  {
    const double hz = 30;
    const std::size_t width = 640;
    const std::size_t height = 480;
    const std::size_t hfov = GZ_DTOR(60);
    const double near = 0.1;
    const double far = 30;
    const auto format = sdf::PixelFormatType::RGB_INT8;
    const std::string name = "ExampleDepthCamera";
    const std::string topic = "/gz/sensors/examples/depth_sensor";
    const std::string info_topic = "/gz/sensors/examples/depth_sensor/camera_info";

    sdf::Camera cameraSdf;
    cameraSdf.SetImageWidth(width);
    cameraSdf.SetImageHeight(height);
    cameraSdf.SetHorizontalFov(hfov);
    cameraSdf.SetNearClip(near);
    cameraSdf.SetFarClip(far);
    cameraSdf.SetPixelFormat(format);
    cameraSdf.SetOpticalFrameId("tool0");
    cameraSdf.SetCameraInfoTopic(info_topic);

    sdf::Sensor sensorSdf;
    sensorSdf.SetType(sdf::SensorType::DEPTH_CAMERA);
    sensorSdf.SetName(name);
    sensorSdf.SetTopic(topic);
    sensorSdf.SetUpdateRate(hz);
    sensorSdf.SetCameraSensor(cameraSdf);

    // Create a CameraSensor
    auto* cameraSensor = sensor_manager_.CreateSensor<gz::sensors::DepthCameraSensor>(sensorSdf);

    if (cameraSensor == nullptr)
    {
      gzerr << "Unable to load camera sensor\n";
      return false;
    }
    cameraSensor->SetScene(scene_);
    Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
    pose.translation() = Eigen::Vector3d(-1.5, 0, 1);
    cameraSensor->SetPose(gz::math::eigen3::convert(pose));
  }

  return true;
}

void SensorSimulator::tesseractEventFilter(const tesseract_environment::Event& event)
{
  if (!env_monitor_->environment().isInitialized())
    return;

  switch (event.type)
  {
    case tesseract_environment::Events::COMMAND_APPLIED:
    {
      bool reset{ false };
      const auto& e = static_cast<const tesseract_environment::CommandAppliedEvent&>(event);
      if (current_revision_ == 0 || e.revision < current_revision_)
      {
        reset = true;
      }
      else
      {
        /**
         * @todo update to handle explicit commands
         * @note See ign_scene_graph_render_manager.cpp
         */
        for (std::size_t i = current_revision_; i < e.revision; ++i)
        {
          const auto& cmd = e.commands.at(i);
          switch (cmd->getType())
          {
            case tesseract_environment::CommandType::ADD_SCENE_GRAPH:
            case tesseract_environment::CommandType::ADD_LINK:
            case tesseract_environment::CommandType::ADD_TRAJECTORY_LINK:
            case tesseract_environment::CommandType::CHANGE_LINK_VISIBILITY:
            case tesseract_environment::CommandType::REMOVE_LINK:
            case tesseract_environment::CommandType::REMOVE_JOINT:
            {
              reset = true;
              break;
            }
            case tesseract_environment::CommandType::MOVE_LINK:
            case tesseract_environment::CommandType::MOVE_JOINT:
            case tesseract_environment::CommandType::REPLACE_JOINT:
            case tesseract_environment::CommandType::CHANGE_JOINT_ORIGIN:
            case tesseract_environment::CommandType::CHANGE_LINK_ORIGIN:
            case tesseract_environment::CommandType::CHANGE_LINK_COLLISION_ENABLED:
            case tesseract_environment::CommandType::MODIFY_ALLOWED_COLLISIONS:
            case tesseract_environment::CommandType::REMOVE_ALLOWED_COLLISION_LINK:
            case tesseract_environment::CommandType::CHANGE_JOINT_POSITION_LIMITS:
            case tesseract_environment::CommandType::CHANGE_JOINT_VELOCITY_LIMITS:
            case tesseract_environment::CommandType::CHANGE_JOINT_ACCELERATION_LIMITS:
            case tesseract_environment::CommandType::ADD_KINEMATICS_INFORMATION:
            case tesseract_environment::CommandType::CHANGE_COLLISION_MARGINS:
            case tesseract_environment::CommandType::ADD_CONTACT_MANAGERS_PLUGIN_INFO:
            case tesseract_environment::CommandType::SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER:
            case tesseract_environment::CommandType::SET_ACTIVE_DISCRETE_CONTACT_MANAGER:
            {
              break;
            }
            // LCOV_EXCL_START
            default:
            {
              CONSOLE_BRIDGE_logError("Tesseract Qt Gazebo Utils, Unhandled environment command");
            }
          }
        }
      }
      if (reset)
      {
        std::scoped_lock render_lock(mutex_);
        // Clear Scene
        clearScene(*scene_, *entity_container_);
        entity_container_->clear();

        auto lock = env_monitor_->environment().lockRead();
        loadSceneGraph(*scene_, *entity_container_, *(env_monitor_->environment().getSceneGraph()), prefix_);
      }
      current_revision_ = e.revision;
      break;
    }
    case tesseract_environment::Events::SCENE_STATE_CHANGED:
    {
      std::scoped_lock lock(mutex_);
      const auto& e = static_cast<const tesseract_environment::SceneStateChangedEvent&>(event);
      setSceneState(*scene_, *entity_container_, e.state.link_transforms);
      break;
    }
  }
}

}
