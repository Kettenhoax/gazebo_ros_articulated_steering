// Copyright 2020 AIT Austrian Institute of Technology GmbH
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

#pragma once

#include <gazebo/common/Plugin.hh>
#include <memory>

namespace gazebo_plugins
{
class GazeboRosArticulatedSteeringPrivate;

enum JointIdentifier
{
  /// Front right traction motor
  FRONT_RIGHT_MOTOR,

  /// Front left traction motor
  FRONT_LEFT_MOTOR,

  /// Rear right traction motor
  REAR_RIGHT_MOTOR,

  /// Rear left traction motor
  REAR_LEFT_MOTOR,

  /// Articulation steering joint
  ARTICULATION_JOINT,
};

/// A control plugin for robots with articulated steering and all wheel drive
/// Subscribes to ackermann_msgs/msg/AckermannDriveStamped

/**
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_articulated_steering" filename="libgazebo_ros_articulated_steering.so">

      <ros>
        <namespace>demo</namespace>
        <remapping>cmd_drive:=cmd_demo</remapping>
      </ros>

      <update_rate>100.0</update_rate>
      ...
      <robot_base_frame>base_footprint</robot_base_frame>
    </plugin>
  \endcode
*/
class GazeboRosArticulatedSteering : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosArticulatedSteering();

  /// Destructor
  ~GazeboRosArticulatedSteering();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  // Documentation inherited
  void Reset() override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosArticulatedSteeringPrivate> impl_;
};
}  // namespace gazebo_plugins
