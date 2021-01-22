// Copyright 2021 AIT Austrian Institute of Technology GmbH
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

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <control_msgs/msg/pid_state.hpp>

#include <gazebo/common/Time.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/Noise.hh>
#include <sdf/sdf.hh>

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <utility>
#include <limits>

#include "gazebo_ros_articulated_steering/plugin.hpp"

namespace gazebo_plugins
{
using ignition::math::Vector3d;
using ignition::math::Vector2d;
using gazebo::physics::JointController;
using ackermann_msgs::msg::AckermannDriveStamped;

struct ControlledJoint
{
  gazebo::physics::JointPtr joint;
  std::string scoped_name;
  std::unique_ptr<gazebo::common::PID> pid;
  std::unique_ptr<JointController> controller;
  rclcpp::Publisher<control_msgs::msg::PidState>::SharedPtr pid_publisher;
};

enum JointIdentifier
{
  /// Front right traction motor
  FRONT_RIGHT_MOTOR = 0,

  /// Front left traction motor
  FRONT_LEFT_MOTOR,

  /// Rear right traction motor
  REAR_RIGHT_MOTOR ,

  /// Rear left traction motor
  REAR_LEFT_MOTOR,

  /// Articulation steering joint
  ARTICULATION_JOINT,
};

class GazeboRosArticulatedSteeringPrivate
{
public:
  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// Callback when a drive command is received.
  /// \param[in] _msg AckermannDriveStamped command message.
  void OnCmd(AckermannDriveStamped::SharedPtr _msg);

  /// Extracts radius of a cylinder or sphere collision shape
  /// \param[in] _coll Pointer to collision
  /// \return If the collision shape is valid, return radius
  /// \return If the collision shape is invalid, return 0
  bool CollisionRadius(const gazebo::physics::CollisionPtr & _coll, double * radius);

  /// Infers the wheel radius from the links attached to the wheel joints
  /// \param[out] _radius Output radius in meters
  /// \return whether the radius can be inferred
  bool InferWheelRadius(double * _radius);

  /// Update joint controller targets
  /// \param[in] dt Delta time in seconds
  void UpdateControllers(double);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to command velocities
  rclcpp::Subscription<AckermannDriveStamped>::SharedPtr cmd_sub_;

  /// Publisher of drive odometry
  // rclcpp::Publisher<AckermannDriveStamped>::SharedPtr odom_pub_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_cmd_;

  /// Command in order of reception
  std::deque<AckermannDriveStamped> commands_;

  /// Last received drive command
  AckermannDriveStamped last_cmd_;

  /// Wheel radius in m
  double wheel_radius_;

  /// Noise applied to speed odometry
  // gazebo::sensors::NoisePtr speed_sensing_noise_;

  /// Noise applied to articulation angle odometry
  // gazebo::sensors::NoisePtr articulation_angle_sensing_noise_;

  /// Update period in seconds.
  double update_period_;

  /// Command timeout in seconds
  double command_timeout_;

  /// Last update time.
  gazebo::common::Time last_update_time_;

  /// Robot base frame ID
  std::string state_frame_id_;

  /// Joints and controllers
  std::vector<ControlledJoint> joints_;
};

GazeboRosArticulatedSteering::GazeboRosArticulatedSteering()
: impl_(std::make_unique<GazeboRosArticulatedSteeringPrivate>())
{
}

GazeboRosArticulatedSteering::~GazeboRosArticulatedSteering()
{
}

static bool IsVelocityJoint(JointIdentifier i)
{
  return i <= REAR_LEFT_MOTOR;
}

static double GetVelocityJointDefaultGain(gazebo::physics::JointPtr joint)
{
  return joint->GetVelocityLimit(0) * joint->GetEffortLimit(0);
}

static double GetPositionJointDefaultGain(gazebo::physics::JointPtr joint)
{
  return 1.0;
  // return joint->UpperLimit(0) * joint->GetEffortLimit(0);
}

void GazeboRosArticulatedSteering::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
  impl_->joints_.resize(5);

  std::map<JointIdentifier, std::string> joint_names =
  {{FRONT_RIGHT_MOTOR, "front_right_motor"},
    {FRONT_LEFT_MOTOR, "front_left_motor"},
    {REAR_RIGHT_MOTOR, "rear_right_motor"},
    {REAR_LEFT_MOTOR, "rear_left_motor"},
    {ARTICULATION_JOINT, "articulation_joint"},
  };

  for (const auto & joint_name : joint_names) {
    auto id = joint_name.first;
    auto name = joint_name.second;
    // get remapped name from sdf parameters, or use default name
    auto remapped_name = _sdf->Get<std::string>(name, name).first;
    auto joint = _model->GetJoint(remapped_name);
    if (!joint) {
      RCLCPP_ERROR(
        impl_->ros_node_->get_logger(),
        "Joint [%s] not found, plugin cannot be initialized.", name.c_str());
      impl_->ros_node_.reset();
      return;
    }
    impl_->joints_[id].joint = joint;
    impl_->joints_[id].scoped_name = impl_->joints_[id].joint->GetScopedName();
  }

  size_t i = 0;
  for (auto & j : impl_->joints_) {
    JointIdentifier id = (JointIdentifier)i;
    auto default_gain =
      IsVelocityJoint(id) ? GetVelocityJointDefaultGain(j.joint) : GetPositionJointDefaultGain(
      j.joint);
    if (ignition::math::isnan(default_gain)) {
      default_gain = 1.0;
    }
    auto gain = _sdf->Get(j.scoped_name + "_pid_gain", Vector3d(default_gain, 0.0, 0.0)).first;
    auto i_range =
      _sdf->Get(j.scoped_name + "_i_range", Vector2d(-default_gain, default_gain)).first;

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "Gains [p %.2f, i %.2f, d %.2f] and i_range [%.2f,%.2f] on %s", gain.X(),
      gain.Y(), gain.Z(), i_range.X(), i_range.Y(), j.scoped_name.c_str());

    j.pid = std::make_unique<gazebo::common::PID>();
    j.pid->Init(gain.X(), gain.Y(), gain.Z(), i_range.Y(), i_range.X());

    j.controller = std::make_unique<JointController>(_model);
    j.controller->AddJoint(j.joint);

    if (IsVelocityJoint(id)) {
      j.controller->SetVelocityPID(j.scoped_name, *j.pid);
    } else {
      j.controller->SetPositionPID(j.scoped_name, *j.pid);
    }
    i++;
  }

  impl_->InferWheelRadius(&impl_->wheel_radius_);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Wheel radius %.2f", impl_->wheel_radius_);

  auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }

  impl_->command_timeout_ = _sdf->Get<double>("command_timeout", impl_->update_period_ * 2).first;
  impl_->last_update_time_ = _model->GetWorld()->SimTime();

  impl_->last_cmd_ = AckermannDriveStamped();
  impl_->cmd_sub_ =
    impl_->ros_node_->create_subscription<AckermannDriveStamped>(
    "cmd_drive", rclcpp::QoS(1),
    std::bind(&GazeboRosArticulatedSteeringPrivate::OnCmd, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Subscribed to [%s]", impl_->cmd_sub_->get_topic_name());

  if (_sdf->Get("publish_pid", true).first) {
    for (const auto & joint_name : joint_names) {
      auto name = joint_name.second;
      auto pub =
        impl_->ros_node_->create_publisher<control_msgs::msg::PidState>(
        "pid/" + name,
        rclcpp::QoS(1));

      RCLCPP_INFO(
        impl_->ros_node_->get_logger(),
        "Publishing [%s]", pub->get_topic_name());
      impl_->joints_[joint_name.first].pid_publisher = pub;
    }
  }

  impl_->state_frame_id_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosArticulatedSteeringPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosArticulatedSteering::Reset()
{
  impl_->last_update_time_ = impl_->model_->GetWorld()->SimTime();
  impl_->last_cmd_ = AckermannDriveStamped();
  impl_->commands_.clear();
}

void GazeboRosArticulatedSteeringPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  std::lock_guard<std::mutex> lock(lock_cmd_);
  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

  if (seconds_since_last_update >= update_period_) {
    UpdateControllers(seconds_since_last_update);
    last_update_time_ = _info.simTime;
    for (auto & joint : joints_) {
      joint.controller->Update();
    }
  }
}

void GazeboRosArticulatedSteeringPrivate::UpdateControllers(double dt)
{
  auto ros_time = ros_node_->now();

  auto latency_ = 0.0;
  while (!commands_.empty() && (ros_time - commands_.front().header.stamp).seconds() >= latency_) {
    last_cmd_ = commands_.front();
    commands_.pop_front();
  }
  if ((ros_time - last_cmd_.header.stamp).seconds() >= (latency_ + command_timeout_)) {
    // if the latest command is too old, set speed to zero
    last_cmd_.drive.speed = 0;
  }

  auto motor_speed = last_cmd_.drive.speed / wheel_radius_;
  if (ignition::math::isnan(motor_speed)) {
    RCLCPP_WARN(ros_node_->get_logger(), "NaN motor speed command");
    motor_speed = 0.0;
  }

  double errors[5];

  // set wheel speed efforts
  for (auto wheel_i : {FRONT_RIGHT_MOTOR, FRONT_LEFT_MOTOR, REAR_RIGHT_MOTOR, REAR_LEFT_MOTOR}) {
    // get wheel speed in rad/s
    auto name = joints_[wheel_i].scoped_name;
    auto joint_velocity = joints_[wheel_i].joint->GetVelocity(0);
    errors[wheel_i] = joint_velocity - motor_speed;

    if (!joints_[wheel_i].controller->SetVelocityTarget(name, motor_speed)) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Joint [%s] was not found", name.c_str());
    }
  }

  auto articulation_angle = last_cmd_.drive.steering_angle;
  if (ignition::math::isnan(articulation_angle)) {
    RCLCPP_WARN(ros_node_->get_logger(), "NaN target articulation angle");
    articulation_angle = 0.0;
  }

  // set articulation effort
  auto & articulation_joint = joints_[ARTICULATION_JOINT];
  auto name = articulation_joint.scoped_name;
  auto joint = articulation_joint.joint;
  auto current_angle = joint->Position(0);
  if (ignition::math::isnan(current_angle)) {
    RCLCPP_WARN(ros_node_->get_logger(), "NaN current articulation angle");
    current_angle = 0.0;
  }
  errors[ARTICULATION_JOINT] = current_angle - articulation_angle;

  if (!articulation_joint.controller->SetPositionTarget(name, articulation_angle)) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Joint [%s] was not found", name.c_str());
  }

  size_t i = 0;
  for (auto & joint : joints_) {
    auto pub = joint.pid_publisher;
    if (!pub) {
      break;
    }
    control_msgs::msg::PidState state;
    state.header.frame_id = state_frame_id_;
    state.header.stamp = ros_time;
    state.timestep = rclcpp::Duration::from_seconds(dt);
    state.error = errors[i];
    state.error_dot = NAN;
    joint.pid->GetErrors(state.p_error, state.i_error, state.d_error);
    state.p_term = joint.pid->GetPGain();
    state.i_term = joint.pid->GetIGain();
    state.d_term = joint.pid->GetDGain();
    state.i_max = joint.pid->GetIMax();
    state.i_min = joint.pid->GetIMin();
    state.output = joint.pid->GetCmd();
    joint.pid_publisher->publish(state);
    i++;
  }
}

void GazeboRosArticulatedSteeringPrivate::OnCmd(AckermannDriveStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(lock_cmd_);
  commands_.push_back(*msg);
}

bool GazeboRosArticulatedSteeringPrivate::CollisionRadius(
  const gazebo::physics::CollisionPtr & _coll, double * radius)
{
  if (!_coll || !(_coll->GetShape())) {
    return false;
  }
  if (_coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE)) {
    gazebo::physics::CylinderShape * cyl =
      dynamic_cast<gazebo::physics::CylinderShape *>(_coll->GetShape().get());
    *radius = cyl->GetRadius();
    return true;
  } else if (_coll->GetShape()->HasType(gazebo::physics::Base::SPHERE_SHAPE)) {
    gazebo::physics::SphereShape * sph =
      dynamic_cast<gazebo::physics::SphereShape *>(_coll->GetShape().get());
    *radius = sph->GetRadius();
    return true;
  }
  return false;
}

bool GazeboRosArticulatedSteeringPrivate::InferWheelRadius(
  double * radius)
{
  // Update wheel radius for wheel from SDF collision objects
  // assumes that wheel link is child of joint (and not parent of joint)
  // assumes that wheel link has only one collision
  // assumes all wheel of both rear wheels of same radii
  double radii[4];

  for (size_t i = 0; i < 4; i++) {
    const auto wheel = joints_[FRONT_RIGHT_MOTOR + i].joint->GetChild();
    if (wheel->GetCollisions().size() != 1) {
      return false;
    }
    const auto collision_object = wheel->GetCollisions().at(0);
    double radius;
    if (!CollisionRadius(collision_object, &radius)) {
      return false;
    }
    radii[i] = radius;
  }
  for (size_t i = 1; i < 4; i++) {
    if (fabs(radii[i] - radii[0]) > radii[0] * std::numeric_limits<double>::epsilon()) {
      return false;
    }
  }
  *radius = radii[0];
  return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosArticulatedSteering)
}  // namespace gazebo_plugins
