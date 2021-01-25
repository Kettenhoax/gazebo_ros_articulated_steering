# Articulated steering control plugin for Gazebo

Controls a wheeled vehicle with articulation steering and all wheel drive based on `ackermann_msgs/msg/AckermannDriveStamped`.

## Installation

* clone to your ros2 workspace
* rosdep install --from-path src -i
* colcon build
* source install/setup.bash

Make sure the package environment is sourced, so the library is on `LD_LIBRARY_PATH` when spawning a robot into a Gazebo world.

## Parameters

The following parameters can be passed as children on the sdf plugin element.

* `command_timeout`: seconds to wait until invalidating the last command and setting motor target velocity to zero, latency will be added implicitly
* `update_rate`: update rate in Hz
* `publish_pid`: enable publishing of PID controller values as ROS topic
* `{joint_name}_pid_gain`: gains in order PID for respective joint motor
* `{joint_name}_i_range`: integral error bounds for respective joint motor
* `{joint_name}`: change the plugins search name for logical joint

Parameters on the geometry of the vehicle are derived from the links and joints on the model.

### Subscriptions

* `cmd_drive`: drive commands of type `ackermann_msgs/msg/AckermannDriveStamped`

### Publishers

* `odom_drive`: drive odometry of type `ackermann_msgs/msg/AckermannDriveStamped`
* `pid/{joint_name}`: PID state of type `control_msgs/msg/PidState`, enabled if `publish_pid` is true

### Example

```xml
<model>
    ...
    <plugin name="gazebo_ros_articulated_steering" filename="libgazebo_ros_articulated_steering.so">
        <ros>
          <namespace>vehicle</namespace>
        </ros>

        <robot_base_frame>base_footprint</robot_base_frame>
        <update_rate>100.0</update_rate>

        <front_right_motor>front_right_motor</front_right_motor>
        <front_left_motor>front_left_motor</front_left_motor>
        <rear_right_motor>rear_right_motor</rear_right_motor>
        <rear_left_motor>rear_left_motor</rear_left_motor>

        <articulation_joint>articulation_joint</articulation_joint>

        <front_right_motor_pid_gain>1 0 0</front_right_motor_pid_gain>
        <front_left_motor_pid_gain>1 0 0</front_left_motor_pid_gain>
        <rear_right_motor_pid_gain>1 0 0</rear_right_motor_pid_gain>
        <rear_left_motor_pid_gain>1 0 0</rear_left_motor_pid_gain>

        <articulation_pid_gain>1 0 0</articulation_pid_gain>
      </plugin>
      ...
</model>
```
