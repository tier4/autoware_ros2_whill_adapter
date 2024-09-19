// Copyright 2024 Tier IV, Inc.
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

#ifndef AUTOWARE_ROS2_WHILL_SENDER__AUTOWARE_ROS2_WHILL_SENDER_HPP_
#define AUTOWARE_ROS2_WHILL_SENDER__AUTOWARE_ROS2_WHILL_SENDER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"

#include "tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp"

#include "vehicle_info_util/vehicle_info_util.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "whill_msgs/srv/set_speed_profile.hpp"

class AutowareRos2WhillSender : public rclcpp::Node
{
public:
  AutowareRos2WhillSender(const rclcpp::NodeOptions & node_options);

private:
  // Parameters
  double wheel_base_;
  double wheel_tread_;
  double wheel_radius_;

  // Variables
  bool is_emergency_{false};
  double control_cmd_timeout_sec_;
  autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr control_cmd_ptr_;
  autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr gear_cmd_ptr_;

  rclcpp::TimerBase::SharedPtr cmd_timer_;
  double loop_rate_;

  // Functions
  void onTimer();
  bool isCmdTimeout();

  // Callbacks
  void onAckermannControlCmd(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
  void onGearCmd(const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
  void onEmergencyCmd(const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg);

  // Subscrib from Autoware
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    control_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr emergency_sub_;

  // Publish to Autoware
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;
  // Publish to ros2_whill
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr whill_twist_pub_;

  // Service Client of ros2_whill
  rclcpp::Client<whill_msgs::srv::SetSpeedProfile>::SharedPtr whill_speed_profile_client_;
};

#endif  // AUTOWARE_ROS2_WHILL_SENDER__AUTOWARE_ROS2_WHILL_SENDER_HPP_
