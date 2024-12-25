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

#ifndef AUTOWARE_ROS2_WHILL_RECEIVER__AUTOWARE_ROS2_WHILL_RECEIVER_HPP_
#define AUTOWARE_ROS2_WHILL_RECEIVER__AUTOWARE_ROS2_WHILL_RECEIVER_HPP_

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/headlights_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"

#include "tier4_debug_msgs/msg/float32_stamped.hpp"
#include "tier4_vehicle_msgs/msg/battery_status.hpp"
#include "tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp"

#include "vehicle_info_util/vehicle_info_util.hpp"

#include "whill_msgs/msg/model_cr2_state.hpp"

class AutowareRos2WhillReceiver : public rclcpp::Node
{
public:
  AutowareRos2WhillReceiver(const rclcpp::NodeOptions & node_options);

private:
  // Parameters
  double wheel_base_;
  double wheel_tread_;
  double wheel_radius_;

  // Functions
  void publishStaticTopics();

  // Diagnostics
  int err_code_;
  void setupDiagnosticUpdater();
  void checkErrCode(diagnostic_updater::DiagnosticStatusWrapper & stat);
  diagnostic_updater::Updater diagnostic_updater_{this};

  // Callbacks
  void onWhillStates(const whill_msgs::msg::ModelCr2State::ConstSharedPtr msg);

  // Subscribe from ros2_whill
  rclcpp::Subscription<whill_msgs::msg::ModelCr2State>::SharedPtr whill_state_sub_;

  // Publish to Autoware
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr
    velocity_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr
    steering_status_pub_;
//   rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;    // gearだけはsender側で処理が必要
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
    turn_indicators_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>::SharedPtr
    hazard_lights_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    control_mode_status_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr velocity_kmph_status_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr
    steering_wheel_deg_status_pub_;
};

#endif  // AUTOWARE_ROS2_WHILL_RECEIVER__AUTOWARE_ROS2_WHILL_RECEIVER_HPP_
