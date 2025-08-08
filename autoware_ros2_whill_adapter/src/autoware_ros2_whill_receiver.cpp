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

#include "autoware_ros2_whill_receiver.hpp"

AutowareRos2WhillReceiver::AutowareRos2WhillReceiver(const rclcpp::NodeOptions & node_options)
: Node("autoware_ros2_whill_receiver", node_options)
{
  using std::placeholders::_1;

  // Parameters
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  wheel_base_ = vehicle_info.wheel_base_m;
  wheel_tread_ = vehicle_info.wheel_tread_m;
  wheel_radius_ = vehicle_info.wheel_radius_m;

  // Subscribe from ros2_whill
  whill_state_sub_ = this->create_subscription<whill_msgs::msg::ModelCr2State>(
    "/whill/states/model_cr2", 1, std::bind(&AutowareRos2WhillReceiver::onWhillStates, this, _1));

  // Publish to autoware
  velocity_status_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", 1);
  steering_status_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", 1);
  turn_indicators_status_pub_ =
    this->create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
      "/vehicle/status/turn_indicators_status", 1);
  hazard_lights_status_pub_ =
    this->create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>(
      "/vehicle/status/hazard_lights_status", 1);
  control_mode_status_pub_ =
    this->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
      "/vehicle/status/control_mode", 1);
  velocity_kmph_status_pub_ = this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "/vehicle/status/velocity_kmph", 1);
  steering_wheel_deg_status_pub_ = this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "/vehicle/status/steering_wheel_deg", 1);

  setupDiagnosticUpdater();
}

void AutowareRos2WhillReceiver::onWhillStates(const whill_msgs::msg::ModelCr2State::ConstSharedPtr msg)
{
  const auto now_stamp = this->now();

  // Right motor outputs negative value when the vehicle moving forward.
  tier4_debug_msgs::msg::Float32Stamped velocity_kmph_msg;
  velocity_kmph_msg.data = (msg->left_motor_speed - msg->right_motor_speed) / 2;
  velocity_kmph_msg.stamp = now_stamp;
  velocity_kmph_status_pub_->publish(velocity_kmph_msg);

  autoware_vehicle_msgs::msg::VelocityReport twist;
  twist.header.stamp = now_stamp;
  twist.header.frame_id = "base_link";
  twist.longitudinal_velocity = velocity_kmph_msg.data / 3.6;
  twist.heading_rate = (msg->right_motor_speed + msg->left_motor_speed) / -3.6 / wheel_tread_;
  velocity_status_pub_->publish(twist);

  autoware_vehicle_msgs::msg::SteeringReport steer_msg;
  steer_msg.stamp = now_stamp;
  steer_msg.steering_tire_angle =
    twist.longitudinal_velocity != 0.0
      ? std::atan(twist.heading_rate * wheel_base_ / twist.longitudinal_velocity)
      : 0.0;
  steering_status_pub_->publish(steer_msg);

  tier4_debug_msgs::msg::Float32Stamped steer_wheel_deg_msg;
  steer_wheel_deg_msg.data = steer_msg.steering_tire_angle * 180.0 / M_PI;
  steer_wheel_deg_msg.stamp = now_stamp;
  steering_wheel_deg_status_pub_->publish(steer_wheel_deg_msg);

  publishStaticTopics();
  err_code_ = msg->error;
  diagnostic_updater_.force_update();
}

void AutowareRos2WhillReceiver::publishStaticTopics()
{
  const auto now_stamp = this->now();

  // WIP
  using autoware_vehicle_msgs::msg::ControlModeReport;
  ControlModeReport control_mode_report;
  control_mode_report.stamp = now_stamp;
  control_mode_report.mode = ControlModeReport::AUTONOMOUS;
  control_mode_status_pub_->publish(control_mode_report);

  using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
  TurnIndicatorsReport turn_indicator_report;
  turn_indicator_report.stamp = now_stamp;
  turn_indicator_report.report = TurnIndicatorsReport::DISABLE;
  turn_indicators_status_pub_->publish(turn_indicator_report);

  using autoware_vehicle_msgs::msg::HazardLightsReport;
  HazardLightsReport hazard_light_report;
  hazard_light_report.stamp = now_stamp;
  hazard_light_report.report =HazardLightsReport::DISABLE;
  hazard_lights_status_pub_->publish(hazard_light_report);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(AutowareRos2WhillReceiver)
