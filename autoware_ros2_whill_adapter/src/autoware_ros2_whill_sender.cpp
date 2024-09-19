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

#include "autoware_ros2_whill_sender.hpp"

AutowareRos2WhillSender::AutowareRos2WhillSender(const rclcpp::NodeOptions & node_options)
: Node("autoware_ros2_whill_sender", node_options)
{
  using std::placeholders::_1;

  // Parameters
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  wheel_base_ = vehicle_info.wheel_base_m;
  wheel_tread_ = vehicle_info.wheel_tread_m;
  wheel_radius_ = vehicle_info.wheel_radius_m;

  control_cmd_timeout_sec_ = declare_parameter<double>("control_cmd_timeout_sec", 0.5);
  loop_rate_ = declare_parameter<double>("loop_rate", 50.0);

  // Subscribe from Autoware
  control_cmd_sub_ =
    this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "/control/command/control_cmd", 1,
      std::bind(&AutowareRos2WhillSender::onAckermannControlCmd, this, _1));
  gear_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 1, std::bind(&AutowareRos2WhillSender::onGearCmd, this, _1));
  emergency_sub_ = create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
    "/control/command/emergency_cmd", 1, std::bind(&AutowareRos2WhillSender::onEmergencyCmd, this, _1));

  // Publish to Autoware
  gear_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", 1);
  // Publish to ros2_whill
  whill_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/whill/controller/cmd_vel", 1);

  // Service Client of ros2_whill
  whill_speed_profile_client_ = this->create_client<whill_msgs::srv::SetSpeedProfile>("/whill/set_speed_profile_srv");
  // TODO: speed profile initialization

  // Timer
  const auto period_ns = rclcpp::Rate(loop_rate_).period();
  cmd_timer_ = rclcpp::create_timer(
    this, this->get_clock(), period_ns, std::bind(&AutowareRos2WhillSender::onTimer, this));
}

void AutowareRos2WhillSender::onTimer()
{
  if (!gear_cmd_ptr_ || !control_cmd_ptr_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, "Command from Autoware is not ready.");
    return;
  }

  using autoware_auto_vehicle_msgs::msg::GearCommand;

  geometry_msgs::msg::Twist cmd_twist;

  if (!isCmdTimeout() && gear_cmd_ptr_->command != GearCommand::PARK && !is_emergency_)
  {
    cmd_twist.linear.x = control_cmd_ptr_->longitudinal.speed;
    cmd_twist.angular.z = control_cmd_ptr_->longitudinal.speed * std::tan(control_cmd_ptr_->lateral.steering_tire_angle) / wheel_base_;
    RCLCPP_INFO(this->get_logger(), "Publish Command: linear:['%f'] angular:['%f']", cmd_twist.linear.x, cmd_twist.angular.z);
  }
  else if (is_emergency_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000, "Autoware is Emergency.");
  }
  else if (gear_cmd_ptr_->command == GearCommand::PARK)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000, "Gear is Parking.");
  }
  else
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000, "Control command timed out.");
  }

  whill_twist_pub_->publish(cmd_twist);
}

bool AutowareRos2WhillSender::isCmdTimeout()
{
  if (!control_cmd_ptr_) {
    return true;
  }

  rclcpp::Time cmd_stamp = control_cmd_ptr_->stamp;
  double time_diff = this->now().seconds() - cmd_stamp.seconds();

  return time_diff > control_cmd_timeout_sec_ ? true : false;
}

void AutowareRos2WhillSender::onAckermannControlCmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  control_cmd_ptr_ = msg;
}

void AutowareRos2WhillSender::onGearCmd(const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
  using autoware_auto_vehicle_msgs::msg::GearCommand;
  using autoware_auto_vehicle_msgs::msg::GearReport;

  gear_cmd_ptr_ = msg;
  auto report_msg = GearReport{};
  report_msg.stamp = msg->stamp;

  if (msg->command == GearCommand::PARK) {
    report_msg.report = GearReport::PARK;
  }
  if (msg->command == GearCommand::REVERSE) {
    report_msg.report = GearReport::REVERSE;
  }
  if (msg->command == GearCommand::DRIVE) {
    report_msg.report = GearReport::DRIVE;
  }
  if (msg->command == GearCommand::LOW) {
    report_msg.report = GearReport::LOW;
  }
  gear_status_pub_->publish(report_msg);
}

void AutowareRos2WhillSender::onEmergencyCmd(const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg)
{
  is_emergency_ = msg->emergency;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(AutowareRos2WhillSender)
