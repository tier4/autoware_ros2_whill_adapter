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

void AutowareRos2WhillReceiver::setupDiagnosticUpdater()
{
  diagnostic_updater_.setHardwareID("Autoware ROS2 WHILL Adapter");
  diagnostic_updater_.add("Vehicle error", this, &AutowareRos2WhillReceiver::checkErrCode);
}

void AutowareRos2WhillReceiver::checkErrCode(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  int level = DiagnosticStatus::OK;
  std::string summary_msg = "OK";

  switch (err_code_)
  {
  case 0:
    break;
  case 5:
    level = DiagnosticStatus::WARN;
    summary_msg = "Battery Low";
    break;
  case 8:
    level = DiagnosticStatus::WARN;
    summary_msg = "Uphill";
    break;
  case 14:
    level = DiagnosticStatus::WARN;
    summary_msg = "Low Temperature";
    break;
  case 16:
    level = DiagnosticStatus::WARN;
    summary_msg = "Key Battery Low";
    break;
  case 33:
    level = DiagnosticStatus::WARN;
    summary_msg = "Bttery Timer Initialization Error";
    break;
  case 42:
    level = DiagnosticStatus::WARN;
    summary_msg = "LED Error";
    break;
  case 50:
    level = DiagnosticStatus::WARN;
    summary_msg = "Tail Light Overcurrent";
    break;
  case 63:
    level = DiagnosticStatus::WARN;
    summary_msg = "Speed Profile Setting Error";
    break;
  default:
    level = DiagnosticStatus::ERROR;
    summary_msg = "Fatal Error, Error Code = " + std::to_string(err_code_);
    break;
  }

  stat.summary(level, summary_msg);
}
