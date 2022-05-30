// Copyright 2022 HarvestX Inc.
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

#include "p9n_test/workflow_handler.hpp"


namespace p9n_test
{
void WorkflowHandler::goNextTask()
{
  this->trial_ = 0;
  this->current_task_idx_ += 1;
  if (this->current_task_idx_ ==
    static_cast<size_t>(Task::END))
  {
    this->done_ = true;
  } else {
    this->explainNextTask();
  }
}

bool WorkflowHandler::execCurrentTask(
  std::reference_wrapper<std::unique_ptr<p9n_interface::PlayStationInterface>>
  ref_if)
{
  this->trial_ += 1;
  bool trial_result = false;

  switch (this->current_task_idx_) {
    case static_cast<int>(Task::CROSS):
      trial_result = ref_if.get()->pressedCross();
      break;
    case static_cast<int>(Task::CIRCLE):
      trial_result = ref_if.get()->pressedCircle();
      break;
    case static_cast<int>(Task::TRIANGLE):
      trial_result = ref_if.get()->pressedTriangle();
      break;
    case static_cast<int>(Task::SQUARE):
      trial_result = ref_if.get()->pressedSquare();
      break;
    case static_cast<int>(Task::L1):
      trial_result = ref_if.get()->pressedL1();
      break;
    case static_cast<int>(Task::R1):
      trial_result = ref_if.get()->pressedR1();
      break;
    case static_cast<int>(Task::L2):
      trial_result = ref_if.get()->pressedL2();
      break;
    case static_cast<int>(Task::R2):
      trial_result = ref_if.get()->pressedR2();
      break;
    case static_cast<int>(Task::SELECT):
      trial_result = ref_if.get()->pressedSelect();
      break;
    case static_cast<int>(Task::START):
      trial_result = ref_if.get()->pressedStart();
      break;
    case static_cast<int>(Task::PS):
      trial_result = ref_if.get()->pressedPS();
      break;
    case static_cast<int>(Task::DPAD_UP):
      trial_result = ref_if.get()->pressedDPadUp();
      break;
    case static_cast<int>(Task::DPAD_DOWN):
      trial_result = ref_if.get()->pressedDPadDown();
      break;
    case static_cast<int>(Task::DPAD_RIGHT):
      trial_result = ref_if.get()->pressedDPadRight();
      break;
    case static_cast<int>(Task::DPAD_LEFT):
      trial_result = ref_if.get()->pressedDPadLeft();
      break;
    case static_cast<int>(Task::STICK_L_LEFT):
      trial_result =
        ref_if.get()->tiltedStickLX() > 1e-1;
      break;
    case static_cast<int>(Task::STICK_L_UP):
      trial_result =
        ref_if.get()->tiltedStickLY() > 1e-1;
      break;
    case static_cast<int>(Task::STICK_R_LEFT):
      trial_result =
        ref_if.get()->tiltedStickRX() > 1e-1;
      break;
    case static_cast<int>(Task::STICK_R_UP):
      trial_result =
        ref_if.get()->tiltedStickRY() > 1e-1;
      break;
    case static_cast<int>(Task::END):
    default:
      RCLCPP_ERROR(
        this->LOGGER_,
        "Now workflow remains");
      return true;
  }

  return trial_result;
}

std::string WorkflowHandler::getCurrentTaskStr()
{
  return this->getTaskStr(this->current_task_idx_);
}

std::string WorkflowHandler::getTaskStr(const int task_idx)
{
  switch (task_idx) {
    case static_cast<int>(Task::CROSS):
      return "☓";
    case static_cast<int>(Task::CIRCLE):
      return "○";
    case static_cast<int>(Task::TRIANGLE):
      return "△";
    case static_cast<int>(Task::SQUARE):
      return "□";
    case static_cast<int>(Task::L1):
      return "L1";
    case static_cast<int>(Task::R1):
      return "R1";
    case static_cast<int>(Task::L2):
      return "L2";
    case static_cast<int>(Task::R2):
      return "R2";
    case static_cast<int>(Task::SELECT):
      return "Select";
    case static_cast<int>(Task::START):
      return "Start";
    case static_cast<int>(Task::PS):
      return "PS";
    case static_cast<int>(Task::DPAD_UP):
      return "Button ↑";
    case static_cast<int>(Task::DPAD_DOWN):
      return "Button ↓";
    case static_cast<int>(Task::DPAD_RIGHT):
      return "Button →";
    case static_cast<int>(Task::DPAD_LEFT):
      return "Button ←";
    case static_cast<int>(Task::STICK_L_LEFT):
      return "Stick L ←";
    case static_cast<int>(Task::STICK_L_UP):
      return "Stick L ↑";
    case static_cast<int>(Task::STICK_R_LEFT):
      return "Stick R ←";
    case static_cast<int>(Task::STICK_R_UP):
      return "Stick R ↑";
    case static_cast<int>(Task::END):
    default:
      return "All Task Done";
  }
}

bool WorkflowHandler::isDone()
{
  return this->done_;
}

bool WorkflowHandler::isTrialGreaterThan(const int max)
{
  return this->trial_ >= max;
}

void WorkflowHandler::explainNextTask(bool once)
{
  const std::string text =
    "Press [ " + this->getCurrentTaskStr() +
    " ] button : ";
  if (once) {
    RCLCPP_WARN_ONCE(
      this->LOGGER_,
      text.c_str());
  } else {
    RCLCPP_WARN(
      this->LOGGER_,
      text.c_str());
  }
}

void WorkflowHandler::markCurrentTaskAsFailed()
{
  this->failed_idx_list_.emplace_back(this->current_task_idx_);
}

void WorkflowHandler::showResult()
{
  if (this->failed_idx_list_.empty()) {
    RCLCPP_INFO(
      this->LOGGER_,
      "Passed all task! 🍻");
  } else {
    std::stringstream ss;
    ss << "Failed target: ";
    for (auto failed_idx : this->failed_idx_list_) {
      ss << "[ " <<
        this->getTaskStr(failed_idx) << " ] ";
    }
    RCLCPP_ERROR(
      this->LOGGER_,
      ss.str().c_str());
  }
}
}  // namespace p9n_test
