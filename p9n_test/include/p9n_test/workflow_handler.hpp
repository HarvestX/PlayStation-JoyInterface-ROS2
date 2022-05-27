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

#pragma once

#include <string>
#include <vector>
#include <memory>

#include <p9n_interface/p9n_interface.hpp>

namespace p9n_test
{

enum class Task
{
  CROSS = 0,
  CIRCLE,
  TRIANGLE,
  SQUARE,

  L1,
  R1,
  L2,
  R2,

  SELECT,
  START,
  PS,

  DPAD_UP,
  DPAD_DOWN,
  DPAD_RIGHT,
  DPAD_LEFT,

  STICK_L_LEFT,
  STICK_L_UP,
  STICK_R_LEFT,
  STICK_R_UP,

  END
};

class WorkflowHandler
{
private:
  int trial_ = 0;
  int current_task_idx_ = 0;
  bool done_ = false;
  rclcpp::Logger LOGGER_ = rclcpp::get_logger("WorkflowHandler");

  std::vector<int> failed_idx_list_;

public:
  void goNextTask();
  bool execCurrentTask(
    std::reference_wrapper<
      std::unique_ptr<p9n_interface::PlayStationInterface>>);
  std::string getCurrentTaskStr();
  std::string getTaskStr(const int);
  bool isDone();
  bool isTrialGreaterThan(const int);
  void explainNextTask(bool = false);
  void markCurrentTaskAsFailed();
  void showResult();
};
}  // namespace p9n_test
