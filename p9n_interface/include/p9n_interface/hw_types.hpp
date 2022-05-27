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

namespace p9n_interface
{
enum class HW_TYPE
{
  DUALSHOCK3,
  DUALSHOCK4,
  DUALSENSE,
};

std::string getHWName(const HW_TYPE & hw_type)
{
  switch (hw_type) {
    case HW_TYPE::DUALSHOCK3:
      return "DualShock3";
    case HW_TYPE::DUALSHOCK4:
      return "DualShock4";
    case HW_TYPE::DUALSENSE:
      return "DualSense";
    default:
      return "Unknown";
  }
}
}  // namespace p9n_interface
