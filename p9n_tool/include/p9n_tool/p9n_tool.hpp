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
#include <memory>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "p9n_utilities.hpp"

namespace p9n_tool
{
class PlayStationTool
{
public:
  using UniquePtr = std::unique_ptr<PlayStationTool>;

private:
  const rclcpp::Logger LOGGER_;
  const char * dev_serial = NULL;
  struct dualsense ds;

public:
  explicit PlayStationTool();
  ~PlayStationTool();
  void setLEDMsg(std_msgs::msg::ColorRGBA::ConstSharedPtr);
};
}  // namespace p9n_tool