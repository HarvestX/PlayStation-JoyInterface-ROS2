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

#include "p9n_tool/p9n_tool.hpp"

namespace p9n_tool
{
PlayStationTool::PlayStationTool(
)
: LOGGER_(rclcpp::get_logger("PlayStationInterface"))
{
  FILE * fp;
  fp = popen("find /sys/devices/ -name ps-controller-battery*", "r");
  if (fp == NULL) {
  }
}

void PlayStationTool::setLEDMsg(std_msgs::msg::ColorRGBA::ConstSharedPtr msg)
{
  this->color_ = msg;
}
}  // namespace p9n_interface
