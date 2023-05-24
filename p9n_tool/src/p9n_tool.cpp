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
  if (!dualsense_init(&ds, dev_serial)) {
    RCLCPP_ERROR(LOGGER_, "couldn't init");
  }
}

PlayStationTool::~PlayStationTool()
{
  dualsense_close(&ds);
}

void PlayStationTool::setLEDMsg(std_msgs::msg::ColorRGBA::ConstSharedPtr msg)
{
  uint8_t r = std::round(msg->r * 255);
  uint8_t g = std::round(msg->g * 255);
  uint8_t b = std::round(msg->b * 255);
  uint8_t brightness = std::round(msg->a * 255);

  command_lightbar_color(&ds, r, g, b, brightness);
}
}  // namespace p9n_tool
