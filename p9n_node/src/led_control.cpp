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


#include "p9n_node/led_control.hpp"

namespace p9n_node
{
LedControlNode::LedControlNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("led_control_node", options)
{
  this->p9n_to_ =
    std::make_unique<p9n_tool::PlayStationTool>();

  using namespace std::placeholders;  // NOLINT
  this->led_sub_ = this->create_subscription<ColorRGBA>(
    "color", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&LedControlNode::onLed, this, _1)
  );

  using namespace std::chrono_literals; // NOLINT
}

void LedControlNode::onLed(ColorRGBA::ConstSharedPtr color_msg)
{
  this->p9n_to_->setLEDMsg(color_msg);
}
}  // namespace p9n_node
