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
#include <std_msgs/msg/color_rgba.hpp>
#include <rclcpp/rclcpp.hpp>

#include <p9n_interface/p9n_interface.hpp>
#include <p9n_tool/p9n_tool.hpp>


namespace p9n_node
{
class LedControlNode : public rclcpp::Node
{
public:
  using ColorRGBA = std_msgs::msg::ColorRGBA;

private:
  p9n_interface::HW_TYPE hw_type_;
  std::unique_ptr<p9n_tool::PlayStationTool> p9n_to_;

  rclcpp::Subscription<ColorRGBA>::SharedPtr led_sub_;

public:
  LedControlNode() = delete;
  explicit LedControlNode(const rclcpp::NodeOptions & options);
  void onLed(ColorRGBA::ConstSharedPtr color_msg);
};
}  // namespace p9n_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(p9n_node::LedControlNode)
