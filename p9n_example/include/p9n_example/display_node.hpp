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

#include <p9n_interface/p9n_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace p9n_example
{
class DisplayNode : public rclcpp::Node
{
private:
  p9n_interface::HW_TYPE hw_type_;
  std::unique_ptr<p9n_interface::PlayStationInterface> p9n_if_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

public:
  explicit DisplayNode(const rclcpp::NodeOptions &);
  void onJoy(sensor_msgs::msg::Joy::ConstSharedPtr);
};
}  // namespace p9n_example
