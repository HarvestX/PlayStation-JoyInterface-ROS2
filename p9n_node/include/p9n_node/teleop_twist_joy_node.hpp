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
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <p9n_interface/p9n_interface.hpp>


namespace p9n_node
{
class TeleopTwistJoyNode : public rclcpp::Node
{
public:
  using Twist = geometry_msgs::msg::Twist;
  using Joy = sensor_msgs::msg::Joy;

private:
  double linear_max_speed_, angular_max_speed_;

  p9n_interface::HW_TYPE hw_type_;
  std::unique_ptr<p9n_interface::PlayStationInterface> p9n_if_;

  rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<Twist>::SharedPtr twist_pub_;

  rclcpp::TimerBase::SharedPtr timer_watchdog_;

public:
  TeleopTwistJoyNode() = delete;
  explicit TeleopTwistJoyNode(const rclcpp::NodeOptions & options);
  void onJoy(Joy::ConstSharedPtr joy_msg);
  void onWatchdog();
};
}  // namespace p9n_node
