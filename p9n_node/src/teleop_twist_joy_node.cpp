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


#include "p9n_node/teleop_twist_joy_node.hpp"

namespace p9n_node
{

TeleopTwistJoyNode::TeleopTwistJoyNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("teleop_twist_joy_node", options)
{

  const std::string hw_name = this->declare_parameter<std::string>(
    "hw_type", p9n_interface::HW_NAME::DUALSENSE);

  try {
    this->hw_type_ = p9n_interface::getHwType(hw_name);
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      e.what());
    RCLCPP_ERROR(
      this->get_logger(),
      "Please select hardware from %s",
      p9n_interface::getAllHwName().c_str());
    rclcpp::shutdown();
    return;
  }

  this->p9n_if_ =
    std::make_unique<p9n_interface::PlayStationInterface>(this->hw_type_);

  this->joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::SensorDataQoS(rclcpp::KeepLast(1)),
    std::bind(&TeleopTwistJoyNode::onJoy, this, std::placeholders::_1));

  this->twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10));


  if (this->joy_sub_->get_publisher_count() == 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Joy node not launched");
  }
}

void TeleopTwistJoyNode::onJoy(sensor_msgs::msg::Joy::ConstSharedPtr joy_msg)
{
  this->p9n_if_->setJoyMsg(joy_msg);
  if (
    std::abs(this->p9n_if_->tiltedStickLX()) > 1e-2 ||
    std::abs(this->p9n_if_->tiltedStickLY()) > 1e-2)
  {
    float l_x = this->p9n_if_->tiltedStickLX();
    float l_y = this->p9n_if_->tiltedStickLY();

    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = l_y;
    twist.angular.z = l_x;
    this->twist_pub_->publish(twist);
  } else {
    using namespace std::chrono_literals;
    rclcpp::sleep_for(100ms);
  }
}
} // namespace p9n_node
