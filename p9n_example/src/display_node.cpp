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


#include "p9n_example/display_node.hpp"


namespace p9n_example
{
DisplayNode::DisplayNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("display_node", options)
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
    std::bind(&DisplayNode::onJoy, this, std::placeholders::_1));


  if (this->joy_sub_->get_publisher_count() == 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Joy node not launched");
  }
}

void DisplayNode::onJoy(sensor_msgs::msg::Joy::ConstSharedPtr joy_msg)
{
  this->p9n_if_->setJoyMsg(joy_msg);

  if (this->p9n_if_->pressedCross()) {
    RCLCPP_INFO(this->get_logger(), " ☓ Pressed!");
  }

  if (this->p9n_if_->pressedCircle()) {
    RCLCPP_INFO(this->get_logger(), " ○ Pressed!");
  }

  if (this->p9n_if_->pressedTriangle()) {
    RCLCPP_INFO(this->get_logger(), " △ Pressed!");
  }

  if (this->p9n_if_->pressedSquare()) {
    RCLCPP_INFO(this->get_logger(), " □ Pressed!");
  }

  if (this->p9n_if_->pressedL1()) {
    RCLCPP_INFO(this->get_logger(), " L1 Pressed!");
  }

  if (this->p9n_if_->pressedR1()) {
    RCLCPP_INFO(this->get_logger(), " R1 Pressed!");
  }

  if (this->p9n_if_->pressedL2()) {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      " L2 Pressed! : " <<
        std::right << std::setw(6) <<
        std::fixed << std::setprecision(3) <<
        this->p9n_if_->pressedL2Analog());
  }

  if (this->p9n_if_->pressedR2()) {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      " R2 Pressed! : " <<
        std::right << std::setw(6) <<
        std::fixed << std::setprecision(3) <<
        this->p9n_if_->pressedR2Analog());
  }

  if (this->p9n_if_->pressedSelect()) {
    RCLCPP_INFO(
      this->get_logger(), " Select Pressed!");
  }

  if (this->p9n_if_->pressedStart()) {
    RCLCPP_INFO(
      this->get_logger(), " Start Pressed!");
  }

  if (this->p9n_if_->pressedPS()) {
    RCLCPP_INFO(
      this->get_logger(), " PS Pressed!");
  }

  if (this->p9n_if_->pressedDPadUp()) {
    RCLCPP_INFO(
      this->get_logger(), " ↑ Pressed!");
  }

  if (this->p9n_if_->pressedDPadDown()) {
    RCLCPP_INFO(
      this->get_logger(), " ↓ Pressed!");
  }

  if (this->p9n_if_->pressedDPadRight()) {
    RCLCPP_INFO(
      this->get_logger(), " → Pressed!");
  }

  if (this->p9n_if_->pressedDPadLeft()) {
    RCLCPP_INFO(
      this->get_logger(), " ← Pressed!");
  }

  if (
    std::abs(this->p9n_if_->tiltedStickLX()) > 1e-2 ||
    std::abs(this->p9n_if_->tiltedStickLY()) > 1e-2)
  {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      " LStick Tilted!" <<
        " x:" << std::fixed << std::setw(6) << std::setprecision(3) <<
        this->p9n_if_->tiltedStickLX() <<
        " y:" << std::fixed << std::setw(6) << std::setprecision(3) <<
        this->p9n_if_->tiltedStickLY());
  }

  if (
    std::abs(this->p9n_if_->tiltedStickRX()) > 1e-2 ||
    std::abs(this->p9n_if_->tiltedStickRY()) > 1e-2)
  {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      " RStick Tilted!" <<
        " x:" << std::fixed << std::setw(6) << std::setprecision(3) <<
        this->p9n_if_->tiltedStickRX() <<
        " y:" << std::fixed << std::setw(6) << std::setprecision(3) <<
        this->p9n_if_->tiltedStickRY());
  }

  if (this->p9n_if_->pressedAny()) {
    using namespace std::chrono_literals;  // NOLINT
    rclcpp::sleep_for(200ms);
  }
}
}  // namespace p9n_example

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(p9n_example::DisplayNode)
