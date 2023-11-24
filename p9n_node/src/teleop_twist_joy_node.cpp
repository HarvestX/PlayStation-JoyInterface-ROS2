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
  this->linear_max_speed_ =
    this->declare_parameter<double>("linear_speed", 0.2);
  this->angular_max_speed_ =
    this->declare_parameter<double>("angular_speed", 0.6);

  try {
    this->hw_type_ = p9n_interface::getHwType(hw_name);
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
    RCLCPP_ERROR(
      this->get_logger(), "Please select hardware from %s",
      p9n_interface::getAllHwName().c_str());
    exit(EXIT_FAILURE);
    return;
  }

  this->p9n_if_ =
    std::make_unique<p9n_interface::PlayStationInterface>(this->hw_type_);

  using namespace std::placeholders;  // NOLINT
  this->joy_sub_ = this->create_subscription<Joy>(
    "joy", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&TeleopTwistJoyNode::onJoy, this, _1));

  this->twist_pub_ = this->create_publisher<Twist>(
    "cmd_vel", rclcpp::QoS(10).reliable().durability_volatile());


  using namespace std::chrono_literals; // NOLINT
  this->timer_watchdog_ = this->create_wall_timer(
    1s, std::bind(&TeleopTwistJoyNode::onWatchdog, this));

  if (this->joy_sub_->get_publisher_count() == 0) {
    RCLCPP_WARN(this->get_logger(), "Joy node not launched");
  }
}

void TeleopTwistJoyNode::onJoy(Joy::ConstSharedPtr joy_msg)
{
  this->timer_watchdog_->reset();
  this->p9n_if_->setJoyMsg(joy_msg);

  static bool stopped = true;
  if (this->p9n_if_->isTiltedStickL()) {
    auto twist_msg = std::make_unique<Twist>();
    twist_msg->linear.set__x(this->linear_max_speed_ * this->p9n_if_->tiltedStickLY());

    if (this->p9n_if_->tiltedStickLY() > sin(M_PI * 0.125)) {
      twist_msg->angular.set__z(this->angular_max_speed_ * this->p9n_if_->tiltedStickLX());
    } else if (this->p9n_if_->tiltedStickLY() < sin(-M_PI * 0.125)) {
      twist_msg->angular.set__z(-this->angular_max_speed_ * this->p9n_if_->tiltedStickLX());
    } else {
      twist_msg->linear.set__x(0.0);
      twist_msg->angular.set__z(this->angular_max_speed_ * this->p9n_if_->tiltedStickLX());
    }
    this->twist_pub_->publish(std::move(twist_msg));

    stopped = false;
  } else if (!stopped) {
    // Stop cart
    auto twist_msg = std::make_unique<Twist>(rosidl_runtime_cpp::MessageInitialization::ZERO);
    this->twist_pub_->publish(std::move(twist_msg));

    stopped = true;
  }
}

void TeleopTwistJoyNode::onWatchdog()
{
  RCLCPP_WARN(this->get_logger(), "Couldn't subscribe joy topic before timeout");

  // Publish zero velocity to stop vehicle
  auto twist_msg = std::make_unique<Twist>(rosidl_runtime_cpp::MessageInitialization::ZERO);
  this->twist_pub_->publish(std::move(twist_msg));

  this->timer_watchdog_->cancel();
}
}  // namespace p9n_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(p9n_node::TeleopTwistJoyNode)
