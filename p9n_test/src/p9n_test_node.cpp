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

#include "p9n_test/p9n_test_node.hpp"

namespace p9n_test
{
PlayStationTestNode::PlayStationTestNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("p9n_test_node", options)
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

  this->max_trial_ = this->declare_parameter<int>("trial", 3);
  if (this->max_trial_ < 1) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Trial should be grater than 1");
    rclcpp::shutdown();
    return;
  }

  this->joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::SensorDataQoS(rclcpp::KeepLast(1)),
    std::bind(&PlayStationTestNode::onJoy, this, std::placeholders::_1));

  this->wf_handler_ = std::make_unique<WorkflowHandler>();
  this->p9n_interface_ =
    std::make_unique<p9n_interface::PlayStationInterface>(this->hw_type_);


  if (this->joy_sub_->get_publisher_count() == 0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Joy node not launched");
    rclcpp::shutdown();
    return;
  }
}

void PlayStationTestNode::onJoy(sensor_msgs::msg::Joy::ConstSharedPtr joy_msg)
{
  this->p9n_interface_->setJoyMsg(joy_msg);
  this->wf_handler_->explainNextTask(true);
  if (!this->p9n_interface_->pressedAny()) {
    return;
  }

  bool status = this->wf_handler_->execCurrentTask(this->p9n_interface_);

  if (status) {
    RCLCPP_INFO(this->get_logger(), "OK");
    this->wf_handler_->goNextTask();
  } else {
    // When Task failed ..
    if (!this->wf_handler_->isTrialGreaterThan(this->max_trial_)) {
      // You still have chance to try...
      RCLCPP_ERROR(
        this->get_logger(),
        "Invalid input given. Try again...");
    } else {
      // Game over ...
      // Skip current trial.
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed %d times...",
        this->max_trial_);
      RCLCPP_ERROR(
        this->get_logger(),
        "Go next");
      this->wf_handler_->markCurrentTaskAsFailed();
      this->wf_handler_->goNextTask();
      return;
    }
  }

  if (this->wf_handler_->isDone()) {
    RCLCPP_INFO(this->get_logger(), "Done!");
    this->wf_handler_->showResult();
    rclcpp::shutdown();
    return;
  }

  // Delay for button chattering
  using namespace std::chrono_literals;  // NOLINT
  rclcpp::sleep_for(500ms);
}


}  // namespace p9n_test

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(p9n_test::PlayStationTestNode)
