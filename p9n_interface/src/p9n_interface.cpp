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

#include "p9n_interface/p9n_interface.hpp"

namespace p9n_interface
{
PlayStationInterface::PlayStationInterface(
  const HW_TYPE type
)
: HW_TYPE_(type),
  LOGGER_(rclcpp::get_logger("PlayStationInterface"))
{
  RCLCPP_INFO(
    this->LOGGER_,
    "Hardware Type: %s",
    getHWName(type).c_str());

  this->btn_idx_ = std::make_unique<JoyButtonIdx>();
  this->axes_idx_ = std::make_unique<JoyAxesIdx>();

  switch (this->HW_TYPE_) {
    case HW_TYPE::DUALSHOCK3:
      throw std::runtime_error("Not supported yet.");
    case HW_TYPE::DUALSHOCK4:
      throw std::runtime_error("Not supported yet.");
    case HW_TYPE::DUALSENSE:
      {
        using BTN_IDX = BUTTONS_DUALSENSE;
        using AXES_IDX = AXES_DUALSENSE;

        this->btn_idx_->cross =
          static_cast<size_t>(BTN_IDX::CROSS);
        this->btn_idx_->circle =
          static_cast<size_t>(BTN_IDX::CIRCLE);
        this->btn_idx_->triangle =
          static_cast<size_t>(BTN_IDX::TRIANGLE);
        this->btn_idx_->square =
          static_cast<size_t>(BTN_IDX::SQUARE);

        this->btn_idx_->L1 =
          static_cast<size_t>(BTN_IDX::L1);
        this->btn_idx_->R1 =
          static_cast<size_t>(BTN_IDX::R1);
        this->btn_idx_->L2 =
          static_cast<size_t>(BTN_IDX::L2);
        this->btn_idx_->R2 =
          static_cast<size_t>(BTN_IDX::R2);

        this->btn_idx_->select =
          static_cast<size_t>(BTN_IDX::SELECT);
        this->btn_idx_->start =
          static_cast<size_t>(BTN_IDX::START);
        this->btn_idx_->PS =
          static_cast<size_t>(BTN_IDX::PS);

        this->axes_idx_->stick_lx =
          static_cast<size_t>(AXES_IDX::STICK_LX);
        this->axes_idx_->stick_ly =
          static_cast<size_t>(AXES_IDX::STICK_LY);
        this->axes_idx_->stick_rx =
          static_cast<size_t>(AXES_IDX::STICK_RX);
        this->axes_idx_->stick_ry =
          static_cast<size_t>(AXES_IDX::STICK_RY);
        this->axes_idx_->R2_analog =
          static_cast<size_t>(AXES_IDX::R2);
        this->axes_idx_->L2_analog =
          static_cast<size_t>(AXES_IDX::L2);
        this->axes_idx_->d_pad_x =
          static_cast<size_t>(AXES_IDX::DPAD_X);
        this->axes_idx_->d_pad_y =
          static_cast<size_t>(AXES_IDX::DPAD_Y);

        break;
      }
    default:
      throw std::runtime_error("Unknown Hardware Type.");
      break;
  }
}

bool PlayStationInterface::isAvailable()
{
  if (this->joy_) {
    return true;
  }
  RCLCPP_ERROR_ONCE(
    this->LOGGER_,
    "Joy Message not set. Please call setJotMsg before use.");
  return false;
}

void PlayStationInterface::setJoyMsg(sensor_msgs::msg::Joy::UniquePtr msg)
{
  this->joy_ = std::move(msg);
}

bool PlayStationInterface::pressedSquare()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->square);
}

bool PlayStationInterface::pressedCircle()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->circle);
}

bool PlayStationInterface::pressedTriangle()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->triangle);
}

bool PlayStationInterface::pressedCross()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->cross);
}

bool PlayStationInterface::pressedL1()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->L1);
}

bool PlayStationInterface::pressedR1()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->R1);
}

bool PlayStationInterface::pressedR2()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->R2);
}

bool PlayStationInterface::pressedL2()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->L2);
}

bool PlayStationInterface::pressedSelect()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->select);
}

bool PlayStationInterface::pressedStart()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->start);
}

bool PlayStationInterface::pressedPS()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->buttons.at(
    this->btn_idx_->PS);
}

float PlayStationInterface::pressedDPadX()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->d_pad_x);
}

float PlayStationInterface::pressedDPadY()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->d_pad_y);
}

float PlayStationInterface::tiltedStickLX()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->stick_lx);
}

float PlayStationInterface::tiltedStickLY()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->stick_ly);
}

float PlayStationInterface::tiltedStickRX()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->stick_rx);
}

float PlayStationInterface::tiltedStickRY()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->stick_ry);
}

float PlayStationInterface::pressedR2Analog()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->R2_analog);
}

float PlayStationInterface::pressedL2Analog()
{
  if (!this->isAvailable()) {
    return false;
  }
  return this->joy_->axes.at(
    this->axes_idx_->L2_analog);
}


}  // namespace p9n_interface
