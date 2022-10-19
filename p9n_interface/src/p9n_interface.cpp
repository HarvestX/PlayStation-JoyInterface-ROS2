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

std::string getAllHwName()
{
  std::stringstream ss;
  ss << HW_NAME::DUALSHOCK3 << ", ";
  ss << HW_NAME::DUALSHOCK4 << ", ";
  ss << HW_NAME::DUALSENSE;
  return ss.str();
}

std::string getHwName(const HW_TYPE & hw_type)
{
  switch (hw_type) {
    case HW_TYPE::DUALSHOCK3:
      return HW_NAME::DUALSHOCK3;
    case HW_TYPE::DUALSHOCK4:
      return HW_NAME::DUALSHOCK4;
    case HW_TYPE::DUALSENSE:
      return HW_NAME::DUALSENSE;
    default:
      throw std::runtime_error("Invalid hardware type.");
  }
}

HW_TYPE getHwType(const std::string & hw_name)
{
  if (hw_name == HW_NAME::DUALSHOCK3) {
    return HW_TYPE::DUALSHOCK3;
  } else if (hw_name == HW_NAME::DUALSHOCK4) {
    return HW_TYPE::DUALSHOCK4;
  } else if (hw_name == HW_NAME::DUALSENSE) {
    return HW_TYPE::DUALSENSE;
  }
  throw std::runtime_error("Invalid hardware name: " + hw_name);
}


PlayStationInterface::PlayStationInterface(
  const HW_TYPE type
)
: HW_TYPE_(type),
  LOGGER_(rclcpp::get_logger("PlayStationInterface"))
{
  RCLCPP_INFO(
    this->LOGGER_,
    "Hardware: %s",
    getHwName(this->HW_TYPE_).c_str());

  this->btn_idx_ = std::make_unique<JoyButtonIdx>();
  this->axes_idx_ = std::make_unique<JoyAxesIdx>();

  switch (this->HW_TYPE_) {
    case HW_TYPE::DUALSHOCK3:
      {
        using BTN_IDX = BUTTONS_DUALSHOCK3;
        using AXES_IDX = AXES_DUALSHOCK3;

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

        this->btn_idx_->dpad_up =
          static_cast<size_t>(BTN_IDX::UP);
        this->btn_idx_->dpad_down =
          static_cast<size_t>(BTN_IDX::DOWN);
        this->btn_idx_->dpad_right =
          static_cast<size_t>(BTN_IDX::RIGHT);
        this->btn_idx_->dpad_left =
          static_cast<size_t>(BTN_IDX::LEFT);

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
        break;
      }
    case HW_TYPE::DUALSHOCK4:
      {
        using BTN_IDX = BUTTONS_DUALSHOCK4;
        using AXES_IDX = AXES_DUALSHOCK4;

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

bool PlayStationInterface::isTilted(const size_t idx, const double threshold)
{
  return std::abs(this->joy_->axes.at(idx)) > threshold;
}

void PlayStationInterface::setJoyMsg(sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  this->joy_ = msg;
}

bool PlayStationInterface::pressedAny()
{
  if (!this->isAvailable()) {
    return false;
  }

  bool pressed = false;
  for (auto btn : this->joy_->buttons) {
    pressed |= btn;
  }
  for (size_t i = 0; i < this->joy_->axes.size(); ++i) {
    if (i == this->axes_idx_->L2_analog || i == this->axes_idx_->R2_analog) {
      pressed |= this->joy_->axes.at(i) < 0.0;
      continue;
    }
    pressed |= this->isTilted(i);
  }
  return pressed;
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

bool PlayStationInterface::pressedDPadUp()
{
  return this->pressedDPadY() == 1.0;
}

bool PlayStationInterface::pressedDPadDown()
{
  return this->pressedDPadY() == -1.0;
}

bool PlayStationInterface::pressedDPadRight()
{
  return this->pressedDPadX() == -1.0;
}

bool PlayStationInterface::pressedDPadLeft()
{
  return this->pressedDPadX() == 1.0;
}

float PlayStationInterface::pressedDPadX()
{
  if (!this->isAvailable()) {
    return false;
  }
  switch (this->HW_TYPE_) {
    case HW_TYPE::DUALSHOCK3:
      if (this->joy_->buttons.at(this->btn_idx_->dpad_left)) {
        return 1.0;
      } else if (this->joy_->buttons.at(this->btn_idx_->dpad_right)) {
        return -1.0;
      } else {
        return 0.0;
      }
    case HW_TYPE::DUALSHOCK4:
      return this->joy_->axes.at(
        this->axes_idx_->d_pad_x);
    case HW_TYPE::DUALSENSE:
      return this->joy_->axes.at(
        this->axes_idx_->d_pad_x);
    default:
      throw std::runtime_error("Invalid hardware type");
  }
}

float PlayStationInterface::pressedDPadY()
{
  if (!this->isAvailable()) {
    return false;
  }
  switch (this->HW_TYPE_) {
    case HW_TYPE::DUALSHOCK3:
      if (this->joy_->buttons.at(this->btn_idx_->dpad_down)) {
        return -1.0;
      } else if (this->joy_->buttons.at(this->btn_idx_->dpad_up)) {
        return 1.0;
      } else {
        return 0.0;
      }
    case HW_TYPE::DUALSHOCK4:
      return this->joy_->axes.at(
        this->axes_idx_->d_pad_y);
    case HW_TYPE::DUALSENSE:
      return this->joy_->axes.at(
        this->axes_idx_->d_pad_y);
    default:
      throw std::runtime_error("Invalid hardware type");
  }
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

bool PlayStationInterface::isTiltedStickL()
{
  if (!this->isAvailable()) {
    return false;
  }

  return this->isTilted(this->axes_idx_->stick_lx) ||
         this->isTilted(this->axes_idx_->stick_ly);
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

bool PlayStationInterface::isTiltedStickR()
{
  if (!this->isAvailable()) {
    return false;
  }

  return this->isTilted(this->axes_idx_->stick_rx) ||
         this->isTilted(this->axes_idx_->stick_ry);
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
