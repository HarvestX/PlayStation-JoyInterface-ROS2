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

#include "p9n_interface/hw_types.hpp"

#include "p9n_interface/ps3_dualshock3.hpp"
#include "p9n_interface/ps4_dualshock4.hpp"
#include "p9n_interface/ps5_dualsense.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace p9n_interface
{
typedef struct
{
  size_t square;
  size_t circle;
  size_t triangle;
  size_t cross;

  size_t L1;
  size_t R1;
  size_t R2;
  size_t L2;

  size_t select;
  size_t start;
  size_t PS;

  size_t dpad_up;
  size_t dpad_down;
  size_t dpad_right;
  size_t dpad_left;
} JoyButtonIdx;

typedef struct
{
  size_t d_pad_x;
  size_t d_pad_y;
  size_t stick_lx;
  size_t stick_ly;
  size_t stick_rx;
  size_t stick_ry;

  size_t R2_analog;
  size_t L2_analog;
} JoyAxesIdx;

std::string getAllHwName();
std::string getHwName(const HW_TYPE &);
HW_TYPE getHwType(const std::string & hw_name);

class PlayStationInterface
{
public:
  using UniquePtr = std::unique_ptr<PlayStationInterface>;

private:
  HW_TYPE HW_TYPE_;
  sensor_msgs::msg::Joy::ConstSharedPtr joy_;
  const rclcpp::Logger LOGGER_;
  std::unique_ptr<JoyButtonIdx> btn_idx_;
  std::unique_ptr<JoyAxesIdx> axes_idx_;

private:
  bool isAvailable();
  bool isTilted(const size_t, const double = 1e-1);

public:
  explicit PlayStationInterface(const HW_TYPE);
  void setJoyMsg(sensor_msgs::msg::Joy::ConstSharedPtr);

  bool pressedAny();

  bool pressedSquare();
  bool pressedCircle();
  bool pressedTriangle();
  bool pressedCross();

  bool pressedL1();
  bool pressedR1();
  bool pressedR2();
  bool pressedL2();

  bool pressedSelect();
  bool pressedStart();
  bool pressedPS();

  bool pressedDPadUp();
  bool pressedDPadDown();
  bool pressedDPadLeft();
  bool pressedDPadRight();

  float pressedDPadX();
  float pressedDPadY();

  float tiltedStickLX();
  float tiltedStickLY();
  bool isTiltedStickL();

  float tiltedStickRX();
  float tiltedStickRY();
  bool isTiltedStickR();

  float pressedR2Analog();
  float pressedL2Analog();
};
}  // namespace p9n_interface
