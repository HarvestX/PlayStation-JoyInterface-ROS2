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

namespace p9n_interface
{
enum class BUTTONS_DUALSHOCK4
{
  CROSS = 0,
  CIRCLE,
  TRIANGLE,
  SQUARE,
  L1,
  R1,
  L2,
  R2,
  SELECT,
  START,
  PS
};

enum class AXES_DUALSHOCK4
{
  STICK_LX = 0,
  STICK_LY,
  L2,
  STICK_RX,
  STICK_RY,
  R2,
  DPAD_X,
  DPAD_Y,
};

}  // namespace p9n_interface
