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

#include <p9n_tool/crc32.hpp>

namespace p9n_tool
{
uint32_t crc32_le(uint32_t crc, unsigned char const * p, size_t len)
{
  while (len--) {
    crc ^= *p++;
    crc = (crc >> 8) ^ tab[crc & 255];
  }
  return crc;
}
}  //namespace p9n_tool
