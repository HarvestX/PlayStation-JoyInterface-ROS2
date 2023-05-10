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

#include <unistd.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <poll.h>
#include <sys/wait.h>

#include <hidapi/hidapi.h>
#include <libudev.h>

#include "crc32.hpp"


#define DS_VENDOR_ID 0x054c
#define DS_PRODUCT_ID 0x0ce6

/* Seed values for DualShock4 / DualSense CRC32 for different report types. */
#define PS_INPUT_CRC32_SEED 0xA1
#define PS_OUTPUT_CRC32_SEED 0xA2
#define PS_FEATURE_CRC32_SEED 0xA3

#define DS_INPUT_REPORT_USB 0x01
#define DS_INPUT_REPORT_USB_SIZE 64
#define DS_INPUT_REPORT_BT 0x31
#define DS_INPUT_REPORT_BT_SIZE 78
#define DS_OUTPUT_REPORT_USB 0x02
#define DS_OUTPUT_REPORT_USB_SIZE 63
#define DS_OUTPUT_REPORT_BT 0x31
#define DS_OUTPUT_REPORT_BT_SIZE 78

#define DS_FEATURE_REPORT_CALIBRATION 0x05
#define DS_FEATURE_REPORT_CALIBRATION_SIZE 41
#define DS_FEATURE_REPORT_PAIRING_INFO 0x09
#define DS_FEATURE_REPORT_PAIRING_INFO_SIZE 20
#define DS_FEATURE_REPORT_FIRMWARE_INFO 0x20
#define DS_FEATURE_REPORT_FIRMWARE_INFO_SIZE 64

/* Magic value required in tag field of Bluetooth output report. */
#define DS_OUTPUT_TAG 0x10
/* Flags for DualSense output report. */
#define BIT(n) (1 << n)
#define DS_OUTPUT_VALID_FLAG0_COMPATIBLE_VIBRATION BIT(0)
#define DS_OUTPUT_VALID_FLAG0_HAPTICS_SELECT BIT(1)
#define DS_OUTPUT_VALID_FLAG1_MIC_MUTE_LED_CONTROL_ENABLE BIT(0)
#define DS_OUTPUT_VALID_FLAG1_POWER_SAVE_CONTROL_ENABLE BIT(1)
#define DS_OUTPUT_VALID_FLAG1_LIGHTBAR_CONTROL_ENABLE BIT(2)
#define DS_OUTPUT_VALID_FLAG1_RELEASE_LEDS BIT(3)
#define DS_OUTPUT_VALID_FLAG1_PLAYER_INDICATOR_CONTROL_ENABLE BIT(4)
#define DS_OUTPUT_VALID_FLAG2_LIGHTBAR_SETUP_CONTROL_ENABLE BIT(1)
#define DS_OUTPUT_POWER_SAVE_CONTROL_MIC_MUTE BIT(4)
#define DS_OUTPUT_LIGHTBAR_SETUP_LIGHT_ON BIT(0)
#define DS_OUTPUT_LIGHTBAR_SETUP_LIGHT_OUT BIT(1)

/* Status field of DualSense input report. */
#define DS_STATUS_BATTERY_CAPACITY 0xF
#define DS_STATUS_CHARGING 0xF0
#define DS_STATUS_CHARGING_SHIFT 4

namespace p9n_tool
{

struct dualsense_touch_point
{
  uint8_t contact;
  uint8_t x_lo;
  uint8_t x_hi : 4, y_lo : 4;
  uint8_t y_hi;
} __attribute__((packed));

/* Main DualSense input report excluding any BT/USB specific headers. */
struct dualsense_input_report
{
  uint8_t x, y;
  uint8_t rx, ry;
  uint8_t z, rz;
  uint8_t seq_number;
  uint8_t buttons[4];
  uint8_t reserved[4];

  /* Motion sensors */
  uint16_t gyro[3];   /* x, y, z */
  uint16_t accel[3];   /* x, y, z */
  uint32_t sensor_timestamp;
  uint8_t reserved2;

  /* Touchpad */
  struct dualsense_touch_point points[2];

  uint8_t reserved3[12];
  uint8_t status;
  uint8_t reserved4[10];
} __attribute__((packed));

/* Common data between DualSense BT/USB main output report. */
struct dualsense_output_report_common
{
  uint8_t valid_flag0;
  uint8_t valid_flag1;

  /* For DualShock 4 compatibility mode. */
  uint8_t motor_right;
  uint8_t motor_left;

  /* Audio controls */
  uint8_t reserved[4];
  uint8_t mute_button_led;

  uint8_t power_save_control;
  uint8_t reserved2[28];

  /* LEDs and lightbar */
  uint8_t valid_flag2;
  uint8_t reserved3[2];
  uint8_t lightbar_setup;
  uint8_t led_brightness;
  uint8_t player_leds;
  uint8_t lightbar_red;
  uint8_t lightbar_green;
  uint8_t lightbar_blue;
} __attribute__((packed));

struct dualsense_output_report_bt
{
  uint8_t report_id;   /* 0x31 */
  uint8_t seq_tag;
  uint8_t tag;
  struct dualsense_output_report_common common;
  uint8_t reserved[24];
  uint32_t crc32;
} __attribute__((packed));

struct dualsense_output_report_usb
{
  uint8_t report_id;   /* 0x02 */
  struct dualsense_output_report_common common;
  uint8_t reserved[15];
} __attribute__((packed));

/*
 * The DualSense has a main output report used to control most features. It is
 * largely the same between Bluetooth and USB except for different headers and CRC.
 * This structure hide the differences between the two to simplify sending output reports.
 */
struct dualsense_output_report
{
  uint8_t * data;  /* Start of data */
  uint8_t len;   /* Size of output report */

  /* Points to Bluetooth data payload in case for a Bluetooth report else NULL. */
  struct dualsense_output_report_bt * bt;
  /* Points to USB data payload in case for a USB report else NULL. */
  struct dualsense_output_report_usb * usb;
  /* Points to common section of report, so past any headers. */
  struct dualsense_output_report_common * common;
};

struct dualsense
{
  bool bt;
  hid_device * dev;
  char mac_address[18];
  uint8_t output_seq;
};

void dualsense_init_output_report(
  struct dualsense * ds, struct dualsense_output_report * rp,
  void * buf);

void dualsense_send_output_report(
  struct dualsense * ds,
  struct dualsense_output_report * report);

bool compare_serial(const char * s, const wchar_t * dev);

bool dualsense_init(struct dualsense * ds, const char * serial);

void dualsense_destroy(struct dualsense * ds);

int command_lightbar1(struct dualsense * ds, char * state);

int command_lightbar3(
  struct dualsense * ds, uint8_t red, uint8_t green, uint8_t blue,
  uint8_t brightness);

int command_player_leds(struct dualsense * ds, uint8_t number);

int command_microphone_led(struct dualsense * ds, char * state);
} // namespace p9n_tool
