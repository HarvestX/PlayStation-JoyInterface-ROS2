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

#include "p9n_tool/p9n_utilities.hpp"

namespace p9n_tool
{

void dualsense_init_output_report(
  struct dualsense * ds, struct dualsense_output_report * rp,
  void * buf)
{
  if (ds->bt) {
    struct dualsense_output_report_bt * bt = (struct dualsense_output_report_bt *)buf;

    memset(bt, 0, sizeof(*bt));
    bt->report_id = DS_OUTPUT_REPORT_BT;
    bt->tag = DS_OUTPUT_TAG;     /* Tag must be set. Exact meaning is unclear. */

    /*
     * Highest 4-bit is a sequence number, which needs to be increased
     * every report. Lowest 4-bit is tag and can be zero for now.
     */
    bt->seq_tag = (ds->output_seq << 4) | 0x0;
    if (++ds->output_seq == 16) {
      ds->output_seq = 0;
    }

    rp->data = (uint8_t *)buf;
    rp->len = sizeof(*bt);
    rp->bt = bt;
    rp->usb = NULL;
    rp->common = &bt->common;
  } else {   /* USB */
    struct dualsense_output_report_usb * usb = (struct dualsense_output_report_usb *)buf;

    memset(usb, 0, sizeof(*usb));
    usb->report_id = DS_OUTPUT_REPORT_USB;

    rp->data = (uint8_t *)buf;
    rp->len = sizeof(*usb);
    rp->bt = NULL;
    rp->usb = usb;
    rp->common = &usb->common;
  }
}

void dualsense_send_output_report(
  struct dualsense * ds,
  struct dualsense_output_report * report)
{
  /* Bluetooth packets need to be signed with a CRC in the last 4 bytes. */
  if (report->bt) {
    uint32_t crc;
    uint8_t seed = PS_OUTPUT_CRC32_SEED;

    crc = crc32_le(0xFFFFFFFF, &seed, 1);
    crc = ~crc32_le(crc, report->data, report->len - 4);

    report->bt->crc32 = crc;
  }

  int res = hid_write(ds->dev, report->data, report->len);
  if (res < 0) {
    fprintf(stderr, "Error: %ls\n", hid_error(ds->dev));
  }
}

bool compare_serial(const char * s, const wchar_t * dev)
{
  if (!s) {
    return true;
  }
  const size_t len = wcslen(dev);
  if (strlen(s) != len) {
    return false;
  }
  for (size_t i = 0; i < len; ++i) {
    if (s[i] != dev[i]) {
      return false;
    }
  }
  return true;
}

bool dualsense_init(struct dualsense * ds, const char * serial)
{
  bool ret = false;

  memset(ds, 0, sizeof(*ds));

  bool found = false;
  struct hid_device_info * devs = hid_enumerate(DS_VENDOR_ID, DS_PRODUCT_ID);
  struct hid_device_info * dev = devs;
  while (dev) {
    if (compare_serial(serial, dev->serial_number)) {
      found = true;
      break;
    }
    dev = dev->next;
  }

  if (!found) {
    if (serial) {
      fprintf(stderr, "Device '%s' not found\n", serial);
    } else {
      fprintf(stderr, "No device found\n");
    }
    ret = false;
    if (devs) {
      hid_free_enumeration(devs);
    }
    return ret;
  }

  ds->dev = hid_open(DS_VENDOR_ID, DS_PRODUCT_ID, dev->serial_number);
  if (!ds->dev) {
    fprintf(stderr, "Failed to open device: %ls\n", hid_error(NULL));
    ret = false;
    if (devs) {
      hid_free_enumeration(devs);
    }
    return ret;
  }

  wchar_t * serial_number = dev->serial_number;

  if (wcslen(serial_number) != 17) {
    fprintf(stderr, "Invalid device serial number: %ls\n", serial_number);
    // Let's just fake serial number as everything except disconnecting will still work
    serial_number = L"00:00:00:00:00:00";
  }

  for (int i = 0; i < 18; ++i) {
    char c = serial_number[i];
    if (c && (i + 1) % 3) {
      c = toupper(c);
    }
    ds->mac_address[i] = c;
  }

  ds->bt = dev->interface_number == -1;

  ret = true;
  if (devs) {
    hid_free_enumeration(devs);
  }
  return ret;
}

void dualsense_destroy(struct dualsense * ds)
{
  hid_close(ds->dev);
}


int command_lightbar1(struct dualsense * ds, char * state)
{
  struct dualsense_output_report rp;
  uint8_t rbuf[DS_OUTPUT_REPORT_BT_SIZE];
  dualsense_init_output_report(ds, &rp, rbuf);

  rp.common->valid_flag2 = DS_OUTPUT_VALID_FLAG2_LIGHTBAR_SETUP_CONTROL_ENABLE;
  if (!strcmp(state, "on")) {
    rp.common->lightbar_setup = DS_OUTPUT_LIGHTBAR_SETUP_LIGHT_ON;
  } else if (!strcmp(state, "off")) {
    rp.common->lightbar_setup = DS_OUTPUT_LIGHTBAR_SETUP_LIGHT_OUT;
  } else {
    fprintf(stderr, "Invalid state\n");
    return 1;
  }

  dualsense_send_output_report(ds, &rp);

  return 0;
}

int command_lightbar3(
  struct dualsense * ds, uint8_t red, uint8_t green, uint8_t blue,
  uint8_t brightness)
{
  struct dualsense_output_report rp;
  uint8_t rbuf[DS_OUTPUT_REPORT_BT_SIZE];
  dualsense_init_output_report(ds, &rp, rbuf);

  uint8_t max_brightness = 255;

  rp.common->valid_flag1 = DS_OUTPUT_VALID_FLAG1_LIGHTBAR_CONTROL_ENABLE;
  rp.common->lightbar_red = brightness * red / max_brightness;
  rp.common->lightbar_green = brightness * green / max_brightness;
  rp.common->lightbar_blue = brightness * blue / max_brightness;

  dualsense_send_output_report(ds, &rp);

  return 0;
}

int command_player_leds(struct dualsense * ds, uint8_t number)
{
  if (number > 5) {
    fprintf(stderr, "Invalid player number\n");
    return 1;
  }

  struct dualsense_output_report rp;
  uint8_t rbuf[DS_OUTPUT_REPORT_BT_SIZE];
  dualsense_init_output_report(ds, &rp, rbuf);

  static const int player_ids[6] = {
    0,
    BIT(2),
    BIT(3) | BIT(1),
    BIT(4) | BIT(2) | BIT(0),
    BIT(4) | BIT(3) | BIT(1) | BIT(0),
    BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0)
  };

  rp.common->valid_flag1 = DS_OUTPUT_VALID_FLAG1_PLAYER_INDICATOR_CONTROL_ENABLE;
  rp.common->player_leds = player_ids[number];

  dualsense_send_output_report(ds, &rp);

  return 0;
}

int command_microphone_led(struct dualsense * ds, char * state)
{
  struct dualsense_output_report rp;
  uint8_t rbuf[DS_OUTPUT_REPORT_BT_SIZE];
  dualsense_init_output_report(ds, &rp, rbuf);

  rp.common->valid_flag1 = DS_OUTPUT_VALID_FLAG1_MIC_MUTE_LED_CONTROL_ENABLE;
  if (!strcmp(state, "on")) {
    rp.common->mute_button_led = 1;
  } else if (!strcmp(state, "off")) {
    rp.common->mute_button_led = 0;
  } else {
    fprintf(stderr, "Invalid state\n");
    return 1;
  }

  dualsense_send_output_report(ds, &rp);

  return 0;
}
}  // namespace p9n_tool
