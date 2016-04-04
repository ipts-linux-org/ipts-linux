/*
 * MEI-HID glue driver.
 *
 * Copyright (c) 2012-2016, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include "itouch.h"

#ifndef _MEI_HID_H_
#define	_MEI_HID_H_

#define	BUS_MEI	0x44

#define WRITE_CHANNEL_REPORT_ID 0xa
#define READ_CHANNEL_REPORT_ID 0xb
#define CONFIG_CHANNEL_REPORT_ID 0xd
#define VENDOR_INFO_REPORT_ID 0xF
#define SINGLE_TOUCH_REPORT_ID 0x40

#define HID_GET_DEVICE_DESCRIPTOR 1
#define HID_GET_DEVICE_ATTRIBUTES 2
#define HID_GET_REPORT_DESCRIPTOR 3
#define HID_READ_REPORT           4
#define HID_WRITE_REPORT          5
#define HID_SET_FEATURE           6
#define HID_GET_FEATURE           7

int mei_hid_probe(struct itouch_device *idv);
int mei_hid_remove(struct itouch_device *idv);
int hid_descriptor_read_from_file(struct itouch_device *idv);
#endif /* _MEI_HID_H_ */
