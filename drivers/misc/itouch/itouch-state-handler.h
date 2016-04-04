/*
 *
 * iTouch State Handler
 *
 * Copyright (c) 2014, Intel Corporation.
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

#ifndef _ITOUCH_STATE_HANDLER_H_
#define _ITOUCH_STATE_HANDLER_H_

int sensor_mode_handler(struct itouch_device *idv);
int driver_state_handler(struct itouch_device *idv);
int me_state_handler(struct itouch_device *idv);
int hid_state_handler(struct itouch_device *idv);
int graphics_state_handler(struct itouch_device *idv);

#endif // _ITOUCH_STATE_HANDLER_H_
