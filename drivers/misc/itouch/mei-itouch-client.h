/*
 * MEI client driver for iTouch
 *
 * Copyright (c) 2016, Intel Corporation.
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

#ifndef _TOUCH_MEI_CLIENT_H
#define _TOUCH_MEI_CLIENT_H

#include "mei-hid.h"

extern int mei_itouch_recv(struct mei_cl_device *cldev, u8 *buf, size_t length);
extern int mei_itouch_send(struct mei_cl_device *cldev, u8 *buf, size_t length);

int itouch_mei_cl_send(struct mei_cl_device *cl, u8 * buf, size_t length,
		       bool blocking);
int itouch_mei_cl_recv(struct mei_cl_device *cl, u8 * buf, size_t length);

#endif //_TOUCH_MEI_CLIENT_H
