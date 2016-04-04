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

#include "mei-hid.h"
#include "itouch-state-codes.h"
#include "itouch-state-handler.h"

#define USE_HARDCODED_HID_DEVICE_DESCRIPTOR

/* This is the default HID descriptor of the IntelTestDescriptor
 * The Panel Vendor HID Descriptor will be appended to this
 * descriptor to create a full HID descriptor
 */
struct hid_descriptor G_DefaultHidDescriptor = {
	.bLength		= 0x09,			// length of HID descriptor
	.bDescriptorType	= 0x21,			// descriptor type == HID  0x21
	.bcdHID			= 0x0100,		// hid spec release
	.bCountryCode		= 0x00,			// country code == Not Specified
	.bNumDescriptors	= 0x01,			// number of HID class descriptors
	.desc[0].bDescriptorType	= 0x22,	// descriptor type
	.desc[0].wDescriptorLength	= 0,	// total length of report descriptor.
											//that will be obtained dynamically
};

int mei_hid_itouch_get_hid_descriptor(struct itouch_device *idv)
{
	size_t bytesToCopy = 0;

	hid_descriptor_read_from_file(idv);
	if (idv->hid_descriptor_size < 1) {
		itouch_dbg(idv, "Error in getting any HID Descriptor");
		return -ENOENT;
	}

	bytesToCopy = G_DefaultHidDescriptor.bLength;
	G_DefaultHidDescriptor.desc[0].wDescriptorLength =
	    idv->hid_descriptor_size;

	idv->rdesc = kzalloc((idv->hid_descriptor_size+100), GFP_ATOMIC);
	if (!idv->rdesc) {
		itouch_dbg(idv, "couldn't allocate rdesc memory\n");
		return -ENOMEM;
	}

	memcpy(idv->rdesc, idv->report_descriptor, idv->hid_descriptor_size);
	return 0;
}

int int_touch_complete_read_report(struct itouch_device *idv,
				   phid_loopback_report rep)
{
	int status = 0;
	phid_loopback_report HidinputReport = NULL;
	HidinputReport = rep;
	if (!HidinputReport) {
		status = -1;
		itouch_dbg(idv, "no hid report \n");
	}

	if (HidinputReport->collection_size <= 0) {
		status = -1;
		itouch_dbg(idv, "LoopBackcollection_size is not valid \n");
		return status;
	}
//	itouch_info(idv, "hid report collection size %u\n",
//		   HidinputReport->collection_size);

	status = hid_input_report(idv->hid, HID_INPUT_REPORT, (u8*)HidinputReport,
			 HidinputReport->collection_size, 1);
	return status;
}

//TODO: function implementation pending
int int_touch_complete_hid_read_report_request(struct itouch_device *idv,
					       int action_status,
					       bool get_feature_comp)
{
	int status = 0;

	itouch_dbg(idv,
		   "int_touch_complete_hid_read_report_request Error Status received is 0x%x \n",
		   action_status);

	return status;
}
