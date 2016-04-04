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
#include "mei-hid.h"
#include "itouch-state-codes.h"

static int hid_get_blob_size(struct file *file)
{
	struct kstat st;
	if (vfs_getattr(&file->f_path, &st))
		return -1;
	if (!S_ISREG(st.mode))
		return -1;
	if (st.size != (int)st.size)
		return -1;
	return st.size;
}

static int itouch_hid_read_file(struct itouch_device *idv, u32 bufferIndex,
			 bool vendorFile)
{
	struct file *f;
	void *buf = NULL;
	mm_segment_t fs;
	int size_read = 0;
	loff_t pos = 0;
	/* Interface with gfx blob test */
	fs = get_fs();
	set_fs(get_ds());
	if (vendorFile){
		f = filp_open(ITOUCH_VENDOR_HID_DESCRIPTOR, O_RDONLY, 0644);
		itouch_dbg(idv, "Loading Panel Vendor HID Descriptor %s\n",
					ITOUCH_VENDOR_HID_DESCRIPTOR);
	} else {
		f = filp_open(ITOUCH_INTEG_KERNEL_HID_DESCRIPTOR, O_RDONLY, 0644);
		itouch_dbg(idv, "Loading Intel Precision Touch HID Descriptor");
	}
	set_fs(fs);

	if (IS_ERR(f) || NULL == f->f_op) {
		itouch_dbg(idv, "HID Descriptor File not found\n");
	} else {
		/* Get the file size first */
		int size = -1;

		size = hid_get_blob_size(f);

		if (size < 0) {
			printk(KERN_ERR "failed to get the blob file size\n");

			/*Fall back to static size */
			size = 1083 * 1024;
		}

		buf = vmalloc(size);
		if (NULL == buf) {
			printk(KERN_ERR "iTouch Blob buf allocation failed\n");
			return 0;
		} else {
			fs = get_fs();
			set_fs(get_ds());
			size_read = __vfs_read( f, buf, size, &pos);
			set_fs(fs);
			filp_close(f, NULL);

			/* If size read is less than expected, fail the call */
			if (size_read < size) {
				printk(KERN_ERR "iTouch Blob read failed\n");
				vfree(buf);
				buf = NULL;
				return 0;
			}
		}
		memcpy((idv->report_descriptor + bufferIndex), buf, size_read);
		vfree(buf);
		buf = NULL;
	}

	return size_read;
}

int hid_descriptor_read_from_file(struct itouch_device *idv)
{
	u16 BufferLength = 0;
	u16 bufferIndex = 0;

	idv->hid_descriptor_size = 0;

	BufferLength = itouch_hid_read_file(idv, bufferIndex, false);
	idv->hid_descriptor_size += BufferLength;

	bufferIndex = BufferLength;
	BufferLength = itouch_hid_read_file(idv, bufferIndex, true);

	idv->hid_descriptor_size += BufferLength;
	return 0;
}

int itouch_blob_read_from_file(struct itouch_device *idv)
{
    return 0;
}

static void mei_hid_request(struct hid_device *hid, struct hid_report *rep,
			    int reqtype)
{
	return;
}

static int mei_hid_parse(struct hid_device *hid)
{
	struct itouch_device *idv = hid->driver_data;
	int ret = 0;

	itouch_dbg(idv, "mei_hid_parse() start\n");
	ret = mei_hid_itouch_get_hid_descriptor(idv);
	if (ret != 0) {
		itouch_dbg(idv, "mei_hid_itouch_get_hid_descriptor ret %d\n", ret);
		return -EIO;
	}

	ret = hid_parse_report(hid, idv->rdesc, idv->hid_descriptor_size);
	if (ret) {
		return ret;
	}

	kfree(idv->rdesc);
	return 0;
}

static int mei_hid_start(struct hid_device *hid)
{
	itouch_dbg(idv, "%s() called.......\n",__func__);
	return 0;
}

static void mei_hid_stop(struct hid_device *hid)
{
	return;
}

static int mei_hid_open(struct hid_device *hid)
{
    itouch_dbg(idv, "%s() called.......\n",__func__);
	return 0;
}

static void mei_hid_close(struct hid_device *hid)
{
	return;
}

static int mei_wait_for_response(struct hid_device *hid)
{
	/*timed_wait_for_timeout(WAIT_FOR_SEND_SLICE, get_report_done, (10 * HZ));

	   if (!get_report_done) {
	   dbg_hid("timeout waiting for heci device\n");
	   return -1;
	   } */

	return 0;
}
static int mei_hid_raw_request(struct hid_device *hid,
                                  unsigned char report_number, __u8 * buf,
                                  size_t count, unsigned char report_type,int reqtype)
{
    itouch_dbg(idv, "mei_hid_raw_request() called...[Empty Function]\n");
	return 0;
}

static int mei_hid_output_report(struct hid_device *hid,
                                     __u8 * buf, size_t count)
{
	itouch_dbg(idv, "mei_hid_output_report() called...[Empty Function]\n");
    return 0;
}

static struct hid_ll_driver mei_hid_ll_driver = {
	.parse = mei_hid_parse,
	.start = mei_hid_start,
	.stop = mei_hid_stop,
	.open = mei_hid_open,
	.close = mei_hid_close,
	.request = mei_hid_request,
	.wait = mei_wait_for_response,
    .raw_request = mei_hid_raw_request,
    .output_report = mei_hid_output_report,
};

int mei_hid_probe(struct itouch_device *idv)
{
	int ret = 0;
	struct hid_device *hid;

	hid = hid_allocate_device();
	if (IS_ERR(hid)) {
		ret = PTR_ERR(hid);
		goto err_dev;
	}

	hid->driver_data = idv;
	hid->ll_driver = &mei_hid_ll_driver;
	hid->dev.parent = &idv->mdv->dev;
	hid->bus = BUS_MEI;
	hid->version = idv->sensor_data.FwRev;
	hid->vendor = idv->sensor_data.VendorId;
	hid->product = idv->sensor_data.DeviceId;
	snprintf(hid->phys, sizeof(hid->phys), "heci3");
	snprintf(hid->name, sizeof(hid->name),
		 "%s %04hX:%04hX", "itouch", hid->vendor, hid->product);

	idv->device_touch_data = kzalloc(sizeof(hid_loopback_report), GFP_KERNEL);
	idv->hid = hid;
	ret = hid_add_device(hid);
	if (ret) {
		if (ret != -ENODEV)
			dev_err(&hid->dev, "can't add hid device: %d\n", ret);
		goto err_mem_free;
	}
#ifdef MULTI_TOUCH_DEFAULT
	if( (hid->group == HID_GROUP_MULTITOUCH) ||
		(hid->group == HID_GROUP_MULTITOUCH_WIN_8)) {
#else
	if(0) { /*this will drop to single touch*/
#endif
		itouch_dbg(idv, "Detected a multi-touch device\n");
		idv->vendor_entries.reg_sensor_mode = TOUCH_SENSOR_MODE_RAW_DATA;
		idv->hid_state.comp_state = HID_MULTI_TOUCH_READY;
	} else {
		itouch_dbg(idv, "Detected a non-multitouch device, group=0x%x\n", hid->group);
		idv->vendor_entries.reg_sensor_mode = TOUCH_SENSOR_MODE_HID;
		idv->hid_state.comp_state = HID_SINGLE_TOUCH_READY;
	}

	idv->driver_state.comp_state = DRIVER_HW_INIT;
	idv->graphics_state.comp_state = GFX_DETECTED;
	idv->me_state.comp_state = ME_MEI_INT_READY;

	return 0;

err_mem_free:
	hid_destroy_device(hid);
	idv->hid = NULL;
	idv->driver_state.comp_state = 0;
	idv->graphics_state.comp_state = GFX_STATE_NONE;
	idv->vendor_entries.reg_sensor_mode = TOUCH_SENSOR_MODE_MAX;
	idv->me_state.comp_state = ME_STATE_NONE;
	idv->hid_state.comp_state = 0;

err_dev:
	idv->hid = NULL;
	return ret;
}

int mei_hid_remove(struct itouch_device *idv)
{
	//struct hid_device *hid = idv->hid;
	//hid_destroy_device(hid);

	return 0;
}
