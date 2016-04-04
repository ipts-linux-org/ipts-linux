/*
 *
 * Intel Integrated Touch Debugfs Linux Driver
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
 *
 */
#include <linux/debugfs.h>

#include "itouch.h"
#include "itouch-gfx-hid-interface.h"
#include "itouch-state-codes.h"
#include "mei-hid.h"
#include "itouch-state-handler.h"

static ssize_t itouch_dbgfs_submit_kernel(struct file *fp, char __user *ubuf,
					size_t cnt, loff_t *ppos)
{
	void *kernelspec = NULL;
	int size = 0, ret;
	struct itouch_device *idv = fp->private_data;
	const char *kernel_path;

	idv->sensor_data.FrameSize = 0x4000;
	idv->sensor_data.FeedbackSize = 0xA40;
	/* Step 1: Open GPU Context
	* Step 2: Read the kernel spec file
	* Step 3: Parse the spec and setup the kernel
	*/
	ret = itouch_open_gpu(idv);
	if (ret)
	{
		printk("open_gpu failed\n");
		return ret;
	}

	kernel_path = ITOUCH_VENDOR_KERNEL_BLOB;
	size = itouch_gfx_read_kernelspec(idv, kernel_path, &kernelspec);

	if (!size || !kernelspec) {
		itouch_close_gpu(idv);
		printk("size match failed\n");
		return -ENOMEM;
	}

	ret = itouch_gfx_setup_kernel(idv, kernelspec, size);
                if (ret) {
                        printk("setup kernel failed\n");
                        itouch_close_gpu(idv);
                        return -ENOMEM;
                }

	idv->multi_touch.memory_allocation_status = true;
	idv->graphics_state.comp_state = GFX_TOUCH_READY;
/*	Submit workload to GUC multiple times.*/
/*	for(count = 0; count < 10000; count++) {
		printk("submitting the index %d\n",count%4);
		manual_submit();
	}
*/
	return 0;
}

static ssize_t itouch_dbgfs_single(struct file *fp, char __user *ubuf,
                                        size_t cnt, loff_t *ppos)
{
	struct itouch_device *idv = fp->private_data;
	int status = 0;

	printk(KERN_ERR "%s()...called\n", __func__);

	idv->multi_touch.memory_allocation_status = true;
	idv->driver_state.comp_state = DRIVER_HID_MODE_READY;
	idv->vendor_entries.reg_sensor_mode = TOUCH_SENSOR_MODE_HID;
	idv->me_state.comp_state = ME_MEI_RESET_SEND;
	driver_state_handler(idv);
	//schedule_work(&idv->multi_touch_work);

	return status;
}

static ssize_t itouch_dbgfs_multi(struct file *fp, char __user *ubuf,
                                        size_t cnt, loff_t *ppos)
{
	struct itouch_device *idv = fp->private_data;
	int ret = 0;
	printk(KERN_ERR "%s()...called\n", __func__);

	idv->multi_touch.memory_allocation_status = true;
	idv->driver_state.comp_state = DRIVER_RAW_DATA_MODE_READY;
	idv->vendor_entries.reg_sensor_mode = TOUCH_SENSOR_MODE_RAW_DATA;
	idv->me_state.comp_state = ME_MEI_GET_DEVICE_INFO_RECEIVED;
	idv->me_state.comp_state = ME_MEI_RESET_SEND;
	driver_state_handler(idv);
	//schedule_work(&idv->multi_touch_work);

        return ret;
}

static ssize_t itouch_dbgfs_hid_ready(struct file *fp, char __user *ubuf,
                                        size_t cnt, loff_t *ppos)
{
        struct itouch_device *idv = fp->private_data;
        int ret = 0;

        ret = hid_touch_heci2_interface(idv, (MEITOUCH_FEATURE_COMMANDS)TOUCH_SET_TOUCH_READY);
        return ret;
}

static ssize_t itouch_dbgfs_feedback_zero(struct file *fp, char __user *ubuf,
                                        size_t cnt, loff_t *ppos)
{
        struct itouch_device *idv = fp->private_data;
        int ret = 0;

        u32 touch_data_buf_index;
        TOUCH_FEEDBACK_HDR HID2MEBuffer = { 0 };

        touch_data_buf_index = idv->touch_buffer_index;

        HID2MEBuffer.BufferId = touch_data_buf_index;
        HID2MEBuffer.FeedbackCmdType = TOUCH_FEEDBACK_CMD_TYPE_NONE;
        HID2MEBuffer.PayloadSizeBytes = 0;
        HID2MEBuffer.ProtocolVer = 0;
        HID2MEBuffer.Reserved[0] = 0xAC;

        memcpy(idv->touch_buffer[touch_data_buf_index].
                pfeedback_buffer, &HID2MEBuffer,
                sizeof(HID2MEBuffer));
        ret = touch_feedback_ready(idv, (u8) touch_data_buf_index, 0);
        return ret;
}

static const struct file_operations itouch_multi_dbgfs_fops = {
        .open = simple_open,
        .read = itouch_dbgfs_multi,
        .llseek = generic_file_llseek,
};

static const struct file_operations itouch_single_dbgfs_fops = {
        .open = simple_open,
        .read = itouch_dbgfs_single,
        .llseek = generic_file_llseek,
};

static const struct file_operations itouch_feedback_zero_dbgfs_fops = {
        .open = simple_open,
        .read = itouch_dbgfs_feedback_zero,
        .llseek = generic_file_llseek,
};

static const struct file_operations itouch_hid_ready_dbgfs_fops = {
        .open = simple_open,
        .read = itouch_dbgfs_hid_ready,
        .llseek = generic_file_llseek,
};

static const struct file_operations itouch_submit_kernel_dbgfs_fops = {
        .open = simple_open,
        .read = itouch_dbgfs_submit_kernel,
        .llseek = generic_file_llseek,
};

void itouch_dbgfs_deregister(struct itouch_device *dev)
{
	if (!dev->dbgfs_dir)
		return;
	debugfs_remove_recursive(dev->dbgfs_dir);
	dev->dbgfs_dir = NULL;
}

int itouch_dbgfs_register(struct itouch_device *dev, const char *name)
{
	struct dentry *dir, *f;
	dir = debugfs_create_dir(name, NULL);
	if (!dir)
		return -ENOMEM;

        f = debugfs_create_file("multi", S_IRUSR, dir,
                                dev, &itouch_multi_dbgfs_fops);
        if (!f) {
                dev_err(dev->dev, "multi debugfs creation failed\n");
                goto err;
        }

        f = debugfs_create_file("single", S_IRUSR, dir,
                                dev, &itouch_single_dbgfs_fops);
        if (!f) {
                dev_err(dev->dev, "itouch hid_ready dump debugfs creation failed\n");
                goto err;
        }

        f = debugfs_create_file("hid_ready", S_IRUSR, dir,
                                dev, &itouch_hid_ready_dbgfs_fops);
        if (!f) {
                dev_err(dev->dev, "itouch hid_ready dump debugfs creation failed\n");
                goto err;
        }

        f = debugfs_create_file("feedback_zero", S_IRUSR, dir,
                                dev, &itouch_feedback_zero_dbgfs_fops);
        if (!f) {
                dev_err(dev->dev, "itouch hid_ready dump debugfs creation failed\n");
                goto err;
        }

        f = debugfs_create_file("submit_kernel", S_IRUSR, dir,
                                dev, &itouch_submit_kernel_dbgfs_fops);
        if (!f) {
                dev_err(dev->dev, "itouch hid_ready dump debugfs creation failed\n");
                goto err;
	}
	dev->dbgfs_dir = dir;
	return 0;
err:
	itouch_dbgfs_deregister(dev);
	return -ENODEV;
}
