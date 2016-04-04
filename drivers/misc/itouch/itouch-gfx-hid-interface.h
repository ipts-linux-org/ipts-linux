/*
 * Intel Integrated Touch GFX-HID Interface
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


#ifndef _ITOUCH_GFX_HID_INTERFACE_H_
#define _ITOUCH_GFX_HID_INTERFACE_H_

#define DOORBELL_COOKIE_ADDRESS_OFFSET 4

typedef enum _OUTPUT_BUFFER_PAYLOAD_TYPE {
	OUTPUT_BUFFER_PAYLOAD_ERROR = 0,	// Error
	OUTPUT_BUFFER_PAYLOAD_HID_INPUT_REPORT,	// HID Input Report
	OUTPUT_BUFFER_PAYLOAD_HID_FEATURE_REPORT,	// Feature Report
	OUTPUT_BUFFER_PAYLOAD_KERNEL_LOAD,	// Load the new Kernel
	OUTPUT_BUFFER_PAYLOAD_FEEDBACK_BUFFER	// Feedback buffer to be sent to ME
} OUTPUT_BUFFER_PAYLOAD_TYPE;

int itouch_open_gpu(struct itouch_device *idv);
int itouch_close_gpu(struct itouch_device *idv);
struct i915_itouch_mapbuffer * itouch_map_buffer(unsigned int size, u32 flags);
void itouch_unmap_buffer(struct i915_itouch_mapbuffer *buf);
int itouch_send_buffer(struct i915_itouch_mapbuffer *buf, unsigned int id);
int itouch_get_guc_wq(struct i915_itouch_mapbuffer *buf, unsigned int wi_size);
int itouch_gfx_setup_kernel(struct itouch_device *idv,
			    void *buf, unsigned int len);
int itouch_gfx_read_kernelspec(struct itouch_device *idv,
				const char *kernel_path, void **kernelspec);
int itouch_gfx_device_destroy(struct itouch_device *idv);

void itouch_signal_complete(HANDLE ctx, u32 id);
void itouch_display_on_off(HANDLE ctx, bool enable);
void itouch_reacquire_guc_db(void);
void manual_submit(void);

#define ITOUCH_HID_NUM_PARALLEL_WORKLOAD	16
#define ITOUCH_HID_MAX_OUTPUT_BUFFERS		16

/* TBD: Static definition as now. 200 should be sufficient for
 *      4 parallel workloads
 */
#define ITOUCH_HID_MAX_NUM_USER_BUFFERS 50
//#define ITOUCH_HID_CMD_BUFFER_IDX  (ITOUCH_HID_MAX_NUM_USER_BUFFERS)
#define ITOUCH_HID_MAX_NUM_BUFFERS ((ITOUCH_HID_MAX_NUM_USER_BUFFERS + 1) * \
			ITOUCH_HID_NUM_PARALLEL_WORKLOAD)

struct itouch_gfx_user_workload {
	unsigned int sensor_frame_idx;
	unsigned int cmd_buf_idx;
	unsigned int out_buf_idx[ITOUCH_HID_MAX_OUTPUT_BUFFERS];
	unsigned int calib_buf_index;
	unsigned int config_buf_index;

};

struct itouch_gfx_alloc_map {
	unsigned int allocations_num;
	unsigned int output_buf_num;
	unsigned int last_id;

	struct {
		unsigned int handle;
		struct i915_itouch_mapbuffer *buf;
	} buffers[ITOUCH_HID_MAX_NUM_BUFFERS];

	struct i915_itouch_mapbuffer *buf_lastsubmit;
};

struct itouch_gfx_ctx {
	struct i915_itouch_ops gfx_ops;
	HANDLE gfx_ctx;
	struct i915_itouch_register_args clbks;
	void *dev;

	/*GUC specific variables from here */
	u64 gpu_handle;
	void *guc_appprocess;
	unsigned int workitem_size;

};

void print_output(unsigned int id);

#endif // _ITOUCH_GFX_HID_INTERFACE_H_
