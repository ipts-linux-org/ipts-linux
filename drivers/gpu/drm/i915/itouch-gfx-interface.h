/*
 *
 * Intel Management Engine Interface (Intel MEI) Integrated Touch Gfx Interface
 * Copyright (c) 2014 Intel Corporation.
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

#ifndef _ITOUCH_GFX_INTERFACE_H_
#define _ITOUCH_GFX_INTERFACE_H_

#define ITOUCH_INTERFACE_VERSION 9
#define ITOUCH_ALLOC_CONTIGUOUS	0x1

struct i915_itouch_opengpu {
	u8 engines_used;
	u32 priority_requested;
	u32 flags;
	u32 checkcode;
	uint64_t process_desc;
	u32 process_desc_size_bytes;
	u32 priority_assigned;
	uint64_t gpu_handle;
	u32 guc_status;

	/*TBD: Are these really required?? */
	struct page *db_page;
	uint64_t db_phy_addr;
	uint64_t tail_phy_addr;
};

struct i915_itouch_closegpu {
	uint64_t process_desc;
	uint64_t gpu_handle;
	u32 checkcode;
	u32 guc_status;
};

struct i915_itouch_mapbuffer {
	uint64_t	contextid;
	u32		size;
	uint64_t	buf_handle;
	void		*gfx_addr;
	void		*cpu_addr;
	u32		flags;
	uint64_t	cpu_phy_addr;
};

struct i915_itouch_ops {

	int (*open_gpu)(uint64_t gfx_handle,
			struct i915_itouch_opengpu *opengpu);

	int (*close_gpu)(uint64_t gfx_handle,
			struct i915_itouch_closegpu *closegpu);

	int (*map_buffer)(uint64_t gfx_handle,
			struct i915_itouch_mapbuffer *mapbuffer);

	int (*unmap_buffer)(uint64_t gfx_handle, uint64_t buf_handle);

	int (*signal_unload)(uint64_t gfx_handle);

	int (*submit_kernel)(uint64_t gfx_handle, uint64_t batch_buf_handle,
			u32 size, u32 id);
};

struct hid_itouch_ops {
	void (*signal_complete)(uint64_t hid_handle, u32 id);
	void (*display_on_off)(uint64_t hid_handle, bool enable);
};


struct i915_itouch_register_args {

	/* Input: Version of header used by HID */
	u32 hid_version;

	/* Output: Version of header used by GFX */
	u32 gfx_version;

	/* Input: Passed from Hid to Gfx */
	struct hid_itouch_ops hid_ops;

	/* Input: handle to HID context */
	uint64_t hid_handle;

	/* Output: Passed back from Gfx to Hid */
	struct i915_itouch_ops i915_ops;

	/* Output: handle to graphics context */
	uint64_t gfx_handle;
};

extern int i915_itouch_client_register(struct i915_itouch_register_args *args);
extern void i915_print_itouch_guc_info(struct guc_process_desc *p_desc);
extern void i915_itouch_reacquire_db(uint64_t gfx_handle);

#endif //_ITOUCH_GFX_INTERFACE_H_
