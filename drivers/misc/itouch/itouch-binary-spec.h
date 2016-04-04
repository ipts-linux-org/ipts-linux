/*
 *
 * Intel Integrated Touch Binary Spec Handler
 * Copyright (c) 2016 Intel Corporation.
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

#ifndef _IIT_BINARY_SPEC_H
#define _IIT_BINARY_SPEC_H

#define IITB_HEADER_VERSION 2

#pragma pack(1)

// We support 8 output buffers (1 feedback, 7 hid)
#define  MAX_NUM_OUTPUT_BUFFERS 16

typedef enum {
	IITB_KERNEL,
	IITB_RO_DATA,
	IITB_RW_DATA,
	IITB_SENSOR_FRAME,
	IITB_OUTPUT,
	IITB_DYNAMIC_STATE_HEAP,
	IITB_PATCH_LOCATION_LIST,
	IITB_ALLOCATION_LIST,
	IITB_COMMAND_BUFFER_PACKET,
	IITB_TAG,
} IITB_BUFFER_TYPE;

// Integrated Touch header
struct iitb_header {
	char str[4];
	unsigned int version;

#if IITB_HEADER_VERSION > 1
	unsigned int gfxcore;
	unsigned int revid;
#endif
};

// Integrated Touch allocation descriptor
/*_GfxDesc*/
struct iitb_alloc {
	unsigned int handle;
	unsigned int reserved;
};

struct iitb_alloc_spec {
	unsigned int num;
	struct iitb_alloc alloc[];
};

struct iitb_cmdbuf_spec {
	unsigned int size;
	char data[];
};
/*GfxDesc and GfxDescRecord are the same other than the last
	member element "data". GfxDesc has it while the other does not have it*/
struct iitb_res {
	unsigned int handle;
	IITB_BUFFER_TYPE type;
	unsigned int initialize;
	unsigned int aligned_size;
	unsigned int size;
	char data[];
};

struct iitb_io_header {
	char str[10];
	unsigned short type;
};

struct iitb_res_spec {
	unsigned int num;
	struct iitb_res res[];
};

struct iitb_patch {
	unsigned int index;
	unsigned int reserved1[2];
	unsigned int alloc_offset;
	unsigned int patch_offset;
	unsigned int reserved2;
};

struct iitb_patch_spec {
	unsigned int num;
	struct iitb_patch patch[];
};

struct iitb_guc_wq_spec {
	unsigned int batch_offset;
	unsigned int size;
	char data[];
};

struct iitb_bufid_spec {
	unsigned int imm_offset;
	unsigned int mem_offset;
};

#pragma pack()

#endif //_IIT_BINARY_SPEC_H
