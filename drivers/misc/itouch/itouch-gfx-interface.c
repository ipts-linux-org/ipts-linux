/*
 *
 * Intel Management Engine Interface (Intel MEI) Integrated Touch Gfx Interface
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

#include "itouch.h"
#include "itouch-gfx-interface.h"
#include "itouch-gfx-hid-interface.h"

#define IIT_GFX_MINOR   MISC_DYNAMIC_MINOR
void *host_dma_buf;
unsigned host_dma_buf_size = (1024 * 1024);
uint64_t host_dma_buf_phys;

struct inode;			/* make gcc happy */

static struct itouch_gfx_user_workload
    g_workload[ITOUCH_HID_NUM_PARALLEL_WORKLOAD];
static struct itouch_gfx_alloc_map g_allocmap;

int itouch_gfx_read_allocation_handles(struct itouch_device *idv,
			char *ptr, unsigned int size, unsigned int *parsed)
{
	unsigned int i, j;
	struct iitb_alloc_spec *alloc =
	    (struct iitb_alloc_spec *)(&(ptr[*parsed]));

	if (sizeof(alloc->num) > (size - *parsed)) {
		return -EFAULT;
	}

	if (alloc->num > ITOUCH_HID_MAX_NUM_USER_BUFFERS) {
		itouch_dbg(idv, "Too many allocations %u\n", alloc->num);
		return -EFAULT;
	}

	g_allocmap.allocations_num = alloc->num;
	*parsed += sizeof(alloc->num);
	for (i = 0; i < alloc->num; i++) {
		if (sizeof(alloc->alloc[0]) > (size - *parsed)) {
			return -EFAULT;
		}

		for (j = 0; j < ITOUCH_HID_NUM_PARALLEL_WORKLOAD; j++) {

			g_allocmap.buffers[i + (j * alloc->num)].handle =
			    alloc->alloc[i].handle;

		}
		*parsed += sizeof(alloc->alloc[0]);
	}

	itouch_info(idv, "Total Allocation Handles to be parsed are 0x%x\n",
			g_allocmap.allocations_num);

	return 0;
}

int itouch_gfx_read_cmd_buffer(struct itouch_device *idv,
			char *ptr, unsigned int size, unsigned int *parsed)
{
	struct iitb_cmdbuf_spec *cmd =
	    (struct iitb_cmdbuf_spec *)(&(ptr[*parsed]));
	struct i915_itouch_mapbuffer *buf;
	unsigned int offset = 0;
	int i;

	if (sizeof(cmd->size) > (size - *parsed)) {
		return -EFAULT;
	}

	*parsed += sizeof(cmd->size);
	if (cmd->size > (size - *parsed)) {
		return -EFAULT;
	}
	*parsed += cmd->size;

	offset = ITOUCH_HID_NUM_PARALLEL_WORKLOAD * g_allocmap.allocations_num;
	for (i = 0; i < ITOUCH_HID_NUM_PARALLEL_WORKLOAD; i++) {

		buf = itouch_map_buffer(cmd->size, 0);
		if (!buf) {
			return -ENOMEM;
		}

		itouch_info(idv, "Batch buffer GPU address = 0x%08x CPU:0x%08x\n",
					(u32)(unsigned long)buf->gfx_addr,
					(u32)(unsigned long)buf->cpu_addr);

		memcpy((void *)buf->cpu_addr, &(cmd->data[0]), cmd->size);

		g_allocmap.buffers[offset].buf = buf;

		g_workload[i].cmd_buf_idx = offset;

		offset++;
	}
	return 0;
}

int itouch_gfx_read_res_list(struct itouch_device *idv, char *ptr,
			     unsigned int size, unsigned int *parsed)
{
	struct iitb_res_spec *spec = (struct iitb_res_spec *)(&(ptr[*parsed]));
	struct iitb_res *res = &(spec->res[0]);
	int out_buf_idx = 0;
	int i, j;
	unsigned int index = 0;
	const char *kernel_cfg_path;
	void *vendor_config= NULL;
	unsigned int vendor_config_size;

	if (sizeof(spec->num) > (size - *parsed)) {
		return -EFAULT;
	}

	*parsed += sizeof(spec->num);
	itouch_dbg(idv, "Total number of resources to be parsed is %u\n",
				spec->num);

	for (i = 0; i < spec->num; i++) {

		bool initialize, is_raw = false, is_out = false;
		bool is_cal_buf=false,is_config_buf=false;

		res = (struct iitb_res *)(&(ptr[*parsed]));
		if (sizeof(res[0]) > (size - *parsed)) {
			return -EFAULT;
		}

		itouch_info(idv, "Resource %u: handle 0x%08x type %u init %u size %u\n",
		       i, res->handle, res->type, res->initialize, res->size);
		*parsed += sizeof(res[0]);

		if (res->initialize) {
			if (res->size > (size - *parsed)) {
				return -EFAULT;
			}
			*parsed += res->size;
		}

		initialize = res->initialize;

		if (initialize && res->size > sizeof(struct iitb_io_header)) {
			struct iitb_io_header *io_hdr;
			io_hdr = (struct iitb_io_header *)(&res->data[0]);
			if (io_hdr->str[0] == 'I' &&
			    io_hdr->str[1] == 'N' &&
			    io_hdr->str[2] == 'T' &&
			    io_hdr->str[3] == 'E' &&
			    io_hdr->str[4] == 'L' &&
			    io_hdr->str[5] == 'T' &&
			    io_hdr->str[6] == 'O' &&
			    io_hdr->str[7] == 'U' &&
			    io_hdr->str[8] == 'C' && io_hdr->str[9] == 'H') {
				// We support 1 and only 1 sensor frame buffer,
				// and up to MAX_NUM_OUTPUT_BUFFERS output buffers
				switch (io_hdr->type) {
				case 0:
					is_raw = true;
					break;
				case 2:
					is_config_buf = true;
					break;
				case 3:
					is_cal_buf = true;
					break;
				case 1:
					is_out = true;
					out_buf_idx++;
					break;
				default:
					itouch_dbg(idv,
						"Invalid Buffer type specified in OCL kernel %d\n",
						io_hdr->type);
					break;
				}

				// Don't initialize IO buffers with the io_hdr
				if (!is_cal_buf)
					initialize = false;
			}
		}

		if (res->aligned_size) {

			struct i915_itouch_mapbuffer *buf;
			u32 flags = 0;
			int k = 0;

			for (k = 0; k < g_allocmap.allocations_num; k++) {
				if (g_allocmap.buffers[k].handle == res->handle)
					break;
			}

			if (k == g_allocmap.allocations_num)
				itouch_dbg(idv,
				       "Couldnt match the handle 0x%x in res list\n",
						k);

			/* Is this Index already allocated ?? */
			if (g_allocmap.buffers[k].buf != NULL) {
				itouch_info(idv,
				    "Duplicate entry for resource index %d\n",
				     k);
				continue;
			}

			for (j = 0; j < ITOUCH_HID_NUM_PARALLEL_WORKLOAD; j++) {
				int buf_size = 0;

				/* Get the index for the corresponding paralles workload set */
				index = k + (j * g_allocmap.allocations_num);
				buf_size = res->aligned_size;

				if (is_raw) {

					flags = ITOUCH_ALLOC_CONTIGUOUS;

					g_workload[j].sensor_frame_idx = index;
					buf_size =
					    (res->aligned_size > idv->sensor_data.FrameSize) ?
					     res->aligned_size : idv->sensor_data.FrameSize;
				} else if (is_out) {

					flags = ITOUCH_ALLOC_CONTIGUOUS;
					buf_size =
					    (res->aligned_size > idv->sensor_data.FeedbackSize) ?
					     res->aligned_size : idv->sensor_data.FeedbackSize;

					g_workload[j].out_buf_idx[out_buf_idx - 1] = index;

				} else if (is_cal_buf) {

					flags = ITOUCH_ALLOC_CONTIGUOUS;
					g_workload[j].calib_buf_index = index;

				} else if (is_config_buf) {

					flags = ITOUCH_ALLOC_CONTIGUOUS;
                    //TODO: There are more than one config buffers hence//
                    //reading the config file needs to be done once
                    //and content can be memcpy()-ed

					kernel_cfg_path = ITOUCH_VENDOR_KERNEL_CFG;
					vendor_config_size = itouch_gfx_read_kernelspec(idv,
								kernel_cfg_path, &vendor_config);

					g_workload[j].config_buf_index = index;
					if (vendor_config_size > 0)
					{
						//TODO Error handling when buffer not mapped.
						//This is not an error condition, so issue warning.
						//Now returning error value without cleanup or unmapping
						buf = itouch_map_buffer(buf_size, flags);
						if (!buf)
							return -ENOMEM;
						g_allocmap.buffers[index].buf = buf;

						memcpy((void *)g_allocmap.buffers[index].buf->cpu_addr, vendor_config, vendor_config_size);
						continue;
					}
					else{
						itouch_dbg(idv,
							"Error processing Vendor configuration, continuing");
						continue;
					}
				}else {

					/* Neither input nor output. So we dont really need new
					 * parallel resource other than one. So, just allocate one
					 */
					if (j != 0)
						continue;
				}

				buf = itouch_map_buffer(buf_size, flags);
				if (!buf) {
					//TODO cleanup!
					itouch_dbg(idv, "Failed to map buffer\n");
					return -ENOMEM;
				}

				g_allocmap.buffers[index].buf = buf;

				if (initialize) {

					if (res->data[0] == 0xcdcdcdcd) {

						memset((void *)buf->cpu_addr, 0x0, buf_size);
					} else {

						memcpy((void *)buf->cpu_addr, &(res->data[0]), res->size);
#if 0
						if (is_raw) {
							int iter1 = 0;
						printk(KERN_ERR
						       "Dumping Input Raw buffer read from blob\n");

						for (iter1 = 0;
						     iter1 < (res->size / 4);
						     iter1++) {
							printk(" 0x%08x",
							       *((u32 *) buf->
								 cpu_addr +
								 iter1));
						}
						printk("\n");
						}
#endif
					}
				} else {
					memset((void *)buf->cpu_addr, 0x0, buf_size);
				}
			}//end for (j = 0...
		}
	}

	g_allocmap.output_buf_num = out_buf_idx;

	return 0;
}

int itouch_gfx_read_patch_list(struct itouch_device *idv,
			char *ptr, unsigned int size, unsigned int *parsed)
{
	struct iitb_patch_spec *spec =
	    (struct iitb_patch_spec *)(&(ptr[*parsed]));
	struct i915_itouch_mapbuffer *cmd = NULL;
	unsigned int cmd_index = 0, buf_index = 0;
	unsigned int gtt_offset;
	char *batch = NULL;
	int i, j;

	if (sizeof(spec->num) > (size - *parsed)) {
		return -EFAULT;
	}
	*parsed += sizeof(spec->num);

	for (i = 0; i < spec->num; i++) {

		if (sizeof(spec->patch[0]) > (size - *parsed)) {
			return -EFAULT;
		}

		/*itouch_info(idv, "Entry %u: idx %u allocation offset %u patch offset %u\n",
		   i, spec->patch[i].index, spec->patch[i].alloc_offset,
		   spec->patch[i].patch_offset);*/

		for (j = 0; j < ITOUCH_HID_NUM_PARALLEL_WORKLOAD; j++) {
			cmd_index = g_workload[j].cmd_buf_idx;
			buf_index =
			    (spec->patch[i].index +
			     j * g_allocmap.allocations_num);

			if (NULL == g_allocmap.buffers[buf_index].buf) {
				/* Not input or output parallel buffer. Use the
				 * single buffer itself
				 */
				buf_index = spec->patch[i].index;
			}

			cmd = g_allocmap.buffers[cmd_index].buf;
			batch = (char *)(unsigned long)cmd->cpu_addr;

			if(NULL != g_allocmap.buffers[buf_index].buf) {
				gtt_offset =
					(u32) (unsigned long)g_allocmap.buffers[buf_index].
					buf->gfx_addr;
				gtt_offset += spec->patch[i].alloc_offset;
			}
			else{
				/*When an patch index is not pointing to valid resource
				* entry, set the patch address to 0*/
				gtt_offset = 0;
				gtt_offset += spec->patch[i].alloc_offset;
			}

			batch += spec->patch[i].patch_offset;
			*(unsigned int *)batch = gtt_offset;
		}

		*parsed += sizeof(spec->patch[0]);
	}
	return 0;
}

#pragma pack(1)
union isa_instr_header {
	unsigned int value;
	struct {
		unsigned int length:8;	//  7..0
		unsigned int reserved:8;	//  8..15
		unsigned int subopcode:8;	// 16..23
		unsigned int opcode:3;	// 24..26
		unsigned int subtype:2;	// 27..28
		unsigned int type:3;	// 29..31
	} D3;
	struct {
		unsigned int length:8;	//  7..0
		unsigned int reserved:15;	//  8..22
		unsigned int opcode:6;	// 23..28
		unsigned int type:3;	// 29..31
	} MI;
};
#pragma pack()

int itouch_gfx_apply_fixes(void)
{
	struct i915_itouch_mapbuffer *cmd = NULL;
	volatile unsigned int *batch;
	int size = 0;
	int j = 0;
	unsigned int cmd_index = 0;

	for (j = 0; j < ITOUCH_HID_NUM_PARALLEL_WORKLOAD; j++) {
		cmd_index = g_workload[j].cmd_buf_idx;
		cmd = g_allocmap.buffers[cmd_index].buf;

		size = cmd->size;

		batch = (volatile unsigned int *)cmd->cpu_addr;

		while (size > 4) {

			if (*batch == 0x6101000E) {
				unsigned int ss_gtt =
				    (u32) (unsigned long)cmd->gfx_addr;

				/* Surface heap is at 16KB offset from BB start */
#if IITB_HEADER_VERSION > 1
				/* BDW SS offset is at 4 */
				batch[4] = ss_gtt + ((16 * 1024) | 1);
#else
				/* HSW SS offset is at 2 */
				batch[2] = ss_gtt + ((16 * 1024) | 1);
#endif
			}
			size -= 4;
			batch++;
		}
	}

	return 0;
}

void itouch_dump_batch_buffer(void)
{
	struct i915_itouch_mapbuffer *cmd = NULL;
	volatile unsigned int *batch;
	int size = 0;
	int j = 0;
	unsigned int cmd_index = 0;

	cmd_index = g_workload[5].cmd_buf_idx;
	cmd = g_allocmap.buffers[cmd_index].buf;

	size = cmd->size;

	batch = (volatile unsigned int *)cmd->cpu_addr;

	printk(KERN_INFO "START----Dumping Batch Buffe --> Start\n");

	while (size > 4) {

		printk("0x%08x ", *batch);

		/* Adding a new line after every 8 U32*/
		if ((++j % 8) == 0)
			printk("\n ");

		size -= 4;
		batch++;
	}

	printk("Dumping Batch Buffe --> END\n");

	return;
}


int itouch_gfx_read_guc_workqueue(struct itouch_device *idv,
			char *ptr, unsigned int size, unsigned int *parsed)
{
	struct iitb_guc_wq_spec *cmd =
	    (struct iitb_guc_wq_spec *)(&(ptr[*parsed]));
	struct i915_itouch_mapbuffer buf, *cmd_buf;
	int guc_wq_size = 0, i, j;
	int data_offset = sizeof(cmd->size) + sizeof(cmd->batch_offset);
	char *pWQAddr = NULL;
	unsigned int cmd_index = 0;

	if (data_offset > (size - *parsed)) {
		itouch_dbg(idv,
				"GUC WQ command size not right from blob = %zu, remaining size = %d\n",
				sizeof(cmd->size), (size - *parsed));
		return -EFAULT;
	}

	*parsed += data_offset;
	if (cmd->size > (size - *parsed)) {
		itouch_dbg(idv,
				"GUC WQ data size not right from blob = %d, remaining size = %d\n",
				cmd->size, (size - *parsed));
		return -EFAULT;
	}
	guc_wq_size = itouch_get_guc_wq(&buf, cmd->size);
	if (!guc_wq_size) {
		itouch_dbg(idv,
				"GUC WQ size is zero. Can happen if GUC is not enabled\n");
		return -EFAULT;
	}

	itouch_info(idv,
			"Size of GUC WQ item = %u bytes, batch offset = %d, WI data offset = %d\n",
			cmd->size, cmd->batch_offset, data_offset);

	*parsed += cmd->size;

	if (guc_wq_size < cmd->size) {
		itouch_dbg(idv,
			"GUC WQ allocated size = %d less than size in blob which is %d\n",
			guc_wq_size, cmd->size);
		return -EFAULT;
	}
/*
	//Debug code
	for (i = 0; i < (cmd->size) / 4; i++) {
		printk(KERN_ERR "Work item DWORD %d = 0x%08x\n", i,
		       *((u32 *) & cmd->data[0] + i));
	}
*/
	/* Get the GUC WQ Address */
	pWQAddr = (char *)(buf.cpu_addr);

	/* Repeatedly copy the WQ data into allocated GUC WQ */
	for (i = 0;
	     i < (guc_wq_size / (cmd->size * ITOUCH_HID_NUM_PARALLEL_WORKLOAD));
	     i++) {

		for (j = 0; j < ITOUCH_HID_NUM_PARALLEL_WORKLOAD; j++) {
			cmd_index = g_workload[j].cmd_buf_idx;
			cmd_buf = g_allocmap.buffers[cmd_index].buf;

			/* Patch the WQ Data with proper batch buffer offset */
			*(u32 *) (&(cmd->data[0]) + cmd->batch_offset) = (u32) (unsigned long)(cmd_buf->gfx_addr);

			memcpy(pWQAddr, &(cmd->data[0]), cmd->size);

			pWQAddr += cmd->size;
		}
	}

/* //Debug code
	pWQAddr = (char *)(buf.cpu_addr);
	for (i = 0; i < (cmd->size)/4; i++) {
		printk(KERN_ERR "Work item Added %d = 0x%08x\n", i,
				*((u32 *)pWQAddr + i));
	}
*/
	return 0;
}

int itouch_gfx_setup_lastsubmit_buffer(struct itouch_device *idv,
			char *ptr, unsigned int size, unsigned int *parsed)
{
	int j = 0;
	struct i915_itouch_mapbuffer *buf, *cmd_buf;
	unsigned int cmd_index = 0;
	char *batch;
	struct iitb_bufid_spec *cmd =
	    (struct iitb_bufid_spec *)(&(ptr[*parsed]));
	unsigned int mem_offset = 0, imm_offset = 0;

	if (sizeof(struct iitb_bufid_spec) > (size - *parsed)) {
		itouch_dbg(idv,
		       "iitb_bufid size not right from blob = %zu, remaining size = %d\n",
		       sizeof(struct iitb_bufid_spec), (size - *parsed));
		return -EFAULT;
	}

	*parsed += sizeof(struct iitb_bufid_spec);

	mem_offset = cmd->mem_offset;
	imm_offset = cmd->imm_offset;

	/* Alloc a buffer of 1 page size */
	buf = itouch_map_buffer(PAGE_SIZE, 0);
	if (!buf) {
		//TODO cleanup!
		return -ENOMEM;
	}

	g_allocmap.buf_lastsubmit = buf;

	/* Initialize the buffer with default value */
	*((u32 *) buf->cpu_addr) = LASTSUBMITID_DEFAULT_VALUE;

	/* Patch the batch buffer with proper index and address */
	for (j = 0; j < ITOUCH_HID_NUM_PARALLEL_WORKLOAD; j++) {
		cmd_index = g_workload[j].cmd_buf_idx;
		cmd_buf = g_allocmap.buffers[cmd_index].buf;
		batch = (char *)(unsigned long)cmd_buf->cpu_addr;

		*((unsigned int *)(batch + mem_offset)) = (u32) (unsigned long)(buf->gfx_addr);
		*((unsigned int *)(batch + imm_offset)) = j;
	}

	return 0;
}

void print_output(unsigned int id)
{
	g_allocmap.last_id = id;
	return;
}
/*
 * This routine will read the kernel spec file and setup the required
 * surfaces. These allocations include batchbuffer and GUC WorkQueue
*/
int itouch_gfx_setup_kernel(struct itouch_device *idv,
			    void *buf, unsigned int len)
{
	struct iitb_header *hdr;
	unsigned int parsed = 0;
	int i = 0, j = 0;
	int ret;

	// check header version and contents
	hdr = (struct iitb_header *)buf;

	if (hdr->version != IITB_HEADER_VERSION ||
	    hdr->str[0] != 'I' ||
	    hdr->str[1] != 'O' || hdr->str[2] != 'C' || hdr->str[3] != 'L') {
		itouch_dbg(idv,
		       "Binary spec header is not correct: version = %d, string = %s\n",
		       hdr->version, hdr->str);
		return -1;
	}

	parsed += sizeof(struct iitb_header);
	memset(&g_workload, 0,
	       ITOUCH_HID_NUM_PARALLEL_WORKLOAD *
	       sizeof(struct itouch_gfx_user_workload));
	memset(&g_allocmap, 0, sizeof(struct itouch_gfx_alloc_map));

	itouch_dbg(idv, "Setting up GUC workspace as per Vendor kernel spec\n");
	ret = itouch_gfx_read_allocation_handles(idv, (char *)buf, len, &parsed);
	if (ret)
		return ret;

	ret = itouch_gfx_read_cmd_buffer(idv, (char *)buf, len, &parsed);
	if (ret)
		return ret;

	ret = itouch_gfx_apply_fixes();
	if (ret)
		return ret;

	ret = itouch_gfx_read_res_list(idv, (char *)buf, len, &parsed);
	if (ret)
		return ret;

	ret = itouch_gfx_read_patch_list(idv, (char *)buf, len, &parsed);
	if (ret)
		return ret;

	ret = itouch_gfx_read_guc_workqueue(idv, (char *)buf, len, &parsed);
	if (ret)
		return ret;

	ret = itouch_gfx_setup_lastsubmit_buffer(idv, (char *)buf, len, &parsed);
	if (ret)
		return ret;

	/* Dump the batch buffer */
//	itouch_dump_batch_buffer();
	itouch_dbg(idv, "Assigining itouch buffers\n");

	 /* Save the details in idv for future reference */
	idv->current_buffer_index = LASTSUBMITID_DEFAULT_VALUE;
	idv->last_buffer_completed = LASTSUBMITID_DEFAULT_VALUE;
	idv->plast_submitted_id = g_allocmap.buf_lastsubmit->cpu_addr;

	idv->num_output_buffers = g_allocmap.output_buf_num;

	for (i = 0; i < HID_PARALLEL_DATA_BUFFERS; i++) {

		u32 sensor_index = g_workload[i].sensor_frame_idx;

		/* Save the raw input buffer address */
		idv->touch_buffer[i].raw_touch_data_buffer =
		    (char *)g_allocmap.buffers[sensor_index].buf->cpu_addr;

		idv->touch_buffer[i].phy_raw_touch_data_buffer = g_allocmap.buffers[sensor_index].buf->cpu_phy_addr;
		/*itouch_infoi(idv, "itouch = %8x \n",
			   idv->touch_buffer[i].phy_raw_touch_data_buffer);*/

		/* Save the output buffer details */
		for (j = 0; j < g_allocmap.output_buf_num; j++) {
			u32 out_index = g_workload[i].out_buf_idx[j];
			idv->touch_buffer[i].processed_touch_buffer[j] =
			    (char *)(g_allocmap.buffers[out_index].buf->cpu_addr);
		}
	}

	/* GUC doorbell is not getting pinned properly. As a temp WA,
	 * trying to reaquire it after some delay*/
	{
		msleep(500);
		itouch_reacquire_guc_db();

	}

	return 0;
}

int itouch_gfx_device_create(void)
{
	int ret = 0;

	g_allocmap.last_id = (0xFFFFFFFF);
	return ret;
}

int itouch_gfx_device_destroy(struct itouch_device *idv)
{

#if 0
	u32 i;
	u32 numTotalAllocs;
	u32 workQueueSize = idv->gfxAllocMap->workQueueSize;
	int status = 0;

	numTotalAllocs =
	    idv->gfxAllocMap->numAllocations * idv->gfxAllocMap->workQueueSize +
	    idv->gfxAllocMap->workQueueSize;
	for (i = 0; i < numTotalAllocs; ++i) {
		GfxAlloc *pAlloc = &pGfxAllocMap->pResourceBuffers[i];
		if (pAlloc->cpu_addr) {
			status =
			    idv->pHidGfxCallbacks->pGfxTable->
			    pfnUnmapBuffer(hGfxContext, pAlloc->hGmmBlock);
			if (!status)
				itouch_dbg(idv,
					   "Unmap Buffer didnt work for Resource Buffer, Status [%!STATUS!]",
					   status);
		}
	}

	ExFreePoolWithTag(pGfxAllocMap->pResourceBuffers, 'tmk');
	memset(pGfxAllocMap, 0, sizeof(GfxAllocMap));
	memset(ppKernelSpec, 0, sizeof(GfxKernelSpec) * workQueueSize);

	status =
	    pdevContext->pHidGfxCallbacks->pGfxTable->
	    pfnUnmapBuffer(hGfxContext, plast_submitted_id->hGmmBlock);
	itouch_dbg(idv, "Unmap Buffer plast_submitted_id, Status [%!STATUS!]",
		   status);
	memset(plast_submitted_id, 0, sizeof(GfxAlloc));
	idv->multi_touch.memory_allocation_status = false;
#endif

	return 0;
}

static int get_blob_size(struct file *file)
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

int itouch_gfx_read_kernelspec(struct itouch_device *idv,
				const char *kernel_path, void **kernelspec)
{
	struct file *f;
	void *buf = NULL;
	mm_segment_t fs;
	int size_read = 0;

	/* Interface with gfx blob test */
	fs = get_fs();
	set_fs(get_ds());
	f = filp_open(kernel_path, O_RDONLY, 0);

	set_fs(fs);

	if (IS_ERR(f) || NULL == f->f_op) {
		int ret = PTR_ERR(f);
		itouch_dbg(idv, "Bad file ptr for Kernelspec,Err- %d\n", ret);
		return ret;
	} else {
		/* Get the file size first */
		int size = -1;

		size = get_blob_size(f);

		if (size < 0) {
			itouch_dbg(idv, "Invalid blob file size=0x%x;taking static size\n",
					 size);

			/*Fall back to static size */
			size = 1083 * 1024;
		}

		buf = vmalloc(size);
		if (NULL == buf) {
			itouch_dbg(idv, "iTouch Blob buf allocation failed\n");
		} else {
			fs = get_fs();
			set_fs(get_ds());
			size_read = __vfs_read(f, buf, 2637584, &f->f_pos);
			set_fs(fs);

			/* If size read is less than expected, fail the call */
			if (size_read < size) {
				itouch_dbg(idv, "itouch blob read failed, sizes dont match\n");
				vfree(buf);
				buf = NULL;
				size_read = -1;
			}
		}

		*kernelspec = buf;
	}

	filp_close(f, NULL);
	return size_read;
}
