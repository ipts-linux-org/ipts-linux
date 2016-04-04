/*
 *
 * Intel Integrated Touch Gfx Interface Layer
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
#include "itouch-state-codes.h"

static DEFINE_SPINLOCK(input_event_lock);
unsigned long input_event_lock_flags;

static struct itouch_gfx_ctx *itouch_gfx_ctx_object = NULL;

static int itouch_register(struct itouch_device *idv,
			   struct itouch_gfx_ctx *ctx)
{
	int ret = 0;

	/* Gfx is ready for interface exchange. */
	ctx->clbks.hid_version = ITOUCH_INTERFACE_VERSION;
	ctx->clbks.hid_handle = (HANDLE) ctx;
	ctx->clbks.hid_ops.signal_complete = itouch_signal_complete;
	ctx->clbks.hid_ops.display_on_off = itouch_display_on_off;
	ctx->dev = idv;

	ret = i915_itouch_client_register(&(ctx->clbks));
	if (ret) {
		if (ctx->clbks.gfx_version != ctx->clbks.hid_version) {
			itouch_dbg(idv, "gfx/hid header version mismatch, return value=%d\n",ret);
			itouch_dbg(idv, "ctx->clbks.gfxVersion=%d--- ctx->clbks.hidVersion=%d\n",
				ctx->clbks.gfx_version, ctx->clbks.hid_version);
		} else if (!ctx->clbks.gfx_handle) {
			itouch_dbg(idv, "didn't receive the Gfx Context, return value=%d\n",ret);
		} else if (ctx->clbks.i915_ops.map_buffer == NULL) {
			itouch_dbg(idv, "didn't receive the Gfx Context, return value=%d\n",ret);
		}
	}

	/* Save the i915 gfx ops */
	ctx->gfx_ops = ctx->clbks.i915_ops;
	ctx->gfx_ctx = ctx->clbks.gfx_handle;

	return ret;
}

void itouch_signal_complete(HANDLE ctx, u32 id)
{
	struct guc_process_desc *guc_processdesc = NULL;
	struct itouch_device *idv;
	struct itouch_gfx_ctx *context = (struct itouch_gfx_ctx *)(ctx);

	if (context){
		idv = (struct itouch_device *)(context->dev);
	} else {
		itouch_dbg(idv, "GUC Signal received but context is invalid\n");
	}

	guc_processdesc =
	    (struct guc_process_desc *) itouch_gfx_ctx_object->
	    guc_appprocess;

	spin_lock_irqsave(&input_event_lock, input_event_lock_flags);

	/*itouch_info(idv, "GUC WQ TailOffset=0x%08x, Doorbell Quadword=0x%llx",
			guc_processdesc->tail,
			*((ULONG64 *)guc_processdesc->db_base_addr));*/

	/* Need the idv object back in this context
	 * Need to work with the gfx callback API to store idv
	 * as a void * object.
	 */
	if(idv->plast_submitted_id)
		idv->current_buffer_index = *(u32 *) (idv->plast_submitted_id);

	itouch_info(idv, "Got signal with current buffer index = %d \n",
				 idv->current_buffer_index);

	schedule_work(&idv->multi_touch_work);
	mod_delayed_work(system_wq, &idv->reacquire_db_work,
						REACQUIRE_DB_WORK_DELAY);

	itouch_info(idv,"delayed work value modified %d\n",
						REACQUIRE_DB_WORK_DELAY);

	spin_unlock_irqrestore(&input_event_lock, input_event_lock_flags);

}

void itouch_display_on_off(HANDLE ctx, bool enable)
{
	struct itouch_device *idv;
	struct itouch_gfx_ctx *context = (struct itouch_gfx_ctx *)(ctx);

	if (context){
		idv = (struct itouch_device *)(context->dev);
	} else {
		itouch_dbg(idv,
			"Display Signal received but context is invalid\n");
	}

	if(enable)
	{
		itouch_dbg(idv,"display is off\n");

		/*Do all these if display is off*/
		if(idv->display_status == false){
			/*tell ME to enable touchIC*/
			touch_hid_driver_ready(idv);

			/*enable WQ*/
			idv->display_status = true;
			mod_delayed_work(system_wq, &idv->reacquire_db_work,0);
						//REACQUIRE_DB_WORK_DELAY);
		}
	}
	else {
		itouch_dbg(idv,"display is off\n");

		/*disable delayed wq
		* as delayed work could be on another core
		* cancel work does not always perform well
		* so we set display_status also which
		* prevents reschduling of delayed work
		*/
		idv->display_status = false;
		cancel_delayed_work(&idv->reacquire_db_work);
		flush_workqueue(system_wq);

		/* tell ME to disable touchIC*/
		if(touch_sensor_quiesce_io(idv)) {
			itouch_dbg(idv, "touch senser quiesce failed \n");
			return;
		}
		else
			itouch_dbg(idv, "touch quisece success\n");


	}

}

/*
* Calling this function will submit a workload to GuC for exeuction. After
* succesfully completing the workload itouch_signal_complete() will be called.
* Call this function only after successful initilisation of GUC and
* itouch_gfx_setup_kernel().
*/
void manual_submit(void)
{
	struct guc_process_desc *guc_processdesc = NULL;
	union guc_doorbell_qw db_cmp, db_exc, db_ret;
	union guc_doorbell_qw *db;
	int attempt = 2, ret = -EAGAIN;

	guc_processdesc =
		(struct guc_process_desc *) itouch_gfx_ctx_object->
		guc_appprocess;

	/* Increment the tail */
	guc_processdesc->tail += itouch_gfx_ctx_object->workitem_size;

	/* Handle wrap around case */
	if (guc_processdesc->tail >= guc_processdesc->wq_size_bytes)
		guc_processdesc->tail = 0;

	/* Update the doorbell */
	db_cmp.db_status = GUC_DOORBELL_ENABLED;
	db_cmp.cookie = 0;

	/* cookie to be updated */
	db_exc.db_status = GUC_DOORBELL_ENABLED;
	db_exc.cookie = db_cmp.cookie + 1;

	/* pointer of current doorbell cacheline */
	db = (union guc_doorbell_qw *)guc_processdesc->db_base_addr;

	while (attempt--) {
		/* lets ring the doorbell */
		db_ret.value_qw = atomic64_cmpxchg((atomic64_t *)db,
			db_cmp.value_qw, db_exc.value_qw);

		/* if the exchange was successfully executed */
		if (db_ret.value_qw == db_cmp.value_qw) {
			/* db was successfully rung */
			ret = 0;
			break;
		}

        /* XXX: doorbell was lost and need to acquire it again */
        if (db_ret.db_status == GUC_DOORBELL_DISABLED) {
			printk("Doorbell disabled for iTouch\n");
			break;
		}

		printk("Cookie mismatch. Expected %d, returned %d\n",
			db_cmp.cookie, db_ret.cookie);

		/* update the cookie to newly read cookie from GuC */
		db_cmp.cookie = db_ret.cookie;
		db_exc.cookie = db_cmp.cookie + 1;
	}

	i915_print_itouch_guc_info(guc_processdesc);
}


int itouch_open_gpu(struct itouch_device *idv)
{
	int ret;
	struct i915_itouch_opengpu gpu_params = { 0 };

	if (itouch_gfx_ctx_object != NULL)
		return -EINVAL;

	itouch_gfx_ctx_object =
	    kzalloc(sizeof(*itouch_gfx_ctx_object), GFP_KERNEL);
	if (!itouch_gfx_ctx_object)
		return -ENOMEM;

	ret = itouch_register(idv, itouch_gfx_ctx_object);
	if (ret) {
		kfree(itouch_gfx_ctx_object);
		itouch_gfx_ctx_object = NULL;
		return ret;
	}

	/* Assume GUC is available and populate the args. i915 will handle
	 * GUC vs Non-GUC cases appropriately
	 */

	gpu_params.engines_used = 1;
	gpu_params.checkcode = KM_GUC_OP_CHECK_CODE_OPEN_GPU;
	gpu_params.priority_requested = UK_CONTEXT_PRIORITY_HIGH;
	gpu_params.flags = 1 << 2;

	ret =
	    itouch_gfx_ctx_object->gfx_ops.open_gpu(
							itouch_gfx_ctx_object->clbks.gfx_handle,
							&gpu_params);
	if (ret) {
		kfree(itouch_gfx_ctx_object);
		itouch_gfx_ctx_object = NULL;
		return ret;
	}

	/* Save the details */
	itouch_gfx_ctx_object->gpu_handle = gpu_params.gpu_handle;
	itouch_gfx_ctx_object->guc_appprocess =
	    (void *)gpu_params.process_desc;

	itouch_dbg(idv, "GUC based Open GPU success\n");
	itouch_info(idv, "GpuHandle = 0x%llx, app_process = 0x%p\n",
		   itouch_gfx_ctx_object->gpu_handle,
		   itouch_gfx_ctx_object->guc_appprocess);

	/* Save the details in idv for future reference */
	idv->gpu_handle = gpu_params.gpu_handle;
	idv->appPrcocessParams =
	    (struct guc_process_desc *) gpu_params.process_desc;

	idv->DoorbellCookieAddr = (gpu_params.db_phy_addr + DOORBELL_COOKIE_ADDRESS_OFFSET);
	idv->TailPtrAddr = gpu_params.tail_phy_addr;
	itouch_dbg(idv,
		   "DoorbellCookieAddr = 0x%llx, TailPtrAddr = 0x%llx, AppProcess = 0x%p\n",
		   idv->DoorbellCookieAddr, idv->TailPtrAddr,
		   idv->appPrcocessParams);

	idv->work_queue_size = (u16) idv->appPrcocessParams->wq_size_bytes;
	idv->work_queue_item_size = WORK_QUEUE_ITEM_SIZE;

	idv->graphics_state.comp_state = GFX_GPU_OPEN_COMPLETE;
	return ret;
}

int itouch_close_gpu(struct itouch_device *idv)
{
	struct i915_itouch_closegpu close = { 0 };

	close.process_desc = (uint64_t)idv->appPrcocessParams;
	close.gpu_handle = idv->gpu_handle;
	close.checkcode = KM_GUC_OP_CHECK_CODE_CLOSE_GPU;

	if (itouch_gfx_ctx_object == NULL)
		return -1;

	itouch_gfx_ctx_object->gfx_ops.close_gpu(
				itouch_gfx_ctx_object->clbks.gfx_handle,
				&close);

	kfree(itouch_gfx_ctx_object);
	itouch_gfx_ctx_object = NULL;

	return 0;
}

struct i915_itouch_mapbuffer *
itouch_map_buffer(unsigned int size, u32 flags)
{
	int ret;
	struct i915_itouch_mapbuffer *buf;

	if (itouch_gfx_ctx_object == NULL)
		return NULL;

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return NULL;

	buf->contextid = itouch_gfx_ctx_object->clbks.gfx_handle;
	buf->size = size;
	buf->flags = flags;

	ret =
	    itouch_gfx_ctx_object->gfx_ops.map_buffer(itouch_gfx_ctx_object->
							clbks.gfx_handle, buf);
	if (ret) {
		kfree(buf);
		buf = NULL;
	}
	return buf;
}

void itouch_unmap_buffer(struct i915_itouch_mapbuffer *buf)
{
	int ret;

	if (itouch_gfx_ctx_object == NULL)
		return;

	ret =
	    itouch_gfx_ctx_object->gfx_ops.
	    unmap_buffer(itouch_gfx_ctx_object->clbks.gfx_handle,
			   buf->buf_handle);

	kfree(buf);
}

int itouch_get_guc_wq(struct i915_itouch_mapbuffer *buf, unsigned int wi_size)
{
	struct guc_process_desc *guc_processdesc = NULL;

	/* Check if GUC is enabled or not */
	if (!itouch_gfx_ctx_object->gpu_handle)
		return 0;

	guc_processdesc =
	    (struct guc_process_desc *) itouch_gfx_ctx_object->
	    guc_appprocess;

	/* Populate the WQ details Reusing same i915_itouch_mapbuffer struct */
	buf->cpu_addr = (void *)guc_processdesc->wq_base_addr;
	buf->size = guc_processdesc->wq_size_bytes;
	buf->contextid = itouch_gfx_ctx_object->clbks.gfx_handle;

	itouch_gfx_ctx_object->workitem_size = wi_size;

	printk(KERN_ERR "GUC WQ size from i915 = %d wq addr=%p\n", buf->size, buf->cpu_addr);

	return buf->size;
}

void itouch_reacquire_guc_db(void)
{
	/* Check if GUC is enabled or not */
	if (!itouch_gfx_ctx_object)
		return;

	i915_itouch_reacquire_db(itouch_gfx_ctx_object->clbks.gfx_handle);

	return;
}
#if 0
/* int __init itouch_init(void) */
int itouch_gfx_init(void)
{
	printk("iTouch HID module starts loading...\n");
	/*
	 * Register a notification callback in iGfx. It'll be called when
	 * graphics is initialized and ready to work.
	 */
	/* igfx_register_callback(&igfx_nb); */

	// itouch_gfx_device_create();

	printk("iTouch HID module finished loading.\n");
	return 0;
}

void __exit itouch_gfx_exit(void)
{
	printk("iTouch HID module has left the kernel.\n");
}
#endif
int touch_device_intel_graphics_unload(struct itouch_device *idv)
{
	int status = 0;

	if (idv->graphics_state.comp_state == GFX_GPU_OPEN_COMPLETE
	    || idv->graphics_state.comp_state == GFX_TOUCH_READY) {
		status = itouch_close_gpu(idv);
		status = itouch_gfx_device_destroy(idv);
#if 0
		status =
		    GfxDestroy(idv, idv->pHidGfxCallbacks->hGfxContext,
			       idv->gfxKernelSpec, &idv->gfxAllocMap,
			       idv->plast_submitted_id);
		idv->pHidGfxCallbacks->pGfxTable->pfnSignalUnload(idv->
								  pHidGfxCallbacks->
								  hGfxContext);
		ExFreePoolWithTag(idv->pHidGfxCallbacks, IntTouch);
#endif
		idv->graphics_state.comp_state = GFX_STATE_NONE;
	}

	return status;
}
