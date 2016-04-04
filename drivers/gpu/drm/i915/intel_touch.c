/*
 * Copyright  2014 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <drm/drmP.h>

#include "i915_drv.h"
#include "intel_touch.h"
#include "itouch-gfx-interface.h"

/* global iTouch context */
struct itouch_gpu_ctx {
	struct drm_device* dev;
	struct drm_file*   file;
	/* client context list */
	struct {
		spinlock_t       lock;
		struct list_head list;
	}clients;
};

static struct itouch_gpu_ctx *g_touch_ctx = NULL;

struct itouch_client_ctx {
	struct list_head   ctx_list;
	unsigned int       ctx_id;

	/* HID interface */
	struct hid_itouch_ops hid_ops;
	uint64_t hid_handle;

	/* buffers' list */
	struct {
		spinlock_t       lock;
		struct list_head list;
		unsigned int     count;
	}buffers;

};

struct itouch_object {
	/* context list entry for this buffer */
	struct list_head list;

	struct itouch_client_ctx *ctx;

	struct drm_i915_gem_object *gem_obj;
	unsigned int pin_count;

	unsigned int size;
	unsigned int handle;
	void        *cpu_addr;
};

static int
i915_gem_attach_itouch_object(struct drm_device *dev,
			    struct drm_i915_gem_object *obj,
			    int align)
{
	obj->phys_handle = drm_pci_alloc(dev, obj->base.size, align);
	if (!obj->phys_handle) {
		DRM_ERROR("drm_pci_alloc failed\n");
		return -ENOMEM;
	}
#ifdef CONFIG_X86
	set_memory_wc((unsigned long)obj->phys_handle->vaddr, obj->phys_handle->size / PAGE_SIZE);
#endif

	return 0;
}

/* Support for Contiguous allocations*/
static int
i915_gem_object_get_pages_itouch(struct drm_i915_gem_object *obj)
{
	struct drm_i915_private *dev_priv = obj->base.dev->dev_private;
	int page_count, i;
	struct sg_table *st;
	struct scatterlist *sg;
	struct page *page;
	int ret = 0;
	void *alloc_addr = NULL;

	/* Assert that the object is not currently in any GPU domain. As it
	 * wasn't in the GTT, there shouldn't be any way it could have been in
	 * a GPU cache
	 */
	BUG_ON(obj->base.read_domains & I915_GEM_GPU_DOMAINS);
	BUG_ON(obj->base.write_domain & I915_GEM_GPU_DOMAINS);

	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return -ENOMEM;

	page_count = obj->base.size / PAGE_SIZE;
	if (sg_alloc_table(st, page_count, GFP_KERNEL)) {
		kfree(st);
		return -ENOMEM;
	}

	ret = i915_gem_attach_itouch_object(dev_priv->dev, obj, PAGE_SIZE);
	if (ret) {
		DRM_ERROR("i915_gem_attach_itouch_object failed\n");
		sg_free_table(st);
		kfree(st);
		return -ENOMEM;
	}
	alloc_addr = obj->phys_handle->vaddr;
	DRM_DEBUG_DRIVER("Contig size = %ld virtual addr = %p bus addr = %p\n",
			obj->base.size, obj->phys_handle->vaddr,
			(void *)(obj->phys_handle->busaddr));

	sg = st->sgl;
	st->nents = 0;

	/* Extract physical address and popluate sg table */
	for (i = 0; i < page_count; i++) {

		page = virt_to_page(alloc_addr + (i * PAGE_SIZE));

		if (i)
			sg = sg_next(sg);

		st->nents++;
		sg_set_page(sg, page, PAGE_SIZE, 0);
	}

	sg_mark_end(sg);
	obj->pages = st;

	ret = i915_gem_gtt_prepare_object(obj);
	if (ret)
		return ret;

	if (i915_gem_object_needs_bit17_swizzle(obj))
		i915_gem_object_do_bit_17_swizzle(obj);

	return 0;
}

static void
i915_gem_object_put_pages_itouch(struct drm_i915_gem_object *obj)
{
	struct drm_i915_private *dev_priv = obj->base.dev->dev_private;
        struct scatterlist *sg;
        int i;
	void *alloc_addr = NULL;

        if (obj->madv != I915_MADV_WILLNEED)
                obj->dirty = 0;

	if (!obj->phys_handle) {
		DRM_ERROR("iTouch contiguous obj's phy handle is NULL\n");
		return;
	}

	i915_gem_gtt_finish_object(obj);

	alloc_addr = obj->phys_handle->vaddr;

	DRM_ERROR("virtual addr extracted = %p\n", alloc_addr);

        for_each_sg(obj->pages->sgl, sg, obj->pages->nents, i) {
                struct page *page = sg_page(sg);

                if (obj->dirty)
                        set_page_dirty(page);

                //mark_page_accessed(page);
                //page_cache_release(page);
        }
        obj->dirty = 0;

        sg_free_table(obj->pages);
        kfree(obj->pages);

#ifdef CONFIG_X86
	set_memory_wb((unsigned long)obj->phys_handle->vaddr, obj->phys_handle->size / PAGE_SIZE);
#endif
	drm_pci_free(dev_priv->dev, obj->phys_handle);
}

static const struct drm_i915_gem_object_ops i915_gem_object_itouch_ops = {
	.get_pages = i915_gem_object_get_pages_itouch,
	.put_pages = i915_gem_object_put_pages_itouch,
};

static struct itouch_object*
itouch_object_create(struct itouch_client_ctx *client_ctx,
			unsigned int size,
			u32 flags)
{
	struct itouch_object       *obj;
	struct drm_i915_gem_create  args;
	int ret;

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj)
		return NULL;

	args.size = ALIGN(size, 4096);
	ret = i915_gem_create_ioctl(g_touch_ctx->dev, &args, g_touch_ctx->file);
	if (ret) {
		kfree(obj);
		return NULL;
	}

	obj->handle = args.handle;
	obj->size   = size;

	spin_lock(&g_touch_ctx->file->table_lock);
	obj->gem_obj = to_intel_bo(idr_find(&g_touch_ctx->file->object_idr,
					obj->handle));

	/* Replace ops to handle contiguous allocations.
	 * TBD: Any other better way to allocate contig??
	 */
	if (flags & ITOUCH_ALLOC_CONTIGUOUS)
		obj->gem_obj->ops = &i915_gem_object_itouch_ops;

	spin_unlock(&g_touch_ctx->file->table_lock);
	obj->ctx = client_ctx;
	if (obj->ctx) {
		spin_lock(&client_ctx->buffers.lock);
		list_add_tail(&obj->list, &client_ctx->buffers.list);
		(client_ctx->buffers.count)++;
		spin_unlock(&client_ctx->buffers.lock);
	}

	return obj;
}

static void itouch_object_free(struct itouch_object* obj)
{
	drm_gem_handle_delete(g_touch_ctx->file, obj->handle);
	if (obj->ctx) {
		list_del(&obj->list);
		(obj->ctx->buffers.count)--;
	}
	kfree(obj);
}

static int itouch_object_pin(struct itouch_object* obj ,
				struct intel_context *touch_hw_ctx)
{
	struct i915_address_space *vm = NULL;
	struct drm_i915_private *dev_priv = g_touch_ctx->dev->dev_private;
	int ret = 0;

	if (touch_hw_ctx->ppgtt) {
		vm = &touch_hw_ctx->ppgtt->base;
	}
	else {
		if (USES_FULL_PPGTT(g_touch_ctx->dev)) {
			DRM_ERROR("Context PPGTT is not available \n");
			return -1;
		} else {
			/* To handle Aliased PPGTT case */
			vm = &dev_priv->gtt.base;
		}
	}

	ret = i915_gem_object_pin(obj->gem_obj, vm, PAGE_SIZE, PIN_USER);

	return ret;
}

static void itouch_object_unpin(struct itouch_object* obj)
{
	/* TBD: Add support */
}

static void* itouch_object_map(struct drm_i915_gem_object *gem_obj)
{
	struct sg_page_iter sg_iter;
	struct page **pages;
	void *base = NULL;
	int i = 0;

	if(ALIGN(gem_obj->base.size, 4096) > 4096) {
		if (gem_obj->vmapping_count) {
			gem_obj->vmapping_count++;
			base = gem_obj->dma_buf_vmapping;

		} else {
			pages = drm_malloc_ab(gem_obj->base.size >> PAGE_SHIFT,
						sizeof(*pages));
			if (pages == NULL) {
				DRM_ERROR("drm_malloc_ab failed\n");
				return NULL;
			}

			for_each_sg_page(gem_obj->pages->sgl, &sg_iter,
					gem_obj->pages->nents, 0) {

				pages[i++] = sg_page_iter_page(&sg_iter);
			}

			gem_obj->dma_buf_vmapping = vmap(pages, i, 0, PAGE_KERNEL);
			drm_free_large(pages);

			if (!gem_obj->dma_buf_vmapping) {
				DRM_ERROR("vmap failed\n");
				return NULL;
			}

			base = gem_obj->dma_buf_vmapping;
			gem_obj->vmapping_count = 1;
		}
	} else {
		base = kmap(sg_page(gem_obj->pages->sgl));
	}

	return base;
}

static void itouch_object_unmap(struct itouch_object* obj)
{
	struct drm_i915_gem_object *gem_obj = obj->gem_obj;
	if(ALIGN(obj->size, 4096) > 4096) {
		if (--gem_obj->vmapping_count == 0) {
			vunmap(gem_obj->dma_buf_vmapping);
			gem_obj->dma_buf_vmapping = NULL;
		}
	} else {
		kunmap(sg_page(gem_obj->pages->sgl));
	}
	obj->cpu_addr = NULL;
}

static struct itouch_client_ctx* itouch_create_client_context(void)
{
	struct drm_i915_gem_context_create args;
	struct itouch_client_ctx* client_ctx;
	struct intel_context *touch_hw_ctx;
	struct drm_i915_private *dev_priv = g_touch_ctx->dev->dev_private;
	int ret;

	client_ctx = kzalloc(sizeof(*client_ctx), GFP_KERNEL);
	if (!client_ctx)
		return NULL;

	/* Create a GPU context for iTouch client */
	ret = i915_gem_context_create_ioctl(g_touch_ctx->dev, &args,
						g_touch_ctx->file);
	if (ret) {
		DRM_ERROR("Failed to create GPU context for iTouch client\n");
		kfree(client_ctx);
		return NULL;
	}

	/* Save the context id */
	client_ctx->ctx_id = args.ctx_id;

	/* Initialize the context right away.*/
	ret = i915_mutex_lock_interruptible(g_touch_ctx->dev);
	if (ret) {
		DRM_ERROR("i915_mutex_lock_interruptible failed \n");
		kfree(client_ctx);
		return NULL;
	}

	/* Initializing for RCS ring as iTouch will be using only RCS */
	touch_hw_ctx = i915_gem_validate_context(g_touch_ctx->dev,
				g_touch_ctx->file, &dev_priv->ring[RCS],
				client_ctx->ctx_id);

	if (touch_hw_ctx == NULL) {
		DRM_ERROR("Touch GPU HW context is NULL\n");
		mutex_unlock(&g_touch_ctx->dev->struct_mutex);
		kfree(client_ctx);
		return NULL;
	}

	/* Do we need to have reference for ctx at this point?? */
	i915_gem_context_reference(touch_hw_ctx);

	/* Release the mutex */
	mutex_unlock(&g_touch_ctx->dev->struct_mutex);

	spin_lock_init(&client_ctx->buffers.lock);
	INIT_LIST_HEAD(&client_ctx->buffers.list);

	spin_lock(&g_touch_ctx->clients.lock);
	list_add_tail(&client_ctx->ctx_list, &g_touch_ctx->clients.list);
	spin_unlock(&g_touch_ctx->clients.lock);

	return client_ctx;
}

int i915_itouch_notify()
{
	struct itouch_client_ctx *client_ctx;

	if(!g_touch_ctx)
                return 0;

	list_for_each_entry(client_ctx, &g_touch_ctx->clients.list, ctx_list) {

		client_ctx->hid_ops.signal_complete(client_ctx->hid_handle, 0);
	}

	return 0;
}

int i915_itouch_notify_display_status(bool display_status)
{
	struct itouch_client_ctx *client_ctx;

	if(!g_touch_ctx)
		return 0;

	list_for_each_entry(client_ctx, &g_touch_ctx->clients.list, ctx_list) {

		client_ctx->hid_ops.display_on_off(client_ctx->hid_handle,
								display_status);
	}

	return 0;
}

static int i915_itouch_grab_gpu_handle(struct i915_itouch_opengpu *opengpu)
{
	struct drm_i915_private *dev_priv = g_touch_ctx->dev->dev_private;
	struct intel_guc *guc = &dev_priv->guc;
	struct i915_guc_client *client;
	struct guc_process_desc *desc;
	void *base = NULL;
	u64 phy_base = 0;

	client = guc->itouch_client;
	if(!client)
	{
		DRM_ERROR("iTouch GUC client is NOT created\n");
		return -EINVAL;
	}

	/* iTouch module expect GUC client's process descriptor to access
	 * WQ inorder to populate commands and other fields. Map the client
	 * object
	 */
	base = itouch_object_map(client->client_obj);
	if (!base)
		return -ENOMEM;

	desc = (struct guc_process_desc *)((u64)base + client->proc_desc_offset);

	desc->wq_base_addr = (u64)base + client->wq_offset;
	desc->db_base_addr = (u64)base + client->doorbell_offset;

	/* iTouch expects physical addresses to pass it to ME */
	phy_base = sg_dma_address(client->client_obj->pages->sgl);

	opengpu->db_phy_addr = phy_base + client->doorbell_offset;
	opengpu->tail_phy_addr = phy_base + client->proc_desc_offset +
					offsetof(struct guc_process_desc, tail);

	opengpu->gpu_handle = (uint64_t)client;
	opengpu->process_desc = (uint64_t)(void *)desc;
	opengpu->process_desc_size_bytes = sizeof(*desc);
	opengpu->priority_assigned = GUC_CTX_PRIORITY_HIGH;
	opengpu->guc_status = 0;

	return 0;
}

static int i915_itouch_open_gpu(uint64_t gfx_handle,
				struct i915_itouch_opengpu *opengpu)
{
	struct itouch_client_ctx *client_ctx = NULL;
	struct intel_context *touch_hw_ctx = NULL;
	int ret = 0;

	list_for_each_entry(client_ctx, &g_touch_ctx->clients.list, ctx_list) {
		if (client_ctx == (struct itouch_client_ctx *)gfx_handle) {
			break;
		}
	}

	if (client_ctx != (struct itouch_client_ctx *)gfx_handle)
		return -EINVAL;

	/* Acquire mutex for synchronization */
	ret = i915_mutex_lock_interruptible(g_touch_ctx->dev);
	if (ret) {
		DRM_ERROR("i915_mutex_lock_interruptible failed \n");
		return -1;
	}

	/* Get the GPU context created for iTouch client */
	touch_hw_ctx = i915_gem_context_get(g_touch_ctx->file->driver_priv,
						client_ctx->ctx_id);
	if (IS_ERR(touch_hw_ctx)) {
		mutex_unlock(&g_touch_ctx->dev->struct_mutex);
		DRM_ERROR("Not able to retieve GPU context for iTouch client\n");
		return -1;
	}

	/* Disable any stale itouch client for GUC */
	i915_guc_itouch_submission_disable(g_touch_ctx->dev);

	/* Allocate a new GUC client specific to iTouch */
	if(i915_guc_itouch_submission_enable(g_touch_ctx->dev, touch_hw_ctx))
	{
		DRM_ERROR("i915_guc_itouch_submission_enable failed\n");
		mutex_unlock(&g_touch_ctx->dev->struct_mutex);
		return -1;
        }

	if(i915_itouch_grab_gpu_handle(opengpu))
	{
		DRM_ERROR("itouch_grab_gpu_handle failed\n");
		mutex_unlock(&g_touch_ctx->dev->struct_mutex);
		return -1;
	}

	/* Release the mutex */
	mutex_unlock(&g_touch_ctx->dev->struct_mutex);

	DRM_DEBUG_DRIVER("Open GPU completed for iTouch client\n");

	return 0;
}

static int i915_itouch_close_gpu(uint64_t gfx_handle,
				struct i915_itouch_closegpu *closegpu)
{
	/* TBD: Add support */
	return 0;
}

static int i915_itouch_map_buffer(uint64_t gfx_handle,
				struct i915_itouch_mapbuffer *mapbuffer)
{
	struct itouch_client_ctx *client_ctx = NULL;
	struct itouch_object* obj;
	struct page *page;
	int ret = 0;
	struct intel_context *touch_hw_ctx = NULL;
	struct drm_i915_private *dev_priv = g_touch_ctx->dev->dev_private;

	list_for_each_entry(client_ctx, &g_touch_ctx->clients.list, ctx_list) {
		if (client_ctx == (struct itouch_client_ctx *)gfx_handle) {
			break;
		}
	}

	if (client_ctx != (struct itouch_client_ctx *)gfx_handle)
		return -EINVAL;

	obj = itouch_object_create(client_ctx, mapbuffer->size, mapbuffer->flags);
	if (!obj)
		return -ENOMEM;

	/* Acquire mutex first */
	ret = i915_mutex_lock_interruptible(g_touch_ctx->dev);
	if (ret) {
		DRM_ERROR("i915_mutex_lock_interruptible failed \n");
		itouch_object_free(obj);
		return -EINVAL;
	}

	touch_hw_ctx = i915_gem_context_get(g_touch_ctx->file->driver_priv, client_ctx->ctx_id);
	if (IS_ERR(touch_hw_ctx)) {
		DRM_ERROR("itouch_map: Not able to retrieve itouch HW Context \n");
		itouch_object_free(obj);
		mutex_unlock(&g_touch_ctx->dev->struct_mutex);
		return -EINVAL;
	}

	/* Pin the obj. Will be pinned for lifetime unless requested*/
	ret = itouch_object_pin(obj, touch_hw_ctx);
	if (ret) {
		DRM_ERROR("Not able to pin iTouch obj\n");
		itouch_object_free(obj);
		mutex_unlock(&g_touch_ctx->dev->struct_mutex);
		return -ENOMEM;
	}

	obj->cpu_addr = itouch_object_map(obj->gem_obj);

	/* Get the PGGTT address */
	if (USES_FULL_PPGTT(g_touch_ctx->dev)) {
		mapbuffer->gfx_addr = (void*)i915_gem_obj_offset(obj->gem_obj,
						&touch_hw_ctx->ppgtt->base);
	} else {
		mapbuffer->gfx_addr = (void*)i915_gem_obj_offset(obj->gem_obj,
						&dev_priv->gtt.base);

	}
	mapbuffer->cpu_addr = (void*)obj->cpu_addr;
	mapbuffer->buf_handle = (uint64_t)obj;

	/* For contig allocations, share the base phys address */
	if (mapbuffer->flags & ITOUCH_ALLOC_CONTIGUOUS) {
		page = sg_page(obj->gem_obj->pages->sgl);
		mapbuffer->cpu_phy_addr = (uint64_t)
					(page_to_pfn(page) << PAGE_SHIFT);
	}

	/* Release the mutex */
	mutex_unlock(&g_touch_ctx->dev->struct_mutex);

	return 0;
}

static int i915_itouch_unmap_buffer(uint64_t gfx_handle, uint64_t buf_handle)
{
	struct itouch_object* obj = (struct itouch_object*)buf_handle;

	itouch_object_unmap(obj);
	itouch_object_unpin(obj);
	itouch_object_free(obj);

	return 0;
}

static int i915_itouch_signal_unload(uint64_t gfx_handle)
{
	/* TBD: Add support */
	return 0;
}

int i915_itouch_client_register(struct i915_itouch_register_args *args)
{
	int ret = 0;
	struct itouch_client_ctx* client_ctx;

	/* Check if the client framework is supported or not */
	if (!g_touch_ctx)
		return -EINVAL;

	if(args &&
	  (args->hid_version == ITOUCH_INTERFACE_VERSION)) {

		client_ctx = itouch_create_client_context();
		if (!client_ctx)
			return -ENOMEM;

		args->gfx_version = ITOUCH_INTERFACE_VERSION;
		args->i915_ops.open_gpu = i915_itouch_open_gpu;
		args->i915_ops.close_gpu = i915_itouch_close_gpu;
		args->i915_ops.map_buffer = i915_itouch_map_buffer;
		args->i915_ops.unmap_buffer = i915_itouch_unmap_buffer;
		args->i915_ops.signal_unload = i915_itouch_signal_unload;

		args->gfx_handle = (u64)client_ctx;

		client_ctx->hid_ops = args->hid_ops;
		client_ctx->hid_handle = args->hid_handle;
	}
	else {
		ret = -EINVAL;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(i915_itouch_client_register);

void i915_itouch_reacquire_db(uint64_t gfx_handle)
{
	int ret = 0;

	/* Acquire mutex first */
	ret = i915_mutex_lock_interruptible(g_touch_ctx->dev);
	if (ret) {
		DRM_ERROR("i915_mutex_lock_interruptible failed \n");
		return;
	}

	/* Reacquire the doorbell */
	i915_guc_itouch_reacquire_doorbell(g_touch_ctx->dev);

	mutex_unlock(&g_touch_ctx->dev->struct_mutex);
	return;
}
EXPORT_SYMBOL_GPL(i915_itouch_reacquire_db);

void i915_print_itouch_guc_info(struct guc_process_desc *p_desc)
{
	struct drm_i915_private *dev_priv = g_touch_ctx->dev->dev_private;
	struct intel_guc *guc = &dev_priv->guc;
	struct guc_context_desc *ctx_desc;
	struct page *page;
	struct i915_guc_client *client;
	struct intel_context *ctx;
	uint32_t *reg_state;
	char *base;

#define CTX_RING_HEAD	0x04
#define CTX_RING_TAIL	0x06

	page = i915_gem_object_get_page(guc->ctx_pool_obj, 0);
	base = kmap_atomic(page);

	ctx_desc = (struct guc_context_desc *)
			(base + sizeof(*ctx_desc) * p_desc->context_id);

	client = (struct i915_guc_client *)ctx_desc->desc_private;
	ctx = client->owner;

	page = i915_gem_object_get_page(ctx->engine[0].state, LRC_STATE_PN);
	reg_state = kmap_atomic(page);

	DRM_ERROR("iTouch :ring head = %u\n", reg_state[CTX_RING_HEAD+1]);
	DRM_ERROR("iTouch :ring tail = %u\n", reg_state[CTX_RING_TAIL+1]);

	kunmap_atomic(reg_state);
	kunmap_atomic(base);
}
EXPORT_SYMBOL_GPL(i915_print_itouch_guc_info);


/**
 * i915_itouch_init - Initialize framework for guc client i.e.iTouch
 * @dev: drm device
 *
 * Setup the required structures for maintaining the iTouch client. This
 * includes allocating a global structure and drm file private
 *
 */
int i915_itouch_init(struct drm_device *dev)
{
	struct drm_file *file;
	int ret;

	g_touch_ctx = kzalloc(sizeof(*g_touch_ctx), GFP_KERNEL);
	if (!g_touch_ctx)
		return -ENOMEM;

	// create a drm_file object.
	file = kzalloc(sizeof(*file), GFP_KERNEL);
	if (!file) {
		kfree(g_touch_ctx);
		g_touch_ctx = NULL;
		return -ENOMEM;
	}

	// initilize required fields
	idr_init(&file->object_idr);
	spin_lock_init(&file->table_lock);

	// call driver open to initialize the private data
	ret = i915_driver_open(dev, file);
	if (ret) {
		kfree(file);
		kfree(g_touch_ctx);
		g_touch_ctx = NULL;
		DRM_ERROR("Failed to open the i915 driver for iTouch\n");
		return ret;
	}

	g_touch_ctx->dev  = dev;
	g_touch_ctx->file = file;
	spin_lock_init(&g_touch_ctx->clients.lock);
	INIT_LIST_HEAD(&g_touch_ctx->clients.list);

	DRM_DEBUG_DRIVER("Intel iTouch framework initialized\n");

	return ret;
}

void i915_itouch_cleanup(struct drm_device *dev)
{
	if (g_touch_ctx && g_touch_ctx->dev == dev) {

		/*TBD: Need to close the file first */

		kfree(g_touch_ctx->file);
		kfree(g_touch_ctx);
		g_touch_ctx = NULL;
	}
}
