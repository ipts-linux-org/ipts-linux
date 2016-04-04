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

#include "mei-hid.h"
#include "itouch-state-codes.h"
#include "itouch-state-handler.h"
#include "itouch-gfx-hid-interface.h"

#define ITOUCH_DRIVER_NAME "itouch"
#define MEI_ITOUCH_UUID	UUID_LE(0x3e8d0870, 0x271a, 0x4208, \
			0x8e, 0xb5, 0x9a, 0xcb, 0x94, 0x02, 0xae, 0x04)

static DEFINE_MUTEX(event_handler);
static DEFINE_MUTEX(multi_touch_mutex);

const uuid_le mei_itouch_guid = MEI_ITOUCH_UUID;

static struct mei_cl_device_id itouch_mei_cl_tbl[] = {
	{"", MEI_ITOUCH_UUID,MEI_CL_VERSION_ANY},
	{}
};

struct mei_msg_hdr1 {
    u32 me_addr:8;
    u32 host_addr:8;
    u32 length:9;
    u32 reserved:5;
    u32 internal:1;
    u32 msg_complete:1;
} __packed;


ssize_t touch_mode_show(struct device *dev, struct device_attribute *attr,
		                                        char *buf)
{
	struct itouch_device *idv;

	idv = dev_get_drvdata(dev);

	dev_info(dev,
			"GFX_State=0x%x, ME State=0x%x, HID State=0x%x, SensorMode=0x%x\n",
			idv->graphics_state.comp_state,
			idv->me_state.comp_state,
			idv->hid_state.comp_state,
			idv->sensor_mode);

	return sprintf(buf, "%d\n", idv->sensor_mode);
}
//TODO: Verify the function implementation
ssize_t touch_mode_store(struct device *dev, struct device_attribute *attr,
		                                                 const char *buf, size_t count)
{
	int ret;
	long val;
	struct itouch_device *idv;

	idv = dev_get_drvdata(dev);
	ret = kstrtol(buf, 10, &val);
	if (ret)
	   return ret;

	printk(KERN_ERR "%s() called with command = %s(D=%ld)\n", __func__, buf, val);

	switch(val){
	case 0:
		break;
	case 1:
		//will now execute the GET_DEVICE_INFO_CMD
		idv->me_state.comp_state = ME_MEI_IS_SENSOR_READY_RESP_RECEIVED;
		break;
	case 2:
		//will now execute the SET_MODE_CMD;
		idv->me_state.comp_state = ME_MEI_GET_DEVICE_INFO_RECEIVED;
		break;
	case 3:
		//will now execute the SET_MEM_WINDOW_CMD
		idv->me_state.comp_state = ME_MEI_MODE_RESP_RECEIVED;
		break;
	case 4:
		//will now execute the HID_READY_FOR_DATA_CMD
		idv->me_state.comp_state = ME_MEI_SET_MEM_RESP_RECEIVED;
		break;
	case 5:
		break;
	default:
		break;
	}

//	me_state_handler(idv);

	return count;
}

static ssize_t device_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    struct itouch_device *idv;

    idv = dev_get_drvdata(dev);
	return sprintf(buf, "VID=0x%04X, DevID:0x%04X, HWRev:0x%x, FWRev:0x%x\n",
			idv->sensor_data.VendorId, idv->sensor_data.DeviceId,
			idv->sensor_data.HwRev, idv->sensor_data.FwRev);
}


static DEVICE_ATTR_RW(touch_mode);
static DEVICE_ATTR_RO(device_info);

static struct attribute *itouch_attrs[] = {
	&dev_attr_touch_mode.attr,
	&dev_attr_device_info.attr,
	NULL
};

static const struct attribute_group itouch_grp = {
	.attrs = itouch_attrs,
};


MODULE_DEVICE_TABLE(mei, itouch_mei_cl_tbl);


void itouch_stop(struct itouch_device *idv)
{
    free_memory_allocation(idv);
    touch_device_intel_graphics_unload(idv);
    touch_device_mei_disconnect(idv);
}

/*This function gets called during the init stages and
 *when the MultiTouch events are obtained, triggered at
 *touch_signal_complete().
*/
void multi_touch_work(struct work_struct *work)
{
   struct itouch_device *idv = container_of(work,
                        struct itouch_device, multi_touch_work);

    if(idv->driver_state.comp_state == DRIVER_HW_INIT){
        driver_state_handler(idv);
        return;
    }

    mutex_lock(&multi_touch_mutex);

    int_touch_graphics_procssing_complete_dpc(idv);

    mutex_unlock(&multi_touch_mutex);

}

#define to_delayed_work(_work)  container_of(_work, struct delayed_work, work)
/*This function gets called REACQUIRE_DB_WORK_DELAY msec later
 *after last GuC IRQ triggered by touch_singal_complete().
 *In some cases GuC does not pick up the itouch work which we can
 *identify by head and tail difference in ME.Issue happens in GuC FW.
 *reacquiring DB is a WA for this issue.
 */
void reacquire_guc_db(struct work_struct *work)
{
	u32 head=0;
	u32 tail=0;

	struct itouch_device *idv = container_of(to_delayed_work(work),
			struct itouch_device, reacquire_db_work);

	head = idv->appPrcocessParams->head;
	tail = idv->appPrcocessParams->tail;

	itouch_dbg(idv, "In the scheduled reqacuire WQ t:%d h:%d \n",
								tail, head);

	if( (tail - head) > TAIL_THRESHOLD )
	{
		itouch_dbg(idv, "Reqacuiring GUC DB !!! t:%d h:%d \n",
								tail, head);
		itouch_reacquire_guc_db();
	}

	/*Re-schedule the work only if the display is on*/
	if(idv->display_status)
		schedule_delayed_work(&idv->reacquire_db_work,
						REACQUIRE_DB_WORK_DELAY*2);
}

static void itouch_cl_event_cb(struct mei_cl_device *dev, u32 events,
			       void *context)
{
	u8 payload[MAX_MSG_LEN];
	ssize_t payload_size;
	TOUCH_SENSOR_MSG_M2H in_msg;
	struct itouch_device *idv = (struct itouch_device*)context;

	if (test_bit(MEI_CL_EVENT_RX, (unsigned long *)&events)) {

		payload_size = mei_cldev_recv(dev, payload, MAX_MSG_LEN);

		if (payload_size <= 0) {
			itouch_dbg(idv, "mei: event payload is empty. \n");
			goto no_payload;
		}

		//itouch_mei_handle_received_messages(idv, payload, payload_size);
		memcpy(&in_msg, payload, payload_size);
		read_mei_messages(idv, in_msg, payload_size);
		if(idv->me_state.comp_state & 0x80000000){
			itouch_dbg(idv, "ITouch MEI State Machine error=0x%08x\n", idv->me_state.comp_state);
		}

	} else {
		itouch_dbg(idv, "mei: event test bit is not set. \n");
		return;
	}

no_payload:
	clear_bit(MEI_CL_EVENT_RX, (unsigned long *)&events);
	smp_mb__after_atomic();
}

int itouch_mei_cl_probe(struct mei_cl_device *dev,
			const struct mei_cl_device_id *id)
{
	int ret = 0;
	struct itouch_device *idv=NULL;

	pr_info("Intel iTouch: %s() called, %p\n",__func__,dev);

	ret = mei_cldev_enable(dev);
	if (ret < 0) {
		pr_err("Cannot enable iTouch\n");
		return ret;
	}

	idv = kzalloc(sizeof(struct itouch_device), GFP_KERNEL);

	if (!idv)
		return -ENOMEM;

	ret = mei_cldev_register_event_cb(dev, BIT(MEI_CL_EVENT_RX),
					itouch_cl_event_cb, idv);

	if (ret  < 0)
		goto err;

	/*Setup the DMA BIT mask, the system will choose the best possible*/
	if (!dma_coerce_mask_and_coherent(&dev->dev, DMA_BIT_MASK(64))) {
		pr_info("ITouch using DMA_BIT_MASK(64)\n");
	} else if (!dma_coerce_mask_and_coherent(&dev->dev, DMA_BIT_MASK(32))) {
		pr_info("ITouch using  DMA_BIT_MASK(32)\n");
	} else {
		pr_err("ITouch: No suitable DMA available\n");
		ret=-EFAULT;
		goto err;
	}

	itouch_set_clientdata(dev, idv);
	idv->mdv = dev;
	idv->dev = &dev->dev;
	mutex_init(&event_handler);
	mutex_init(&multi_touch_mutex);
	ret = mei_hid_probe(idv);

	if(ret != 0)
		goto err;

	if(itouch_dbgfs_register(idv, "itouch"))
		itouch_dbg(idv, "cannot register debugfs\n");

	INIT_WORK(&idv->multi_touch_work, multi_touch_work);
	INIT_DELAYED_WORK(&idv->reacquire_db_work, reacquire_guc_db);
	mutex_init(&idv->mutex);

	/*we set display status on during init*/
	idv->display_status = true;
	/*Schedule the work, so that the state machine starts*/
	schedule_work(&idv->multi_touch_work);
	schedule_delayed_work(&idv->reacquire_db_work,
						REACQUIRE_DB_WORK_DELAY*2);

	if( ret < 0){
		goto err;
	}

	ret = sysfs_create_group(&dev->dev.kobj, &itouch_grp);
	if(ret){
		pr_err("Itouch: Failed to create sysfs\n");
	}

	return 0;
err:
	if(idv)
		kfree(idv);
	mei_cldev_disable(dev);
	return ret;
}

int itouch_mei_cl_remove(struct mei_cl_device *dev)
{
	struct itouch_device *idv = itouch_get_clientdata(dev);

	itouch_stop(idv);
	mei_hid_remove(idv);
	sysfs_remove_group(&idv->dev->kobj, &itouch_grp);
	itouch_dbgfs_deregister(idv);
	mei_cldev_disable(dev);

	return 0;
}

static struct mei_cl_driver itouch_mei_cl_driver = {
	.id_table = itouch_mei_cl_tbl,
	.name = ITOUCH_DRIVER_NAME,
	.probe = itouch_mei_cl_probe,
	.remove = itouch_mei_cl_remove,
};

static int mei_cl_itouch_init(void)
{
	int result;

	pr_info("Intel iTouch: %s() Called\n", __func__);

	result = mei_cldev_driver_register(&itouch_mei_cl_driver);
	if (result) {
		goto err_unreg;
	}

	pr_info("Intel iTouch:  %s() completed\n",__func__);
	return 0;

err_unreg:
	mei_cldev_driver_unregister(&itouch_mei_cl_driver);
	pr_err("Error in Registering ITouch client with MEI\n");
	return result;
}

static void __exit mei_cl_itouch_exit(void)
{
	printk(KERN_INFO ">> %s called\n", __func__);
	mei_cldev_driver_unregister(&itouch_mei_cl_driver);
//	itouch_exit();
}

int itouch_mei_cl_recv(struct mei_cl_device *cl, u8 * buf, size_t length)
{
	return mei_cldev_recv(cl, buf, length);
}

int itouch_mei_cl_send(struct mei_cl_device *cl, u8 * buf, size_t length,
		       bool blocking)
{
	return mei_cldev_send(cl, buf, length);
}

module_init(mei_cl_itouch_init);
module_exit(mei_cl_itouch_exit);

MODULE_DESCRIPTION
    ("Intel(R) Management Engine Interface Client Driver for Intel Precision Touch");
MODULE_LICENSE("GPL");
