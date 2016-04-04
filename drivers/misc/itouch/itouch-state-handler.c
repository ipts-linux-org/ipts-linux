/*
 * Intel Precision Touch HID Driver State Machine Manager
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
#include "itouch-gfx-interface.h"
#include "itouch-gfx-hid-interface.h"

int sensor_mode_handler(struct itouch_device *idv)
{
	int status = 0;

	if (idv->vendor_entries.reg_sensor_mode == TOUCH_SENSOR_MODE_HID ||
	    idv->vendor_entries.reg_sensor_mode == TOUCH_SENSOR_MODE_RAW_DATA) {

		if (idv->hid_state.comp_state == HID_MULTI_TOUCH_READY
				&& idv->vendor_entries.reg_sensor_mode ==
				TOUCH_SENSOR_MODE_RAW_DATA) {
			idv->driver_state.comp_state = DRIVER_RAW_DATA_MODE_READY;
			idv->sensor_mode = TOUCH_SENSOR_MODE_RAW_DATA;
		} else {
			idv->driver_state.comp_state = DRIVER_HID_MODE_READY;
			idv->sensor_mode = TOUCH_SENSOR_MODE_HID;
		}
	}

	return status;
}

int driver_state_handler(struct itouch_device *idv)
{
	int status = 0;

	if (idv->driver_state.comp_state == DRIVER_HW_INIT) {
		status = me_state_handler(idv);

		if (status != 0) {
			itouch_dbg(idv, "ME State hanlder error(%d)\n", status);
			idv->driver_state.comp_state = DRIVER_HW_INIT_ERROR;
			idv->driver_state.error_status = true;
			return status;
		}
	} else if (idv->driver_state.comp_state == DRIVER_HID_MODE_READY) {
		/*This check is only for debug purpose and does not come in regular
			exeuction. These statement can be hit when transistion is triggerd
			from sysfs interfacesi or during state transistion triggered due
			to error*/
		if (idv->hid_state.comp_state == HID_MULTI_TOUCH_READY) {
			idv->mode_transition = true;
			idv->hid_debug_log_data.mode_transitions++;
			itouch_dbg(idv,
				   "Beginning transition from single touch to multi touch. Number of transitions so far 0x%x \n",
				   idv->hid_debug_log_data.mode_transitions);
			me_state_handler(idv);
		}
		else {
			itouch_dbg(idv,
				"Either Gfx or HID is in an invalid state, hid_state.comp_state=0x%x\n",
				idv->hid_state.comp_state);

		}
	} else if (idv->driver_state.comp_state == DRIVER_RAW_DATA_MODE_READY) {
		/*This check is only for debug purpose and does not come in regular
			exeuction. These statement can be hit when transistion is triggerd
			from sysfs interfaces or during state transistion triggered due
			to error*/
		if (idv->hid_state.comp_state == HID_SINGLE_TOUCH_READY) {
			idv->mode_transition = true;
			idv->hid_debug_log_data.mode_transitions++;
			itouch_dbg(idv,
					"Beginning transition from multi touch to single touch. Number of transitions so far 0x%x \n",
					idv->hid_debug_log_data.mode_transitions);
			me_state_handler(idv);
		} else {
			itouch_dbg(idv,
					"Either Gfx or HID is in an invalid state, hid_state.comp_state=0x%x\n",
					idv->hid_state.comp_state);
		}
	} else {
		/*This is for debug purpose*/
		itouch_dbg(idv, "This is an unknown state\n");
		idv->driver_state.comp_state = DRIVER_STATE_UNKNOWN;
		idv->driver_state.error_status = true;
	}

	return status;
}

int me_state_handler(struct itouch_device *idv)
{
	int status = 0;

	/*If in STATE_NONE or in any other error state, reset the device and exit*/
	if ( (idv->me_state.comp_state == ME_STATE_NONE) ||
		 ((idv->me_state.comp_state & 0x80004000)== 0x80004000) ){

		idv->me_state.comp_state = ME_MEI_RESET_SEND;
		//In error state reset the device and wait for further
		//commands from debugfs or sysfs
		status = mei_hw_reset(idv->mdv->bus, true);
		if (status != 0) {
			itouch_dbg(idv, "Failed to reset itouch mei hw\n");
			idv->me_state.comp_state = ME_MEI_RESET_ERROR;
			idv->me_state.error_status = true;

			return status;
		}
		return status;

	} else if (idv->me_state.comp_state == ME_MEI_INT_READY ) {

		status = sensor_mode_handler(idv);
		status = hid_touch_heci2_interface(idv,
						(MEITOUCH_FEATURE_COMMANDS)TOUCH_IS_SENSOR_READY);

	} else if (idv->me_state.comp_state ==
		 ME_MEI_IS_SENSOR_READY_RESP_RECEIVED) {

		status = hid_touch_heci2_interface(idv,
						(MEITOUCH_FEATURE_COMMANDS)TOUCH_GET_DEVICE_INFO);
		idv->me_state.error_status = false;

	} else if (idv->me_state.comp_state == ME_MEI_CLEAR_MEM_RESP_RECEIVED) {

		//itouch_dbg(idv, "ME_MEI_CLEAR_MEM_RESP_RECEIVED state arrived\n");
		if (idv->sensor_mode == TOUCH_SENSOR_MODE_RAW_DATA) {

			itouch_dbg(idv,"GFX State is GFX_DETECTED\n");
			idv->graphics_state.comp_state = GFX_DETECTED;
			graphics_state_handler(idv);

		}else if (idv->sensor_mode == TOUCH_SENSOR_MODE_HID) {

			if (!idv->single_touch.memory_allocation_status) {
				status = single_touch_memory_init(idv);
				if (status != 0) {
					idv->me_state.comp_state = ME_MEI_SET_MEM_ERROR;
					idv->me_state.error_status = true;
					return status;
				}
			}
		}
		status = hid_touch_heci2_interface(idv,
						(MEITOUCH_FEATURE_COMMANDS)TOUCH_SET_SENSOR_MODE);
	} else if (idv->me_state.comp_state == ME_MEI_MODE_RESP_RECEIVED) {

		status = hid_touch_heci2_interface(idv,
							(MEITOUCH_FEATURE_COMMANDS)TOUCH_SET_MEM_INITIALIZATION);
		idv->mode_transition = false;

	} else if (idv->me_state.comp_state == ME_MEI_SET_MEM_RESP_RECEIVED) {

		status = hid_touch_heci2_interface(idv,
						(MEITOUCH_FEATURE_COMMANDS)TOUCH_SET_TOUCH_READY);

	} else if ( (idv->me_state.comp_state == ME_MEI_GET_DEVICE_INFO_RECEIVED) ||
				(idv->me_state.comp_state == ME_MEI_RESET_SEND)) {

		status = hid_touch_heci2_interface(idv,
						      (MEITOUCH_FEATURE_COMMANDS)TOUCH_CLEAR_MEM);

	} else if (idv->me_state.comp_state == ME_MEI_READY_FORDATA_RESP_RECEIVED) {

			status =  manage_single_touch_data_received_from_sensor(idv);

	} else if (idv->me_state.comp_state == ME_MEI_TOUCH_SENSOR_DISABLED) {

		if (idv->mode_transition) {
			itouch_dbg(idv, "ME_MEI_TOUCH_SENSOR_DISABLED state arrived\n");
			//idv->mode_transition = true;
			status = hid_touch_heci2_interface(idv,
						      (MEITOUCH_FEATURE_COMMANDS)TOUCH_CLEAR_MEM);

			/*Since in transistion and inverted driver state of perform Transistion*/
			if(idv->driver_state.comp_state == DRIVER_RAW_DATA_MODE_READY) {
				idv->hid_state.comp_state = HID_MULTI_TOUCH_READY;
			}
			else if (idv->driver_state.comp_state == DRIVER_HID_MODE_READY) {
				idv->hid_state.comp_state = HID_SINGLE_TOUCH_READY;
			}
			//Update the sensor states
			sensor_mode_handler(idv);
		} else {
			itouch_dbg(idv, "--ERROR-- ME_MEI_TOUCH_SENSOR_DISABLED state without Transistion\n");
			//TODO: Either unload the driver or send reset. Check ME Status and do it.
		}

	} else if (idv->me_state.comp_state == ME_MEI_FEEDBACK_RESP_RECEIVED) {

			/*Only in Single Touch mode, send TOUCH_SENSOR_HID_READY_FOR_DATA_CMD
					for the next touch data to arrive */
			if (idv->sensor_mode == TOUCH_SENSOR_MODE_HID) {
				status = hid_touch_heci2_interface(idv,
								(MEITOUCH_FEATURE_COMMANDS)TOUCH_SET_TOUCH_READY);
			}

	}

	return status;
}

int hid_state_handler(struct itouch_device *idv)
{
	int status = 0;
	return status;
}

int graphics_state_handler(struct itouch_device *idv)
{
	void *kernelspec = NULL;
	int size = 0, ret;
	const char *kernel_path;

	if (idv->graphics_state.comp_state == GFX_STATE_NONE) {
		//TODO: Error handling
		itouch_dbg(idv, "iTouch in GFX_STATE_NONE \n");
	} else if (idv->graphics_state.comp_state == GFX_DETECTED) {
		/* Step 1: Open GPU Context
		 * Step 2: Read the kernel spec file
		 * Step 3: Parse the spec and setup the kernel
		 */

		ret = itouch_open_gpu(idv);
		if (ret)
			return ret;

		kernel_path = ITOUCH_VENDOR_KERNEL_BLOB;
		size = itouch_gfx_read_kernelspec(idv, kernel_path, &kernelspec);
		if (!size || !kernelspec) {
			itouch_close_gpu(idv);
			return -ENOMEM;
		}

		ret = itouch_gfx_setup_kernel(idv, kernelspec, size);
		if (ret < 0) {
			goto mem_err;
		}

		//Allocate the multitouch feedback buffer
		ret = multi_touch_feedback_buff_init(idv);
        if (ret < 0) {
			goto mem_err;
        }

		itouch_dbg(idv, "GUC setup complete for iTouch\n");
		idv->multi_touch.memory_allocation_status = true;
		idv->graphics_state.comp_state = GFX_TOUCH_READY;

#if 0
		/*Code used for testing to perform submit the buffer to kernel manually.
			Not to be used in regular flow */
		for(count = 0; count < 4; count++)
		  manual_submit();
#endif
	}

	return ret;

mem_err:
    itouch_close_gpu(idv);
    free_memory_allocation(idv);
    idv->graphics_state.comp_state = ret;
    idv->graphics_state.error_status = true;
	return ret;
}
