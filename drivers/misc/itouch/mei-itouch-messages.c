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
#include "mei-itouch-client.h"
#include "itouch-state-codes.h"
#include "itouch-state-handler.h"
#include "itouch-gfx-hid-interface.h"

#ifdef DEBUG_PTOUCH
#include <linux/kthread.h>
#endif

const char *rsp_msg[16] = {
	"Invalid Response Code",
	"TOUCH_SENSOR_GET_DEVICE_INFO_RSP",
	"TOUCH_SENSOR_SET_MODE_RSP",
	"TOUCH_SENSOR_SET_MEM_WINDOW_RSP",
	"TOUCH_SENSOR_QUIESCE_IO_RSP",
	"TOUCH_SENSOR_HID_READY_FOR_DATA_RSP",
	"TOUCH_SENSOR_FEEDBACK_READY_RSP",
	"TOUCH_SENSOR_CLEAR_MEM_WINDOW_RSP",
	"TOUCH_SENSOR_NOTIFY_DEV_READY_RSP",
	"TOUCH_SENSOR_SET_POLICIES_RSP",
	"TOUCH_SENSOR_GET_POLICIES_RSP",
	"TOUCH_SENSOR_RESET_RSP",
	"TOUCH_SENSOR_READ_ALL_REGS_RSP",
	"TOUCH_SENSOR_GEN_TEST_PACKETS_RSP",
	"TOUCH_SENSOR_CMD_ERROR_RSP",
	"--LAST--",
};

const char* cmd_msg[16]={
    "Invalid Command Code",
    "TOUCH_SENSOR_GET_DEVICE_INFO_CMD",
    "TOUCH_SENSOR_SET_MODE_CMD",
    "TOUCH_SENSOR_SET_MEM_WINDOW_CMD",
    "TOUCH_SENSOR_QUIESCE_IO_CMD",
    "TOUCH_SENSOR_HID_READY_FOR_DATA_CMD",
    "TOUCH_SENSOR_FEEDBACK_READY_CMD",
    "TOUCH_SENSOR_CLEAR_MEM_WINDOW_CMD",
    "TOUCH_SENSOR_NOTIFY_DEV_READY_CMD",
    "TOUCH_SENSOR_SET_POLICIES_CMD",
    "TOUCH_SENSOR_GET_POLICIES_CMD",
    "TOUCH_SENSOR_RESET_CMD",
    "TOUCH_SENSOR_READ_ALL_REGS_CMD",
    "TOUCH_SENSOR_GEN_TEST_PACKETS_CMD",
    "Invalid Command Code",
	"",
};

int mei_itouch_write_message(struct itouch_device *idv, size_t len, char *buf)
{
	int count  = 0;

#if IS_ENABLED(CONFIG_INTEL_MEI_ITOUCH)
	count = itouch_mei_cl_send(idv->mdv, buf, len, 0);
#else
	printk("iTouch: calling h2m_write_msg()\n");
	count = h2m_write_msg(idv->mdv, len, buf);
#endif
	/* count >0 means write successful, else error.
	 * so propogate the error case to be handled further
	 */
	if (count > 0)
		count = 0;

	return count;
}

int mei_itouch_read_message(struct itouch_device *idv, size_t len, char *buf)
{
	int status = 0;
#if IS_ENABLED(CONFIG_INTEL_MEI_ITOUCH)
	status = itouch_mei_cl_recv(idv->mdv, buf, len);
#endif
	return 0;

}

int hid_touch_heci2_interface(struct itouch_device *idv,
			      MEITOUCH_FEATURE_COMMANDS type)
{
	int status = 0;

	switch (type) {
	case TOUCH_GET_DEVICE_INFO:
		status = touch_get_device_info(idv);
		break;

	case TOUCH_SET_MEM_INITIALIZATION:
		status = touch_set_mem_window(idv);
		break;

	case TOUCH_IS_SENSOR_READY:
		status = touch_is_sensor_ready(idv);
		break;

	case TOUCH_SET_SENSOR_MODE:
		status = touch_set_sensor_mode(idv);
		break;

	case TOUCH_SET_TOUCH_READY:
		status = touch_hid_driver_ready(idv);
		break;

	case TOUCH_CLEAR_MEM:
		status = touch_clear_mem_window(idv);
		break;

	case TOUCH_SET_POLICIES_MODE:
		status = set_log_mode(idv);
		break;
	default:
		{
			return EINVAL;
		}
	}

	return status;
}

int touch_is_sensor_ready(struct itouch_device *idv)
{
	int status = 0;
	TOUCH_SENSOR_MSG_H2M out_msg;
	size_t len = sizeof(out_msg.CommandCode);

	memset(&out_msg, 0, len);
	out_msg.CommandCode = TOUCH_SENSOR_NOTIFY_DEV_READY_CMD;
	itouch_dbg(idv, "Executing touch command %s\n",
				cmd_msg[out_msg.CommandCode]);

	status = mei_itouch_write_message(idv, len, (u8 *) & out_msg);
	if (status != 0) {
		itouch_dbg(idv, "mei write failed with status = %d\n", status);
		idv->me_state.comp_state = ME_MEI_IS_SENSOR_READY_ERROR;
		idv->me_state.error_status = true;

		return status;
	}

	idv->me_state.comp_state = ME_MEI_IS_SENSOR_READY_SEND;

	return status;
}

int touch_set_sensor_mode(struct itouch_device *idv)
{
	int status = 0;
	TOUCH_SENSOR_MSG_H2M out_msg;

	memset(&out_msg, 0, sizeof(out_msg));

	out_msg.CommandCode = TOUCH_SENSOR_SET_MODE_CMD;
	out_msg.H2MData.SetModeCmdData.SensorMode = idv->sensor_mode;
	itouch_dbg(idv, "Executing touch command %s, with Sensor Mode as %s\n",
					cmd_msg[out_msg.CommandCode],
					( (idv->sensor_mode) ? "Raw Mode (MultiTouch)":"HID Mode(SingleTouch)") );

	status =
	    mei_itouch_write_message(idv,
				     sizeof(out_msg.CommandCode) +
				     sizeof(TOUCH_SENSOR_SET_MODE_CMD_DATA),
				     (u8 *) & out_msg);
	if (status != 0) {
		itouch_dbg(idv, "mei write failed with status = %d\n", status);
		idv->me_state.comp_state = ME_MEI_MODE_ERROR;
		return status;
	}

	idv->me_state.comp_state = ME_MEI_MODE_SEND;

	return status;
}

int touch_get_device_info(struct itouch_device *idv)
{
	int status = 0;
	TOUCH_SENSOR_MSG_H2M out_msg;

	memset(&out_msg, 0, sizeof(out_msg));
	out_msg.CommandCode = TOUCH_SENSOR_GET_DEVICE_INFO_CMD;
	itouch_dbg(idv, "Executing touch command %s\n",
					cmd_msg[out_msg.CommandCode]);

	// send the command to touch firmware
	status =
	    mei_itouch_write_message(idv, sizeof(out_msg.CommandCode),
				     (u8 *) & out_msg);

	if (status != 0) {
		itouch_dbg(idv, "mei write failed with status = %d\n", status);
		idv->me_state.comp_state = ME_MEI_GET_DEVICE_INFO_ERROR;
		idv->me_state.error_status = true;

		return status;
	}

	idv->me_state.comp_state = ME_MEI_GET_DEVICE_INFO_SEND;

	return status;
}

int touch_set_mem_window(struct itouch_device *idv)
{
	int status = 0;
	TOUCH_SENSOR_MSG_H2M out_msg;
	u32 touch_data_buf_index;

    //Allocate ME<->HID buffer
    idv->hid_me_buffer = dma_alloc_coherent(idv->dev,
                        idv->sensor_data.FeedbackSize,
                         &idv->phy_hid_me_buff,
                        GFP_ATOMIC|GFP_DMA32);

    idv->me_hid_report_buff =
        kzalloc(idv->sensor_data.FeedbackSize, GFP_KERNEL);

    if (idv->me_hid_report_buff == NULL) {
        //TODO: Error handling on memory allocation failure
        itouch_dbg(idv,
               "The memory allocation for  ME2HIDReportCompletionBuffer Buffer failed \n");
		idv->me_state.comp_state = ME_MEI_SET_MEM_ERROR;
        idv->me_state.error_status = true;
        return -ENOMEM;
    }

	memset(&out_msg, 0, sizeof(out_msg));

	out_msg.CommandCode = TOUCH_SENSOR_SET_MEM_WINDOW_CMD;
	itouch_dbg(idv, "Executing touch command %s\n",
		cmd_msg[out_msg.CommandCode]);

	if (idv->sensor_mode == TOUCH_SENSOR_MODE_RAW_DATA) {
		out_msg.H2MData.SetMemWindowCmdData.WorkQueueItemSize =
				idv->work_queue_item_size;
		out_msg.H2MData.SetMemWindowCmdData.WorkQueueSize =
				idv->work_queue_size;

		out_msg.H2MData.SetMemWindowCmdData.TailOffsetAddrLower =
						lower_32_bits(idv->TailPtrAddr);
		out_msg.H2MData.SetMemWindowCmdData.TailOffsetAddrUpper =
						upper_32_bits(idv->TailPtrAddr);

		out_msg.H2MData.SetMemWindowCmdData.DoorbellCookieAddrLower =
			lower_32_bits(idv->DoorbellCookieAddr);
		out_msg.H2MData.SetMemWindowCmdData.DoorbellCookieAddrUpper =
			upper_32_bits(idv->DoorbellCookieAddr);

		itouch_info(idv,
			"Doorbell Cookie Address Lower: 0x%x. Upper: 0x%x\n",
			out_msg.H2MData.SetMemWindowCmdData.
			DoorbellCookieAddrLower,
			out_msg.H2MData.SetMemWindowCmdData.
			DoorbellCookieAddrUpper);
		itouch_info(idv, "TailOffset Address Lower: 0x%x. Upper: 0x%x\n",
			out_msg.H2MData.SetMemWindowCmdData.
			TailOffsetAddrLower,
			out_msg.H2MData.SetMemWindowCmdData.
			TailOffsetAddrUpper);


		for (touch_data_buf_index = 0;
				touch_data_buf_index < TOUCH_SENSOR_MAX_DATA_BUFFERS;
				touch_data_buf_index++) {

			if (touch_data_buf_index >= HID_PARALLEL_DATA_BUFFERS) {
				out_msg.H2MData.SetMemWindowCmdData.
					TouchDataBufferAddrLower[touch_data_buf_index] = 0;
				out_msg.H2MData.SetMemWindowCmdData.
					TouchDataBufferAddrUpper[touch_data_buf_index] = 0;

			} else {

				out_msg.H2MData.SetMemWindowCmdData.
					TouchDataBufferAddrLower[touch_data_buf_index] =
						lower_32_bits(idv->
						touch_buffer[touch_data_buf_index].
						phy_raw_touch_data_buffer);
				out_msg.H2MData.SetMemWindowCmdData.
					TouchDataBufferAddrUpper[touch_data_buf_index] =
				upper_32_bits(idv->
						touch_buffer[touch_data_buf_index].
						phy_raw_touch_data_buffer);
				itouch_dbg(idv,
					"(RAW)TouchDataBuffer Address Lower: 0x%x. Upper: 0x%x\n",
					out_msg.H2MData.SetMemWindowCmdData.
					TouchDataBufferAddrLower[touch_data_buf_index],
					out_msg.H2MData.SetMemWindowCmdData.
					TouchDataBufferAddrUpper[touch_data_buf_index]);
			}
		}

		for (touch_data_buf_index = 0;
			touch_data_buf_index < TOUCH_SENSOR_MAX_DATA_BUFFERS;
			 touch_data_buf_index++) {

		if (touch_data_buf_index >= HID_PARALLEL_DATA_BUFFERS) {
			out_msg.H2MData.SetMemWindowCmdData.
			    FeedbackBufferAddrLower[touch_data_buf_index] = 0;
			out_msg.H2MData.SetMemWindowCmdData.
			    FeedbackBufferAddrUpper[touch_data_buf_index] = 0;
		} else {
			out_msg.H2MData.SetMemWindowCmdData.
				FeedbackBufferAddrLower[touch_data_buf_index] =
					lower_32_bits(idv->
					touch_buffer[touch_data_buf_index].phy_feedback_buffer);
			out_msg.H2MData.SetMemWindowCmdData.
			FeedbackBufferAddrUpper[touch_data_buf_index] =
					upper_32_bits(idv->
					touch_buffer[touch_data_buf_index].phy_feedback_buffer);
				itouch_dbg(idv,
					"FeedbackBuffer Address Lower: 0x%x. Upper: 0x%x\n",
					out_msg.H2MData.SetMemWindowCmdData.
					FeedbackBufferAddrLower
					[touch_data_buf_index],
					out_msg.H2MData.SetMemWindowCmdData.
					FeedbackBufferAddrUpper
					[touch_data_buf_index]);
			}
		}
	} else {
		/*Set memory pointers for HID_MODE, only one set of feedback
			and touch data bufer*/
        out_msg.H2MData.SetMemWindowCmdData.TouchDataBufferAddrLower[0] =
                lower_32_bits(idv->touch_buffer[0].phy_hid_touch_buffer);
        out_msg.H2MData.SetMemWindowCmdData.TouchDataBufferAddrUpper[0] =
                upper_32_bits(idv->touch_buffer[0].phy_hid_touch_buffer);

		itouch_dbg(idv,
			"HID Mode-TouchDataBuffer Address Lower: 0x%x. Upper: 0x%x\n",
            out_msg.H2MData.SetMemWindowCmdData.TouchDataBufferAddrLower[0],
            out_msg.H2MData.SetMemWindowCmdData.TouchDataBufferAddrUpper[0]);

        out_msg.H2MData.SetMemWindowCmdData.FeedbackBufferAddrLower[0] =
                   lower_32_bits(idv->touch_buffer[0].phy_hid_feedback_buffer);
        out_msg.H2MData.SetMemWindowCmdData.FeedbackBufferAddrUpper[0] =
                   upper_32_bits(idv->touch_buffer[0].phy_hid_feedback_buffer);
        itouch_dbg(idv,
					"HID Mode-FeedbackBuffer Address Lower: 0x%x. Upper: 0x%x\n",
					out_msg.H2MData.SetMemWindowCmdData.FeedbackBufferAddrLower[0],
					out_msg.H2MData.SetMemWindowCmdData.FeedbackBufferAddrUpper[0]);

	}

	out_msg.H2MData.SetMemWindowCmdData.Hid2MeBufferSize =
	    idv->sensor_data.FeedbackSize;
	out_msg.H2MData.SetMemWindowCmdData.Hid2MeBufferAddrLower =
	    lower_32_bits(idv->phy_hid_me_buff);
	out_msg.H2MData.SetMemWindowCmdData.Hid2MeBufferAddrUpper =
	    upper_32_bits(idv->phy_hid_me_buff);

	itouch_dbg(idv, "Hid2MeBufferSize =0x%x, Hid2MeBuffer Address Lower: 0x%x. Upper: 0x%x\n",
		   out_msg.H2MData.SetMemWindowCmdData.Hid2MeBufferSize,
		   out_msg.H2MData.SetMemWindowCmdData.Hid2MeBufferAddrLower,
		   out_msg.H2MData.SetMemWindowCmdData.Hid2MeBufferAddrUpper);

	status = mei_itouch_write_message(idv,
				     sizeof(out_msg.CommandCode) +
				     sizeof
				     (TOUCH_SENSOR_SET_MEM_WINDOW_CMD_DATA),
				     (u8 *) & out_msg);

	if (status != 0) {
		itouch_dbg(idv, "mei write failed with status = %d\n", status);
		idv->me_state.comp_state = ME_MEI_SET_MEM_ERROR;
		idv->me_state.error_status = true;

		return status;
	}

	idv->me_state.comp_state = ME_MEI_SET_MEM_SEND;
	return status;
}

#ifdef DEBUG_PTOUCH
static int touch_debug_thread(void *data)
{
	struct itouch_device *idv = (struct itouch_device *)data;
#ifdef DUMP_BUF
	int i, j;
#endif
	char fw_sts_str[MEI_FW_STATUS_STR_SZ];
	volatile union guc_doorbell_qw *dbi;
	dbi = (union guc_doorbell_qw *)idv->appPrcocessParams->db_base_addr;
	pr_info(">> start debug thread\n");
	while (1) {
                mei_fw_status_str(idv->mdv->bus, fw_sts_str, MEI_FW_STATUS_STR_SZ);
		pr_info(">> tdt : fw status : %s\n", fw_sts_str);

#ifdef DUMP_BUF
		for (i = 0; i < HID_PARALLEL_DATA_BUFFERS; i++) {
			pr_info(">> == input(%p)[%d] == \n",
				idv->touch_buffer[i].raw_touch_data_buffer, i);
			print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 32, 1,
					idv->touch_buffer[i].raw_touch_data_buffer,
					DUMP_BYTES_OF_BUF, false);
                }

		for (i = 0; i < HID_PARALLEL_DATA_BUFFERS; i++) {
			for (j = 0; j < idv->num_output_buffers; j++) {
				pr_info(">> == output(%p)[%d][%d] == \n",
					idv->touch_buffer[i].processed_touch_buffer[j], i, j);
				print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 32, 1,
						idv->touch_buffer[i].processed_touch_buffer[j],
						DUMP_BYTES_OF_BUF, false);
			}
                }
#endif

		pr_info(">> == DB s:%x, c:%x ==\n", dbi->db_status, dbi->cookie);
		pr_info(">> == WQ h:%u, t:%u ==\n", idv->appPrcocessParams->head, idv->appPrcocessParams->tail);
		pr_info(">> == WQ s:%u, eng:%u ==\n",idv->appPrcocessParams->wq_status, idv->appPrcocessParams->engine_presence);
		pr_info(">> == WQ prio: %u \n",idv->appPrcocessParams->priority);

		i915_print_itouch_guc_info(idv->appPrcocessParams);

		msleep(3000);
	}

	return 0;
}
#endif

int touch_sensor_quiesce_io(struct itouch_device *idv)
{
	int status = 0;
	TOUCH_SENSOR_MSG_H2M out_msg;

	memset(&out_msg, 0, sizeof(out_msg));
	out_msg.CommandCode = TOUCH_SENSOR_QUIESCE_IO_CMD;
	itouch_dbg(idv, "Executing touch command %s\n",
                                        cmd_msg[out_msg.CommandCode]);

	/* send the command to touch firmware*/
	status = mei_itouch_write_message(idv, sizeof(out_msg.CommandCode) +
				sizeof(TOUCH_SENSOR_QUIESCE_IO_CMD_DATA),
				(u8 *) & out_msg);

	if (status != 0) {
		itouch_dbg(idv, "mei write failed with status = %d\n", status);
		idv->me_state.comp_state = ME_MEI_GET_DEVICE_INFO_ERROR;
		idv->me_state.error_status = true;

		return status;
	}

	idv->me_state.comp_state = ME_MEI_SENSOR_QUIESCE_IO_CMD_SEND;

	return status;
}

int touch_hid_driver_ready(struct itouch_device *idv)
{
	int status = 0;
	TOUCH_SENSOR_MSG_H2M out_msg;

	memset(&out_msg, 0, sizeof(out_msg));

	out_msg.CommandCode = TOUCH_SENSOR_HID_READY_FOR_DATA_CMD;
	itouch_dbg(idv, "Executing touch command %s\n",
						cmd_msg[out_msg.CommandCode]);

	status = mei_itouch_write_message(idv, sizeof(out_msg.CommandCode), (u8 *) & out_msg);

	if (status != 0) {
		itouch_dbg(idv, "mei write failed with status = %d\n", status);
		idv->me_state.comp_state = ME_MEI_READY_FORDATA_ERROR;
		idv->me_state.error_status = true;
		return status;
	}

	idv->me_state.comp_state = ME_MEI_TOUCH_READY;
#ifdef DEBUG_PTOUCH
	{
		char fw_sts_str[MEI_FW_STATUS_STR_SZ];
		msleep(300);
                mei_fw_status_str(idv->mdv->bus, fw_sts_str, MEI_FW_STATUS_STR_SZ);
		pr_info(">> fw status after asking data: %s\n", fw_sts_str);
	}

	if (idv->sensor_mode == TOUCH_SENSOR_MODE_RAW_DATA) {
		touch_debug_thread((void*)idv);
		kthread_run(touch_debug_thread, (void*)idv, "itouch_dbg");
	}
#endif
	return status;
}

int touch_feedback_ready(struct itouch_device *idv, u8 feedback_buf_index, u32 transaction_id)
{
	int status = 0;
	TOUCH_SENSOR_MSG_H2M out_msg;

	if (feedback_buf_index > TOUCH_SENSOR_MAX_DATA_BUFFERS) {
		itouch_dbg(idv, "Feedback Buffer Index out of range: %d\n",
			   feedback_buf_index);
		return -1;
	}

	memset(&out_msg, 0, sizeof(out_msg));

	out_msg.CommandCode = TOUCH_SENSOR_FEEDBACK_READY_CMD;
	out_msg.H2MData.FeedbackReadyCmdData.FeedbackIndex = feedback_buf_index;
	out_msg.H2MData.FeedbackReadyCmdData.TransactionId = transaction_id;
	itouch_dbg(idv, "Executing touch command %s for Feedback Buffindex 0x%x\n", cmd_msg[out_msg.CommandCode],
						 feedback_buf_index);
	status = mei_itouch_write_message(idv,
				     sizeof(out_msg.CommandCode) +
				     sizeof
				     (TOUCH_SENSOR_FEEDBACK_READY_CMD_DATA),
				     (u8 *) & out_msg);

	if (status != 0)
		itouch_dbg(idv, "mei write failed with status = %d\n", status);

	idv->me_state.comp_state = ME_MEI_FEEDBACK_SEND;
	return status;
}

int touch_clear_mem_window(struct itouch_device *idv)
{
	int status = 0;
	TOUCH_SENSOR_MSG_H2M out_msg;

	memset(&out_msg, 0, sizeof(out_msg));

	out_msg.CommandCode = TOUCH_SENSOR_CLEAR_MEM_WINDOW_CMD;
	itouch_dbg(idv, "Executing touch command %s\n",
					cmd_msg[out_msg.CommandCode]);

	// send the command to touch firmware
	status =
	    mei_itouch_write_message(idv, sizeof(out_msg.CommandCode),
				     (u8 *) & out_msg);

	if (status != 0) {
		itouch_dbg(idv, "mei write failed with status = %d\n", status);
		idv->me_state.comp_state = ME_MEI_CLEAR_MEM_ERROR;
		idv->me_state.error_status = true;

		return status;
	}

	idv->me_state.comp_state = ME_MEI_CLEAR_MEM_SEND;
	return status;
}

int read_mei_messages(struct itouch_device *idv, TOUCH_SENSOR_MSG_M2H in_msg,
		      u32 ilength)
{
	int status = 0;

	itouch_dbg(idv, "Touch command response %s received\n",
				rsp_msg[in_msg.CommandCode&0x0000000F]);

	switch (in_msg.CommandCode) {
	case TOUCH_SENSOR_NOTIFY_DEV_READY_RSP:
		{
			status = parse_touch_sensor_notify_device_dev_ready_rsp_data
			    (idv, in_msg, ilength);
			break;
		}

	case TOUCH_SENSOR_GET_DEVICE_INFO_RSP:
		{
			status = parse_touch_sensor_get_device_info_rsp_data(idv,
									in_msg,
									ilength);
			break;
		}

	case TOUCH_SENSOR_SET_MEM_WINDOW_RSP:
		{
			status = parse_touch_sensor_set_mem_window_rsp(idv,
								  in_msg,
								  ilength);
			break;
		}

	case TOUCH_SENSOR_HID_READY_FOR_DATA_RSP:
		{
			status = parse_touch_sensor_HID_ready_for_data_resp(idv,
								       in_msg,
								       ilength);
			break;
		}

	case TOUCH_SENSOR_FEEDBACK_READY_RSP:
		{
			status = parse_touch_sensor_feedback_ready_resp(idv,
								   in_msg,
								   ilength);
			break;
		}

	case TOUCH_SENSOR_SET_POLICIES_RSP:
		{
			status = parse_touch_sensor_set_perf_mode_resp(idv,
								  in_msg,
								  ilength);
			break;
		}


	case TOUCH_SENSOR_SET_MODE_RSP:
		{
			status =
			    parse_touch_sensor_set_mode_resp(idv, in_msg,
							     ilength);
			break;
		}

	case TOUCH_SENSOR_CLEAR_MEM_WINDOW_RSP:
		{
			status =
			    parse_touch_sensor_clear_mem_window_rsp_data(idv,
									 in_msg,
									 ilength);
			break;
		}
	case TOUCH_SENSOR_QUIESCE_IO_RSP:
		{
			status = parse_touch_sensor_quiesce_io_rsp(idv, in_msg,
								ilength);
			break;
		}

#if 0
#	venkat review - case does not occur
#	case HECI_TOUCH_GET_PROTOCOL_RESPONSE:
#		{
#			status =
#			    parse_touch_client_details(idv, in_msg,
#						       ilength);
#			break;
#		}
#endif
	case TOUCH_SENSOR_CMD_ERROR_RSP:
	default:
		{
			itouch_dbg(idv,
				   "ME sent invalid command. ME didnt understand the message from HID 0x%x \n",
				   in_msg.CommandCode);
			return status;
		}
	}

	if (status != 0) {
		if (idv->me_state.error_status)
			itouch_dbg(idv, "ME Response received with error. \n");
	}

	status = me_state_handler(idv);

	return status;
}

int parse_touch_sensor_quiesce_io_rsp(struct itouch_device *idv,
                                                TOUCH_SENSOR_MSG_M2H in_msg,
                                                u32 ilength)
{
	int status = 0;
	u32 Expectedilength;

	Expectedilength = sizeof(in_msg.CommandCode) +
				sizeof(TOUCH_SENSOR_QUIESCE_IO_RSP_DATA);

	if (ilength != Expectedilength) {
		itouch_dbg(idv,
			"%s - Incorrect response size: expected %d, got %d\n",
				rsp_msg[in_msg.CommandCode&0x0000000F],
				Expectedilength, ilength);
		status = -1;
	}

	if (in_msg.M2HData.QuiesceIoRspData.Status != 0) {
		itouch_dbg(idv,
			"%s - Bad response status: expected %d, got %d\n",
				rsp_msg[in_msg.CommandCode&0x0000000F],
				0, in_msg.M2HData.QuiesceIoRspData.Status);
		status = -1;

		if(TOUCH_STATUS_COMPAT_CHECK_FAIL ==
				in_msg.M2HData.QuiesceIoRspData.Status) {
			itouch_dbg(idv,
			"Continuing with error code as its not non-fatal\n");
			status = 0;
		}
	}

	if (status != 0) {
		idv->me_state.comp_state = ME_MEI_SENSOR_QUIESCE_IO_ERROR;
		idv->me_state.error_status = true;
	} else
		idv->me_state.comp_state =
				ME_MEI_SENSOR_QUIESCE_IO_RSP_RECEIVED;

	return status;

}

int parse_touch_sensor_get_device_info_rsp_data(struct itouch_device *idv,
						TOUCH_SENSOR_MSG_M2H in_msg,
						u32 ilength)
{
	int status = 0;
	u32 Expectedilength;

	Expectedilength = sizeof(in_msg.CommandCode) +
					    sizeof(TOUCH_SENSOR_GET_DEVICE_INFO_RSP_DATA);

	if (ilength != Expectedilength) {
		itouch_dbg(idv,
				"%s - Incorrect response size: expected %d, got %d\n",
				rsp_msg[in_msg.CommandCode&0x0000000F],
				Expectedilength, ilength);
		status = -1;
	}

	if (in_msg.M2HData.DeviceInfoRspData.Status != 0) {
		itouch_dbg(idv,
				"%s - Bad response status: expected %d, got %d\n",
				rsp_msg[in_msg.CommandCode&0x0000000F],
			    0, in_msg.M2HData.DeviceInfoRspData.Status);
			status = -1;

			if(TOUCH_STATUS_COMPAT_CHECK_FAIL ==
					in_msg.M2HData.DeviceInfoRspData.Status) {
				itouch_dbg(idv, "Continuing with error code as its not non-fatal\n");
				status = 0;
			}
	}

	if (status != 0) {
		idv->me_state.comp_state = ME_MEI_GET_DEVICE_INFO_ERROR;
		idv->me_state.error_status = true;
	} else {
			memcpy(&(idv->sensor_data),
						&(in_msg.M2HData.DeviceInfoRspData),
						sizeof(TOUCH_SENSOR_GET_DEVICE_INFO_RSP_DATA));

			printk(KERN_ERR "Itouch Dev Info:VID:0x%04X, DevID:0x%04X ",
					idv->sensor_data.VendorId, idv->sensor_data.DeviceId);
			printk(KERN_ERR "HWRev:0x%x, FWRev:0x%x Framesize=0x%08x ",
				    idv->sensor_data.HwRev, idv->sensor_data.FwRev,
					idv->sensor_data.FrameSize);
			printk(KERN_ERR "Feedback Size: 0x%08X, SensorMode=0x%08X ",
                    idv->sensor_data.FeedbackSize,
					idv->sensor_data.SensorMode);
			printk(KERN_ERR "MaxTouchPoints=0x%x\n",
					idv->sensor_data.MaxTouchPoints);

		idv->me_state.comp_state = ME_MEI_GET_DEVICE_INFO_RECEIVED;
		status = 0;
	}

	return status;
}

int parse_touch_sensor_set_mem_window_rsp(struct itouch_device *idv,
					  TOUCH_SENSOR_MSG_M2H in_msg,
					  u32 ilength)
{
	int status = 0;
	u32 Expectedilength;

	Expectedilength =
			sizeof(in_msg.CommandCode) +
			sizeof(TOUCH_SENSOR_SET_MEM_WINDOW_RSP_DATA);
	if (ilength != Expectedilength) {
		itouch_dbg(idv,
			"%s - Incorrect response size: expected %d, got %d\n",
				rsp_msg[in_msg.CommandCode&0x0000000F],
			Expectedilength, ilength);
		status = -1;
	}

	if (in_msg.M2HData.SetMemWindowRspData.Status != 0) {
		itouch_dbg(idv,
			"%s - Bad response status: expected %d, got %d\n",
				rsp_msg[in_msg.CommandCode&0x0000000F],
			0, in_msg.M2HData.SetMemWindowRspData.Status);
			status = -1;

			if( TOUCH_STATUS_COMPAT_CHECK_FAIL == in_msg.M2HData.SetMemWindowRspData.Status){
				itouch_dbg(idv, "Continuing with error code as its not non-fatal\n");

			status = 0;
		}
	}

	if (status != 0) {
		idv->me_state.comp_state = ME_MEI_SET_MEM_ERROR;
		idv->me_state.error_status = true;
	} else {
		idv->me_state.comp_state = ME_MEI_SET_MEM_RESP_RECEIVED;
		status = 0;
	}

	return status;
}

int parse_touch_sensor_set_mode_resp(struct itouch_device *idv,
						TOUCH_SENSOR_MSG_M2H in_msg,
						u32 ilength)
{
	int status = 0;
	u32 Expectedilength;

	// Set Memory Response
	Expectedilength =
			sizeof(in_msg.CommandCode) +
			sizeof(TOUCH_SENSOR_SET_MODE_RSP_DATA);
	if (ilength != Expectedilength) {
		itouch_dbg(idv,
			"%s - Incorrect response size: expected %d, got %d\n",
				rsp_msg[in_msg.CommandCode&0x0000000F],
			Expectedilength, ilength);
		status = -1;
	}

	if (in_msg.M2HData.SetMemWindowRspData.Status != 0) {
		itouch_dbg(idv,
			"%s - Bad response status: expected %d, got %d\n",
				rsp_msg[in_msg.CommandCode&0x0000000F],
			0, in_msg.M2HData.SetMemWindowRspData.Status);
			status = -1;

			if( TOUCH_STATUS_COMPAT_CHECK_FAIL == in_msg.M2HData.SetMemWindowRspData.Status){
				itouch_dbg(idv, "Continuing with error code as its not non-fatal\n");

			status = 0;
		}

	}

	if (status != 0) {
		idv->me_state.comp_state = ME_MEI_MODE_ERROR;
		idv->me_state.error_status = true;
	} else {
		idv->me_state.comp_state = ME_MEI_MODE_RESP_RECEIVED;
		status = 0;
	}

	return status;
}

int parse_touch_sensor_notify_device_dev_ready_rsp_data(struct itouch_device
							*idv,
							TOUCH_SENSOR_MSG_M2H
							in_msg, u32 ilength)
{
	int status = 0;
	u32 Expectedilength;

	Expectedilength =
	    sizeof(in_msg.CommandCode) +
	    sizeof(TOUCH_SENSOR_NOTIFY_DEV_READY_RSP_DATA);

	if (ilength != Expectedilength) {
		itouch_dbg(idv,
				"%s - Incorrect response size: expected %d, got %u\n",
				rsp_msg[in_msg.CommandCode&0x0000000F],
				Expectedilength, ilength);
		status = -1;
	}

	if (in_msg.M2HData.SetMemWindowRspData.Status != 0) {
		itouch_dbg(idv,
			"%s - Bad response status: expected %d, got %d\n",
				rsp_msg[in_msg.CommandCode&0x0000000F],
				0, in_msg.M2HData.SetMemWindowRspData.Status);
		status = -1;
	}

	if (status != 0) {
		idv->me_state.comp_state = ME_MEI_IS_SENSOR_READY_ERROR;
		idv->me_state.error_status = true;
	} else {
		// Handling TOUCH_STATUS_SENSOR_EXPECTED_RESET event
		idv->me_state.error_status = false;
		idv->me_state.comp_state =
		    ME_MEI_IS_SENSOR_READY_RESP_RECEIVED;
		status = 0;
	}


	return status;
}

int parse_touch_sensor_clear_mem_window_rsp_data(struct itouch_device *idv,
						 TOUCH_SENSOR_MSG_M2H in_msg,
						 u32 ilength)
{
	int status = 0;
	u32 Expectedilength;

	// Set TOUCH_SENSOR_CLEAR_MEM_WINDOW_RSP_DATA Response
	Expectedilength =
			sizeof(in_msg.CommandCode) +
			sizeof(TOUCH_SENSOR_CLEAR_MEM_WINDOW_RSP_DATA);
	if (ilength != Expectedilength) {
		itouch_dbg(idv,
				"%s - Incorrect response size: expected %d, got %d\n",
				rsp_msg[in_msg.CommandCode&0x0000000F],
				Expectedilength, ilength);
		status = -1;
	}

	if (in_msg.M2HData.ClearMemWindowRspData.Status != 0) {
		itouch_dbg(idv,
				"%s - Bad response status: expected %d, got %d\n",
				rsp_msg[in_msg.CommandCode&0x0000000F],
				0, in_msg.M2HData.ClearMemWindowRspData.Status);
		status = -1;
	}

	if (status != 0) {
		idv->me_state.comp_state = ME_MEI_CLEAR_MEM_ERROR;
		idv->me_state.error_status = true;
	} else {
		idv->me_state.comp_state = ME_MEI_CLEAR_MEM_RESP_RECEIVED;
		status = 0;
	}

	return status;
}

int parse_touch_sensor_HID_ready_for_data_resp(struct itouch_device *idv,
					       TOUCH_SENSOR_MSG_M2H in_msg,
					       u32 ilength)
{
	int status = 0;
	u32 Expectedilength;

	// process response message
	Expectedilength =
				sizeof(in_msg.CommandCode) +
				sizeof(TOUCH_SENSOR_HID_READY_FOR_DATA_RSP_DATA);
	if (ilength != Expectedilength) {
		itouch_dbg(idv,
				"%s - Incorrect response size: expected %d, got %d\n",
				rsp_msg[in_msg.CommandCode&0x0000000F],
				Expectedilength, ilength);
		idv->me_state.comp_state = ME_MEI_READY_FORDATA_ERROR;
		idv->me_state.error_status = true;
		return -1;
	}

	if (in_msg.M2HData.HidReadyForDataRspData.Status != 0) {
		if (in_msg.M2HData.HidReadyForDataRspData.Status ==
					TOUCH_STATUS_SENSOR_DISABLED) {
			itouch_dbg(idv,
					"%s - status. This is not an error condition \n",
					rsp_msg[in_msg.CommandCode&0x0000000F]);
			idv->me_state.comp_state = ME_MEI_TOUCH_SENSOR_DISABLED;
			status = 0;
			return status;
		}
		else if( TOUCH_STATUS_COMPAT_CHECK_FAIL == in_msg.M2HData.HidReadyForDataRspData.Status){
						itouch_dbg(idv, "Continuing with error code as its not non-fatal\n");
						status = 0;
		}
		else {
			itouch_dbg(idv,
					"Bad response status for TOUCH_SENSOR_HID_READY_FOR_DATA_RSP_DATA: expected %d, got %d, reset reason=%d\n",
					0,
					in_msg.M2HData.HidReadyForDataRspData.Status,
					in_msg.M2HData.HidReadyForDataRspData.ResetReason);
			idv->me_state.comp_state = ME_MEI_READY_FORDATA_ERROR;
			idv->me_state.error_status = true;

			if(in_msg.M2HData.HidReadyForDataRspData.Status == TOUCH_STATUS_SENSOR_UNEXPECTED_RESET ||
					in_msg.M2HData.HidReadyForDataRspData.Status == TOUCH_STATUS_SENSOR_EXPECTED_RESET ){

				int i,j;
				idv->me_state.comp_state = ME_MEI_RESET_ERROR;
				//Reset the processed touch data buffers to avoid any spurious OS-Driver communication.
				//Reset the first 10 bytes which will include the header.
				 for (i = 0; i < HID_PARALLEL_DATA_BUFFERS; i++)
					  for(j=0; j<idv->num_output_buffers; j++)
						  memset(idv->touch_buffer[i].processed_touch_buffer[j], 0, 10);
			}

			return -1;
		}
	}

	idv->me_state.comp_state = ME_MEI_READY_FORDATA_RESP_RECEIVED;
	idv->present_data_fsize = in_msg.M2HData.HidReadyForDataRspData.DataSize;

	idv->touch_buffer_index = in_msg.M2HData.HidReadyForDataRspData.TouchDataBufferIndex;
	return status;
}

int complete_touch_operations(struct itouch_device *idv)
{
	return post_data_received_from_sensor(idv);
}

int finish_touch_operations(struct itouch_device *idv)
{
	int status = 0;

	if (idv->driver_state.comp_state == DRIVER_HID_MODE_READY) {
		//TODO: There is now no single touch data received.
		//This has to be changed to raw data received and handled accordingly
		status = manage_single_touch_data_received_from_sensor(idv);
	} else if (idv->driver_state.comp_state == DRIVER_RAW_DATA_MODE_READY) {
		status = manage_multi_touch_data_received(idv);
	} else {
		itouch_dbg(idv, "We are not ready to handle the data returned from ME for this sensor. \n");
	}

	return status;
}

int write_raw_data_to_file(struct itouch_device *idv)
{
	int status = 0;
	return status;
}

int manage_single_touch_data_received_from_sensor(struct itouch_device *idv)
{
	int status = 0;
	u32 touch_data_buf_index;
	u32 transaction_id;
	TOUCH_FEEDBACK_HDR HID2MEBuffer = { 0 };

	touch_data_buf_index = idv->touch_buffer_index;

	if (touch_data_buf_index < HID_PARALLEL_DATA_BUFFERS) {

		status = send_single_touch_to_os(idv, touch_data_buf_index);

		if(status == 0) {
			HID2MEBuffer.BufferId = touch_data_buf_index;
			HID2MEBuffer.FeedbackCmdType =
			    TOUCH_FEEDBACK_CMD_TYPE_NONE;
			HID2MEBuffer.PayloadSizeBytes = 0;
			HID2MEBuffer.ProtocolVer = 0;
			HID2MEBuffer.Reserved[0] = 0xAC;

			memcpy(idv->touch_buffer[touch_data_buf_index].
			       hid_feedback_buffer, &HID2MEBuffer,
			       sizeof(HID2MEBuffer));

			transaction_id = ((TOUCH_RAW_DATA_HDR *)idv->
				touch_buffer[touch_data_buf_index].hid_touch_buffer)->HidPrivateData.TransactionId;

			status =
			    touch_feedback_ready(idv, (u8) touch_data_buf_index,
								transaction_id);
			idv->hid_debug_log_data.NonGfxMode.FeedbackData++;
		}
	} else {
		itouch_dbg(idv, "Invalid touch_buffer_index 0x%x",
			   touch_data_buf_index);
	}

	return status;
}

int post_data_received_from_sensor(struct itouch_device *idv)
{
	int status = 0;
	u32 touch_data_buf_index;
	touch_data_buf_index = idv->touch_buffer_index;
	itouch_dbg(idv,
		   "Sending Vendor data. The data was received at the buffer with Index %d",
		   touch_data_buf_index);

	if (touch_data_buf_index != -1) {
		write_raw_data_to_file(idv);
	} else {
		itouch_dbg(idv, "Invalid touch_buffer_index 0x%x",
			   touch_data_buf_index);
	}
	return status;
}

int manage_multi_touch_data_received(struct itouch_device *idv)
{
	int status = 0;
	u32 touch_data_buf_index = 0;

	touch_data_buf_index = idv->touch_buffer_index;

	if (idv->gpuParams.gfxCore != IGFX_GEN7_5_CORE) {
		itouch_dbg(idv,
			   "We are in a Non HSW SKU.Submit should be thru GUC 0x%x \n",
			   idv->gpuParams.gfxCore);
		return -1;
	}

	if (touch_data_buf_index >= HID_PARALLEL_DATA_BUFFERS) {
		itouch_dbg(idv,
			   "We are sending with wrong touch_buffer_index 0x%x",
			   touch_data_buf_index);
	}

	if (idv->graphics_state.comp_state != GFX_TOUCH_READY) {
		itouch_dbg(idv, " Intel Graphics driver not ready 0x%x",
			   idv->graphics_state.comp_state);
	}
	//ret = itouch_gfx_submit_kernel(touch_data_buf_index);

	if ((status != 0)) {
		itouch_dbg(idv,
			   "The submission of raw data to the  Kernel failed with status 0x%x \n",
			   status);
		return status;
	}

	itouch_dbg(idv, "The data submitted to Kernel \n");

	return status;
}

int parse_touch_sensor_feedback_ready_resp(struct itouch_device *idv,
					   TOUCH_SENSOR_MSG_M2H in_msg,
					   u32 ilength)
{
	int status = 0;
	u32 Expectedilength;

	Expectedilength =
	    sizeof(in_msg.CommandCode) +
	    sizeof(TOUCH_SENSOR_FEEDBACK_READY_RSP_DATA);

	if (ilength != Expectedilength) {
		itouch_dbg(idv,
			   "Incorrect response size for TOUCH_SENSOR_FEEDBACK_READY_RSP_DATA: expected %d, got %d\n",
			   Expectedilength, ilength);
		status = -1;
	}

	if (in_msg.M2HData.FeedbackReadyRspData.Status != 0) {
		itouch_dbg(idv,
			   "Bad response status for TOUCH_SENSOR_FEEDBACK_READY_RSP_DATA: expected %d, got %d\n",
			   0, in_msg.M2HData.FeedbackReadyRspData.Status);
		status = -1;
	}

	if (status != 0) {
		idv->me_state.comp_state = ME_MEI_FEEDBACK_ERROR;
		idv->me_state.error_status = true;
	} else {
		idv->me_state.comp_state = ME_MEI_FEEDBACK_RESP_RECEIVED;
		status = 0;
	}
	// The HID2MEBuffer is completed.. Do the necessary action.
	if (in_msg.M2HData.FeedbackReadyRspData.FeedbackIndex ==
	    TOUCH_SENSOR_MAX_DATA_BUFFERS && idv->hid_me_buf_busy) {
		itouch_dbg(idv, "The HID2ME Buffer was completed \n");
		idv->hid_me_buf_busy = false;
		//int_touch_complete_hid_read_report_request(idv, status, false);
	}

	if (idv->vendor_entries.touch_time_log_mode > 0) {
		//itouch_dbg(idv, "The frame processing time is  %d \n",
		//	   in_msg.M2HData.FeedbackReadyRspData.PerfData);
		itouch_dbg(idv, "considtion (idv->vendor_entries.touch_time_log_mode > 0)\n");
	}

	return status;
}

int set_log_mode(struct itouch_device *idv)
{
	int status = 0;
	TOUCH_SENSOR_MSG_H2M out_msg;

	itouch_dbg(idv,
		   "Sending TOUCH_SENSOR_SET_POLICIES_CMD with mode %lu\n",
		   idv->vendor_entries.touch_time_log_mode);
	memset(&out_msg, 0, sizeof(out_msg));

	out_msg.CommandCode = TOUCH_SENSOR_SET_POLICIES_CMD;
//	out_msg.H2MData.SetPerfModeCmdData.PerfMode =
//	    idv->vendor_entries.touch_time_log_mode;

	// send the command to touch firmware
	status =
	    mei_itouch_write_message(idv,
				     sizeof(out_msg.CommandCode) +
				     sizeof
				     (TOUCH_SENSOR_SET_POLICIES_CMD_DATA),
				     (u8 *) & out_msg);

	if (status != 0) {
		itouch_dbg(idv, "mei write failed with status = %d\n", status);
		idv->me_state.comp_state = ME_MEI_SET_POLICIES_ERROR;
		idv->me_state.error_status = true;
		return status;
	}

	idv->me_state.comp_state = ME_MEI_SET_POLICIES_SEND;

	return status;
}

int parse_touch_sensor_set_perf_mode_resp(struct itouch_device *idv,
					  TOUCH_SENSOR_MSG_M2H in_msg,
					  u32 ilength)
{
	int status = 0;
	u32 Expectedilength;

	// process response message
	Expectedilength =
	    sizeof(in_msg.CommandCode) +
	    sizeof(TOUCH_SENSOR_SET_POLICIES_RSP_DATA);
	if (ilength != Expectedilength) {
		itouch_dbg(idv,
			   "%s - Incorrect response size: expected %d, got %d\n",
				rsp_msg[in_msg.CommandCode&0x0000000F],
			   Expectedilength, ilength);
		status = -1;
	}

	if (in_msg.M2HData.SetPoliciesRspData.Status != 0) {
		itouch_dbg(idv,
			    "%s - Bad response status: expected %d, got %d\n",
				rsp_msg[in_msg.CommandCode&0x0000000F],
			    0, in_msg.M2HData.SetPoliciesRspData.Status);
		status = -1;
	}

	if (status != 0) {
		idv->me_state.comp_state = ME_MEI_SET_POLICIES_ERROR;
		idv->me_state.error_status = true;
	} else {
		idv->me_state.comp_state = ME_MEI_SET_POLICIES_RECEIVED;
		status = 0;
	}

	return status;
}

int touch_device_mei_disconnect(struct itouch_device *idv)
{
       int status = 0;
       status =
           hid_touch_heci2_interface(idv,
                                     (MEITOUCH_FEATURE_COMMANDS)
                                     TOUCH_CLEAR_MEM);

       // TODO
       // windows code waits for the HECIInterrupt to indicate success or failure.

       return status;
}

void itouch_print_hex_dump(const void* hex_data, const char* prefix_msg,
		unsigned int hex_len)
{
#ifdef DEBUGITOUCH
	/*Dump a max of 100 lines of hex data, with 32 bytes in each line*/
	printk(KERN_ERR "ITouch Buffer: %s\n", prefix_msg);
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 32, 1,
			hex_data, (hex_len <=3200) ? hex_len : 3200 ,
			false);
#endif
}
