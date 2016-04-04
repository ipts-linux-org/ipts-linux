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

#include "mei-hid.h"
#include "itouch-state-codes.h"
#include "itouch-state-handler.h"
#include "itouch-gfx-hid-interface.h"

static int CompleteTouchProcessing(struct itouch_device *idv,
				   u32 BufferStartIndex, u32 BufferEndIndex);

int int_touch_graphics_procssing_complete_dpc(struct itouch_device *idv)
{
	int status = 0;
	int BufferStartIndex1 = 0;
	int BufferEndIndex1 = 0;
	int BufferStartIndex2 = 0;
	int BufferEndIndex2 = 0;

	if (idv->current_buffer_index >= 0
	    && idv->current_buffer_index < HID_PARALLEL_DATA_BUFFERS) {

		if (idv->vendor_entries.hid_debug_mode || idv->vendor_entries.graphics_debug_mode) {
			//TODO: How/where to log ?
		} else {
			if ((idv->last_buffer_completed != (HID_PARALLEL_DATA_BUFFERS - 1))) {
				if (idv->last_buffer_completed == -1 || idv->current_buffer_index > idv->last_buffer_completed) {
					BufferStartIndex1 = idv->last_buffer_completed + 1;
					BufferEndIndex1   = idv->current_buffer_index + 1;
				} else {
					BufferStartIndex1 = idv->last_buffer_completed + 1;
					BufferEndIndex1 = HID_PARALLEL_DATA_BUFFERS;

					BufferStartIndex2 = 0;
					BufferEndIndex2 = idv->current_buffer_index + 1;
				}

			} else {
				BufferStartIndex1 = 0;
				BufferEndIndex1 = idv->current_buffer_index + 1;
			}

			itouch_dbg(idv, "BufferStartIndex1  ---> [0x%x]\n", BufferStartIndex1);
			itouch_info(idv, "BufferEndIndex1  ---> [0x%x]\n", BufferEndIndex1);
			itouch_info(idv, "BufferStartIndex2  ---> [0x%x]\n", BufferStartIndex2);
			itouch_info(idv, "BufferEndIndex2  ---> [0x%x]\n", BufferEndIndex2);

			if (BufferEndIndex1 > 0)
				status = CompleteTouchProcessing(idv, BufferStartIndex1, BufferEndIndex1);

			if (BufferEndIndex2 > 0)
				status = CompleteTouchProcessing(idv, BufferStartIndex2, BufferEndIndex2);
		}
	} else {
		itouch_dbg(idv, "The Buffer Index is wrong. So we are not sending touches 0x%x", idv->current_buffer_index);
	}

	return status;
}

int CompleteTouchProcessing(struct itouch_device *idv, u32 BufferStartIndex,
			    u32 BufferEndIndex)
{
	int status = 0;
	u32 touch_buffer_index = 0;
	bool fatalerror = false;

	if(idv->me_state.error_status)
		return 0;

	for (touch_buffer_index = BufferStartIndex;
	     touch_buffer_index < BufferEndIndex; touch_buffer_index++) {

		status = send_touches_to_os(idv, touch_buffer_index);
		if (status != 0) {
			itouch_dbg(idv,
				   "send_touches_to_os returned error status = [0x%x]",
				   status);
			//Donot do further processing of touch_complete...move on.
			if(status == GFX_OCL_KERNEL_FATAL_ERROR) {
				fatalerror = true;
				break;
			}
		}
		//TODO: Handled GFX_OCL Error, switch to HID mode.
//	    status = touch_complete_frame_processing(idv, touch_buffer_index);
//		if (status != 0) {
//			itouch_dbg(idv, "ME is not ready to send data. HID Driver ready command returned error  \n");
//		}
	}

	if(fatalerror) {
		//TODO: Fall back to HID Mode
	}

	return status;
}

int send_touches_to_os(struct itouch_device *idv, u32 touch_data_buf_index)
{
	int status = 0;
	u32 OutputBufferIndex = 0;
	u16 BytesToCopy;
	KERNEL_OUTPUT_BUFFER_HEADER Header;
	KERNEL_ERROR_PAYLOAD	ErrorPayload;
//	TOUCH_RAW_DATA_HDR touch_data_hdr = { 0 };

//	memcpy(&touch_data_hdr,
//	       idv->touch_buffer[touch_data_buf_index].raw_touch_data_buffer,
//	       sizeof(TOUCH_RAW_DATA_HDR));

//	itouch_dbg(idv, "RAW_DATA_BFR.DataType=%d, RawDataSizeBytes=%d, BufferId=%d\n",
//		touch_data_hdr.DataType, touch_data_hdr.RawDataSizeBytes, touch_data_hdr.BufferId);

	for (OutputBufferIndex = 0; OutputBufferIndex < idv->num_output_buffers;
	     OutputBufferIndex++) {
		memcpy(&Header,
		       idv->touch_buffer[touch_data_buf_index].
		       processed_touch_buffer[OutputBufferIndex], HEADER_SIZE);

		BytesToCopy = Header.Length;

		if(Header.Length)
			itouch_info(idv,
			"touch: #%d, output_buff= #%d, Header.Len=%d, Header.PayloadType=%d, Hdr.Priv_data.TransId=%d\n",
			touch_data_buf_index, OutputBufferIndex,
			Header.Length, Header.PayloadType, Header.HidPrivateData.TransactionId);

		if (BytesToCopy < HEADER_SIZE) {
			//Incorrect size or invalid size
			continue;
		}

		BytesToCopy = BytesToCopy - HEADER_SIZE;

		itouch_print_hex_dump(idv->touch_buffer[touch_data_buf_index].
							processed_touch_buffer[OutputBufferIndex],
								 "Processed Touch HEADER:  ", HEADER_SIZE);
		if (Header.PayloadType == OUTPUT_BUFFER_PAYLOAD_HID_INPUT_REPORT) {

			itouch_print_hex_dump(idv->touch_buffer[touch_data_buf_index].
                            processed_touch_buffer[OutputBufferIndex]+HEADER_SIZE,
                                 "HID Input Report Buffer:  ", BytesToCopy);

			memcpy(idv->device_touch_data,
			       (idv->touch_buffer[touch_data_buf_index].
				processed_touch_buffer[OutputBufferIndex] +
				HEADER_SIZE), BytesToCopy);

			idv->device_touch_data->collection_size = BytesToCopy;
#ifdef SEND2OS
			int_touch_complete_read_report(idv, idv->device_touch_data);
#endif
			idv->hid_debug_log_data.GfxMode.hid_input_reports++;
			status = 0;
		} else if (Header.PayloadType == OUTPUT_BUFFER_PAYLOAD_FEEDBACK_BUFFER) {
			//This is feedback buffer, send the feedback
			touch_complete_send_feedback(idv, touch_data_buf_index, OutputBufferIndex,
						     Header.HidPrivateData.TransactionId, Header.Length);
		} else if (Header.PayloadType ==
			   OUTPUT_BUFFER_PAYLOAD_HID_FEATURE_REPORT) {

			itouch_print_hex_dump(idv->touch_buffer[touch_data_buf_index].
                            processed_touch_buffer[OutputBufferIndex]+HEADER_SIZE,
                                 "HID Feature Report Buffer:  ", BytesToCopy);

			memcpy(idv->me_hid_report_buff,
			       (idv->touch_buffer[touch_data_buf_index].
					processed_touch_buffer[OutputBufferIndex] + HEADER_SIZE),
					 BytesToCopy);
			idv->hid_report_size = BytesToCopy;

			int_touch_complete_hid_read_report_request(idv, status, true);
			status = 0;

		} else if (Header.PayloadType == OUTPUT_BUFFER_PAYLOAD_KERNEL_LOAD) {

			itouch_dbg(idv, "The OCL OuptutBuffer Header is requesting Kernel Switching 0x%x\n", Header.PayloadType);

		} else if (Header.PayloadType == OUTPUT_BUFFER_PAYLOAD_ERROR) {

			itouch_dbg(idv, "The OCL OuptutBuffer Header returned as OUTPUT_BUFFER_PAYLOAD_ERROR (Hdr.payloadtype=0x%x)\n",
				Header.PayloadType);

			memcpy(&ErrorPayload, idv->touch_buffer[touch_data_buf_index].processed_touch_buffer[OutputBufferIndex]+HEADER_SIZE,
						sizeof(KERNEL_ERROR_PAYLOAD));

			if(ErrorPayload.ErrorSeverity == 0){
				itouch_dbg(idv, "Invalid Error Data,Payload.ErrorSeverity = 0");
			}else {
				itouch_dbg(idv, "Error Severity: %d, Error Source: %d, Error String=%s\n",
					ErrorPayload.ErrorSeverity, ErrorPayload.ErrorSource, ErrorPayload.ErrorString);
				itouch_dbg(idv, "Error code Bytes are[0-3]: %d : %d %d : %d\n",
					ErrorPayload.ErrorCode[0], ErrorPayload.ErrorCode[1],
					ErrorPayload.ErrorCode[2], ErrorPayload.ErrorCode[3]);

				if (ErrorPayload.ErrorSeverity == ERROR_SEVERITY_FATAL){
					status = GFX_OCL_KERNEL_FATAL_ERROR;
					itouch_dbg(idv, "OCL Kernel Encountered ERROR_SEVERITY_FATAL\n");
					//no further processing
					break;
				}else if (ErrorPayload.ErrorSeverity == ERROR_SEVERITY_NONFATAL){
					itouch_dbg(idv, "OCL Kernel Encountered ERROR_SEVERITY_NONFATAL");
				}
			}
		}
		else {
			itouch_dbg(idv, "The OCL OuptutBuffer Header unknown 0x%x\n", Header.PayloadType);
		}
	}

	return status;
}

int send_single_touch_to_os(struct itouch_device *idv, u32 touch_data_buf_index)
{
	int status = 0;
	TOUCH_RAW_DATA_HDR touch_data_hdr = { 0 };

	memcpy(&touch_data_hdr,
	       idv->touch_buffer[touch_data_buf_index].hid_touch_buffer,
	       sizeof(TOUCH_RAW_DATA_HDR));

	itouch_print_hex_dump(idv->touch_buffer[touch_data_buf_index].hid_touch_buffer,
                                "Single Touch Input Report Header:  ",
								sizeof(TOUCH_RAW_DATA_HDR));

	if (touch_data_hdr.DataType == TOUCH_RAW_DATA_TYPE_HID_REPORT) {

		memcpy(idv->device_touch_data,
			idv->touch_buffer[touch_data_buf_index].
			hid_touch_buffer + sizeof(TOUCH_RAW_DATA_HDR),
			touch_data_hdr.RawDataSizeBytes);

		idv->device_touch_data->collection_size =
			(u32) (touch_data_hdr.RawDataSizeBytes);

		itouch_print_hex_dump(idv->device_touch_data,
			"Single Touch Data Buffer:  ", touch_data_hdr.RawDataSizeBytes);

		status = int_touch_complete_read_report(idv, idv->device_touch_data);
		if(status == 0)
			idv->hid_debug_log_data.NonGfxMode.hid_input_reports++;

	} else if (touch_data_hdr.DataType == TOUCH_RAW_DATA_TYPE_GET_FEATURES) {

		itouch_info(idv,
			   "Completing the Get Feature request in Single Touch Mode");
		memcpy(idv->me_hid_report_buff,
		       (idv->touch_buffer[touch_data_buf_index].
			hid_touch_buffer + sizeof(TOUCH_RAW_DATA_HDR)),
		       touch_data_hdr.RawDataSizeBytes);
		idv->hid_report_size = touch_data_hdr.RawDataSizeBytes;
		int_touch_complete_hid_read_report_request(idv, status, true);

	} else {

		itouch_dbg(idv,
			   "Single Touch headers dont match data type is 0x%x and size is 0x%x ",
			   touch_data_hdr.DataType,
			   touch_data_hdr.RawDataSizeBytes);
	}

	return status;
}

//TODO: Check the functionality of this function
int send_vendor_data_to_os(struct itouch_device *idv, u32 touch_data_buf_index)
{
	int status = 0;

	u32 PrintIndex = 0;
	u32 HIDPacketCount = 1;
	unsigned char Rawdata[VENDOR_DATA_SIZE];
	itouch_dbg(idv, "Send Sample data called with index 0x%x",
		   touch_data_buf_index);

	for (PrintIndex = 0; PrintIndex < idv->present_data_fsize;
	     PrintIndex = PrintIndex + (VENDOR_DATA_SIZE)) {
		itouch_dbg(idv,
			   "The number of HID packets %d Delay between packets is 0.1msec",
			   HIDPacketCount);

		memcpy(Rawdata,
		       (idv->touch_buffer[touch_data_buf_index].
			raw_touch_data_buffer + PrintIndex), (VENDOR_DATA_SIZE));
		memset(idv->device_touch_data, 0, sizeof(hid_loopback_report));
		memcpy(idv->device_touch_data->Data, Rawdata, VENDOR_DATA_SIZE);

		idv->device_touch_data->collection_size = (VENDOR_DATA_SIZE + 1);

		HIDPacketCount++;
	}

	itouch_dbg(idv,
		   "Vendor data received and the number of bytes sent is 0x%x",
		   idv->device_touch_data->collection_size);
	return status;
}
