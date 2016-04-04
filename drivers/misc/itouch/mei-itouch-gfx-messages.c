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
#include "mei-itouch-messages.h"
#include "itouch-state-codes.h"
#include "itouch-state-handler.h"
#include "itouch-gfx-hid-interface.h"

//int touch_complete_frame_processing(struct itouch_device *idv, int parallel_buf_index)
int touch_complete_send_feedback(struct itouch_device *idv,
			int parallel_buf_index, int OutputBufferIndex,
			u32 transactionid, int BytesToCopy)
{
	int status = 0;
//	u32 BytesToCopy = 0;
//	u8 OutputBufferIndex = 0;
//	KERNEL_OUTPUT_BUFFER_HEADER Header;
//	TOUCH_FEEDBACK_HDR feedbackheader;
	bool FeedbackBufferPresent = false;
	TOUCH_FEEDBACK_HDR HID2MEBuffer = { 0 };

//	if (idv->graphics_state.comp_state != GFX_TOUCH_READY) {
//		itouch_dbg(idv,
//			   "Gfx is not touch ready. The memory should be corrupted by now. The current Gfx state is 0x%x \n",
//			   idv->graphics_state.comp_state);
//		return -1;
//	}

//	if(parallel_buf_index  >= HID_PARALLEL_DATA_BUFFERS){
//		itouch_dbg(idv, "Invalid buffer index argument\n");
//		goto exit;
//	}
//	itouch_dbg(idv,
//		   "touch_complete_frame_processing called with bufferindex 0x%x \n",
//		   (unsigned char)parallel_buf_index);

//	for (OutputBufferIndex = 0; OutputBufferIndex < idv->num_output_buffers;
//	     OutputBufferIndex++) {

//		memcpy(&Header,
//		       idv->touch_buffer[parallel_buf_index].
//		       processed_touch_buffer[OutputBufferIndex], HEADER_SIZE);

//		BytesToCopy = Header.Length;

//		if (BytesToCopy > HEADER_SIZE) {

//			BytesToCopy = BytesToCopy - HEADER_SIZE;

//			if (Header.PayloadType == OUTPUT_BUFFER_PAYLOAD_FEEDBACK_BUFFER &&
//					BytesToCopy >= sizeof(TOUCH_FEEDBACK_HDR) ) {
				memcpy(idv->touch_buffer[parallel_buf_index].pfeedback_buffer,
				       (idv->touch_buffer[parallel_buf_index].
					processed_touch_buffer[OutputBufferIndex] + HEADER_SIZE),
					BytesToCopy);

//				memcpy(&feedbackheader,
//				       idv->touch_buffer[parallel_buf_index].
//				       pfeedback_buffer, sizeof(TOUCH_FEEDBACK_HDR));

//				itouch_dbg(idv,
//					   "Protocol Ver: 0x%x FeedBackCmdType: 0x%x PayloadBytes: 0x%x BufferId: 0x%x, Transaction ID: %d\n",
//					   (u32) feedbackheader.ProtocolVer,
//					   (unsigned char)feedbackheader.FeedbackCmdType,
//					   feedbackheader.PayloadSizeBytes,
//					   (unsigned char)feedbackheader.BufferId,
//						Header.HidPrivateData.TransactionId);
				itouch_info(idv, "<<<<FEEDBACK>>> for Multitouch present in Processed_buff[%d][%d], for Transaction ID: %d\n",
							parallel_buf_index, OutputBufferIndex, transactionid);

				FeedbackBufferPresent = true;
				status = touch_feedback_ready(idv, (u8) parallel_buf_index,
									transactionid);
				idv->last_buffer_completed = parallel_buf_index;
				idv->hid_debug_log_data.GfxMode.FeedbackData++;
//			}
//		}
//	}

	if (!FeedbackBufferPresent) {
		HID2MEBuffer.BufferId = parallel_buf_index;
		HID2MEBuffer.FeedbackCmdType = TOUCH_FEEDBACK_CMD_TYPE_NONE;
		HID2MEBuffer.PayloadSizeBytes = 0;
		HID2MEBuffer.ProtocolVer = 0;
		HID2MEBuffer.Reserved[0] = 0xab;
		memcpy(idv->touch_buffer[parallel_buf_index].pfeedback_buffer, &HID2MEBuffer, sizeof(HID2MEBuffer));
		status = touch_feedback_ready(idv, (u8) parallel_buf_index, 0);
	}

//exit:
	return status;
}

int single_touch_memory_init(struct itouch_device *idv)
{

#if IS_ENABLED(CONFIG_INTEL_MEI_ITOUCH)
        struct pci_dev *pdev = to_pci_dev(idv->mdv->cl->dev->dev);
#endif
	if (!pdev) {
	itouch_dbg(idv, "PCI device not found\n");
	return -ENODEV;
	}

	//Allocate one touch_data_buffer & feedback buffer
    idv->touch_buffer[0].hid_touch_buffer = dma_alloc_coherent(idv->dev,
                    idv->sensor_data.FrameSize,
                    &idv->touch_buffer[0].phy_hid_touch_buffer,
                    GFP_ATOMIC|GFP_DMA32);
    if (idv->touch_buffer[0].hid_touch_buffer == NULL) {
        itouch_dbg(idv, "Failed to allocated DMA memory for HID Touch buffer\n");
        goto mem_tb_err;
    }
	//Only one feedback buffer for HID touch mode
	idv->touch_buffer[0].hid_feedback_buffer = dma_alloc_coherent(
                idv->dev,
                idv->sensor_data.FeedbackSize,
                &idv->touch_buffer[0].phy_hid_feedback_buffer,
                GFP_ATOMIC|GFP_DMA32);
	if(idv->touch_buffer[0].hid_feedback_buffer == NULL){
		itouch_dbg(idv, "Failed to allocated HID Feedback buffer\n");
		/*No cleanup required for HID touch mode as its only one buffer*/
		goto mem_fb_err;
	}

	memset(idv->touch_buffer[0].hid_feedback_buffer, 0,
			idv->sensor_data.FeedbackSize);
	idv->single_touch.memory_allocation_status = true;
	return 0;

mem_fb_err:
	dma_free_coherent(idv->dev,
                    idv->sensor_data.FrameSize,
					idv->touch_buffer[0].hid_touch_buffer,
                    idv->touch_buffer[0].phy_hid_touch_buffer);
	idv->touch_buffer[0].hid_touch_buffer= NULL;
	idv->touch_buffer[0].phy_hid_touch_buffer = 0;
mem_tb_err:
	return -ENOMEM;
}

int multi_touch_feedback_buff_init(struct itouch_device *idv)
{
	int status = 0;
	int touch_data_buf_index;
	char *feedback_buff;

    for (touch_data_buf_index = 0;
         touch_data_buf_index < HID_PARALLEL_DATA_BUFFERS;
         touch_data_buf_index++) {

		feedback_buff = dma_alloc_coherent(idv->dev,
                idv->sensor_data.FeedbackSize,
                &idv->touch_buffer[touch_data_buf_index].phy_feedback_buffer,
                GFP_ATOMIC|GFP_DMA32);

       if (feedback_buff ==  NULL) {
            //TODO: Error handling and clean up on failure.
            touch_data_buf_index--;
            goto mem_err;
        }

        itouch_dbg(idv, "%u.Feedback buffer size is %d, Alloc buf is at %p\n",
                    (touch_data_buf_index+1),
                    idv->sensor_data.FeedbackSize,
                    (void*)feedback_buff);

		idv->touch_buffer[touch_data_buf_index].pfeedback_buffer = feedback_buff;
    }

	idv->multi_touch.memory_allocation_status = true;
	return status;

mem_err:
    for( ; touch_data_buf_index>=0; touch_data_buf_index--) {
        dma_free_coherent(idv->dev,
            idv->sensor_data.FeedbackSize,
			idv->touch_buffer[touch_data_buf_index].pfeedback_buffer,
            idv->touch_buffer[touch_data_buf_index].phy_feedback_buffer);
		idv->touch_buffer[touch_data_buf_index].pfeedback_buffer = NULL;
		idv->touch_buffer[touch_data_buf_index].phy_feedback_buffer = 0;
	}
	idv->multi_touch.memory_allocation_status = false;
	return -ENOMEM;
}

void free_memory_allocation(struct itouch_device *idv)
{
	int i=0;
	//TODO: Free GFX allocated memory buffers.

	//TODO: Free the DMA allocated Feedback buffer
	if(idv->touch_buffer[0].hid_touch_buffer)
		dma_free_coherent(idv->dev, idv->sensor_data.FrameSize,
			idv->touch_buffer[0].hid_touch_buffer,
			idv->touch_buffer[0].phy_hid_touch_buffer);

	if(idv->touch_buffer[0].hid_feedback_buffer)
		dma_free_coherent(idv->dev, idv->sensor_data.FeedbackSize,
			idv->touch_buffer[0].hid_feedback_buffer,
			idv->touch_buffer[0].phy_hid_feedback_buffer);

	if(idv->hid_me_buffer)
		dma_free_coherent(idv->dev, idv->sensor_data.FeedbackSize,
            idv->hid_me_buffer, idv->phy_hid_me_buff);

	idv->single_touch.memory_allocation_status = false;

	for (i = 0; i < HID_PARALLEL_DATA_BUFFERS; i++)
		if(idv->touch_buffer[i].pfeedback_buffer)
			dma_free_coherent(idv->dev, idv->sensor_data.FeedbackSize,
				idv->touch_buffer[i].pfeedback_buffer,
				idv->touch_buffer[i].phy_feedback_buffer);

	memset(&idv->sensor_data, 0, sizeof(idv->sensor_data));
	memset(&idv->touch_buffer, 0, sizeof(idv->touch_buffer));
}
