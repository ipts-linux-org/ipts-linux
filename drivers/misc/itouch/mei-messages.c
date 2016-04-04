/*
 * MEI-HID glue driver.
 *
 * Copyright (c) 2012-2014, Intel Corporation.
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

int MEI_MemoryInit(struct itouch_device *idv)
{
	int status = STATUS_SUCCESS;
	idv->ReadMsg =
	    (MEI_MESSAGE *) ExAllocatePoolWithTag(NonPagedPool, MAX_MSG_LEN,
						  IntTouch);

	if (idv->ReadMsg == NULL) {
		itouch_dbg(idv,
			   "Failed to allocate memory for the MEI message \n");
		return STATUS_UNSUCCESSFUL;
	}

	idv->WriteMsg =
	    (MEI_MESSAGE *) ExAllocatePoolWithTag(NonPagedPool,
						  (MAX_WRITE_MSG_PACKET_LEN +
						   sizeof(MEI_MESSAGE_HEADER)),
						  IntTouch);
	if (idv->WriteMsg == NULL) {
		itouch_dbg(idv,
			   "Failed to allocate memory for the wRITEMEI message \n");
		return STATUS_UNSUCCESSFUL;
	}

	return status;
}

int MEI_MemoryRelease(struct itouch_device *idv)
{
	int status = STATUS_SUCCESS;

	if (idv->ReadMsg != NULL) {
		ExFreePoolWithTag(idv->ReadMsg, IntTouch);
	}

	if (idv->WriteMsg != NULL) {
		ExFreePoolWithTag(idv->WriteMsg, IntTouch);
	}

	return status;
}

int MEI_ReadMessage(struct itouch_device *idv, u8 * data, u32 buf_len,
		    int *host_addr, int *me_addr, u32 * len)
{
	//MEI_MESSAGE * msg;
	int status = STATUS_SUCCESS;

	// Initialize host and me addresses to an unused value.
	*host_addr = -1;
	*me_addr = -1;

	// Allocated memory for transmit/recevive message.
	// msg = (MEI_MESSAGE *)ExAllocatePoolWithTag(NonPagedPool, MAX_MSG_LEN,IntTouch);
	if (idv->ReadMsg) {
		// Zero out the length of data that has been transferred.
		*len = 0;
		do {
			// Zero out the transaction layer message any message
			memset(idv->ReadMsg, 0, MAX_MSG_LEN);
			status =
			    MEI_HW_ReadMessage(idv, idv->ReadMsg,
					       READ_MSG_TIMEOUT);
			if (status != MEI_STATUS_SUCCESS) {
				itouch_dbg(idv,
					   "MEI_ReadMessage: Failed to get message\n");
				return STATUS_UNSUCCESSFUL;
			}
			// First time through get the message address from the packet header
			if ((*host_addr == -1) || (*me_addr == -1)) {
				*host_addr = idv->ReadMsg->Header.r.HostAddress;
				*me_addr = idv->ReadMsg->Header.r.MEAddress;
			} else
			    if ((*host_addr !=
				 (int)idv->ReadMsg->Header.r.HostAddress)
				|| (*me_addr |=
				    (int)idv->ReadMsg->Header.r.MEAddress)) {
				// Make sure we stay in the loop
				idv->ReadMsg->Header.r.MessageComplete = FALSE;
				continue;
			}
			// Increment the length of data in buffer
			*len += idv->ReadMsg->Header.r.Length;
			if (*len > buf_len) {
				itouch_dbg(idv,
					   "MEI_ReadMessage: Buffer too small to move message data\n");
				return STATUS_UNSUCCESSFUL;
			}
			// Move data from message buffer into the final buffer
			memcpy(data, idv->ReadMsg->Data,
			       idv->ReadMsg->Header.r.Length);
			// Increment pointer location further in buffer
			data += idv->ReadMsg->Header.r.Length;
		} while (!idv->ReadMsg->Header.r.MessageComplete);	// Keep looping until message complete
		return STATUS_SUCCESS;
	} else {
		itouch_dbg(idv, "Failed to allocate memory for message\n");
		return STATUS_UNSUCCESSFUL;
	}
}

#define MAX_WRITE_RETRIES 5

int MEI_WriteMessage(struct itouch_device *idv, u8 * data, u32 buf_len,
		     u32 host_addr, u32 me_addr)
{
	//MEI_MESSAGE * msg;
	int retry, remaining_data;
	int status;
	// Allocated memory for a single transaction packet.  It will be used to
	//  transmit all the data found in "data"
	//msg = (MEI_MESSAGE *)ExAllocatePoolWithTag(NonPagedPool, (MAX_WRITE_MSG_PACKET_LEN + sizeof(MEI_MESSAGE_HEADER)),IntTouch);

	if (idv->WriteMsg) {
		// Initialize a count used to determine how much data remains to be transmitted
		remaining_data = buf_len;
		do {
			// Initialize the Packet Header
			idv->WriteMsg->Header.r.HostAddress =
			    MEI_BUS_MSG_ADDRESS;
			idv->WriteMsg->Header.r.MEAddress = ME_BUS_MSG_ADDRESS;
			idv->WriteMsg->Header.r.Length =
			    (remaining_data <=
			     MAX_WRITE_MSG_PACKET_LEN) ? remaining_data :
			    MAX_WRITE_MSG_PACKET_LEN;

			// Copy data payload from input buffer into the packet.
			memcpy(idv->WriteMsg->Data, data,
			       idv->WriteMsg->Header.r.Length);
			// Reduce count of data remaing and increment pointers
			remaining_data -= idv->WriteMsg->Header.r.Length;
			data += idv->WriteMsg->Header.r.Length;

			// If more data remains, then the message is not complete
			idv->WriteMsg->Header.r.MessageComplete =
			    (remaining_data == 0) ? TRUE : FALSE;
			retry = 0;
			do {
				// Send message to client address
				status =
				    MEI_HW_WriteMessage(idv, idv->WriteMsg);
				if (status == MEI_STATUS_ME_NOT_READY
				    || status ==
				    MEI_STATUS_NOT_ENOUGH_BUFFER_AVAIL) {
					// ME is busy emptying the buffers, so wait a moment and retry
					if (retry > MAX_WRITE_RETRIES) {
						itouch_dbg(idv,
							   "MEI_HW_WriteMessage: Failed to write message after %d retries\n",
							   retry);
					} else {
						retry++;
						itouch_dbg(idv,
							   "MEI_HW_WriteMessage: Failed. The ME status received is 0x%x\n",
							   status);
						continue;
					}
				}

				if (status != MEI_STATUS_SUCCESS) {
					itouch_dbg(idv,
						   "WRITEMSG: Failed to write message (0x%X)\n",
						   status);
					//ExFreePoolWithTag(msg,IntTouch);
					return STATUS_UNSUCCESSFUL;
				}
			}
			while (status != MEI_STATUS_SUCCESS);
		}
		while (!idv->WriteMsg->Header.r.MessageComplete);	// Keep writing until message complete

		//ExFreePoolWithTag(msg,IntTouch);
		return STATUS_SUCCESS;
	} else {
		itouch_dbg(idv, "Failed to allocate memory for message\n");
		return STATUS_UNSUCCESSFUL;
	}
}

int MEI_ClientPropMsg(struct itouch_device *idv, u8 addr,
		      MEI_CLIENT_PROPERTIES * prop)
{
	MEI_MESSAGE *msg;
	int status;

	// Allocated memory for transmit/recevive message.
	msg =
	    (MEI_MESSAGE *) ExAllocatePoolWithTag(NonPagedPool, MAX_MSG_LEN,
						  IntTouch);
	if (msg) {
		HBM_HOST_CLIENT_PROP_REQUEST *prop_req;
		HBM_HOST_CLIENT_PROP_RESPONSE *prop_rsp;

		// Intialize the Transaction Layer's Message buffer for a BUS Message
		memset(msg, 0, MAX_MSG_LEN);
		msg->Header.r.MEAddress = MEI_BUS_MSG_ADDRESS;
		msg->Header.r.HostAddress = MEI_BUS_MSG_ADDRESS;
		msg->Header.r.Length = sizeof(HBM_HOST_CLIENT_PROP_REQUEST);
		msg->Header.r.MessageComplete = 1;

		// Initialize the Bus Layer Message for a Client Property Request
		prop_req = (HBM_HOST_CLIENT_PROP_REQUEST *) msg->Data;
		prop_req->Command.r.Command = HBM_HOST_CLIENT_PROP_COMMAND;
		prop_req->Command.r.IsResponse = FALSE;
		prop_req->Address = addr;

		// Send the Client Property Request to ME F/W
		status = MEI_HW_WriteMessage(idv, msg);
		if (status != MEI_STATUS_SUCCESS) {
			itouch_dbg(idv,
				   "GETPROP: Failed to send PROPERTY Request\n");
			ExFreePoolWithTag(msg, IntTouch);
			return STATUS_UNSUCCESSFUL;
		}

		return STATUS_SUCCESS;
	} else {
		itouch_dbg(idv, "Failed to allocate memory for message\n");
		return STATUS_UNSUCCESSFUL;
	}
}

#ifdef MEIDebug
int MEI_GetVersionMsg(struct itouch_device *idv, HBM_HOST_VERSION host_ver,
		      HBM_ME_VERSION * me_ver)
{
	MEI_MESSAGE *msg;
	int status;
	HBM_HOST_VERSION_REQUEST *ver_req;
	HBM_HOST_VERSION_RESPONSE *ver_rsp;

	// Allocated memory for transmit/recevive message.
	msg =
	    (MEI_MESSAGE *) ExAllocatePoolWithTag(NonPagedPool, MAX_MSG_LEN,
						  IntTouch);
	if (msg) {
		// Intialize the Transaction Layer's Message buffer for a BUS Message
		memset(msg, 0, MAX_MSG_LEN);
		msg->Header.r.MEAddress = MEI_BUS_MSG_ADDRESS;
		msg->Header.r.HostAddress = MEI_BUS_MSG_ADDRESS;
		msg->Header.r.Length = sizeof(HBM_HOST_VERSION_REQUEST);
		msg->Header.r.MessageComplete = TRUE;

		// Initialize the Bus Layer Message for a Get Version Request
		ver_req = (HBM_HOST_VERSION_REQUEST *) msg->Data;
		ver_req->Command.r.Command = HBM_VERSION_COMMAND;
		ver_req->Command.r.IsResponse = FALSE;
		ver_req->Version = host_ver;

		// Send the GetVersion Request to ME F/W
		status = MEI_HW_WriteMessage(idv, msg);
		if (status != MEI_STATUS_SUCCESS) {
			itouch_dbg(idv,
				   "GETVERSION: Failed to send VERSION Request\n");
			ExFreePoolWithTag(msg, IntTouch);
			return status;
		}
		// Re-Initialize the Message Buffer for the Version Response
		memset(msg, 0, MAX_MSG_LEN);
		status = MEI_HW_ReadMessage(idv, msg, READ_MSG_TIMEOUT);
		if (status != MEI_STATUS_SUCCESS) {
			itouch_dbg(idv,
				   "GETVERSION: Failed to get VERSION response\n");
			ExFreePoolWithTag(msg, IntTouch);
			return status;
		}
		// Validate the Response from the ME F/W
		if (msg->Header.r.HostAddress != MEI_BUS_MSG_ADDRESS) {
			itouch_dbg(idv,
				   "GETVERSION: Received response from wrong address 0x%02x\n",
				   msg->Header.r.HostAddress);
			ExFreePoolWithTag(msg, IntTouch);
			return STATUS_UNSUCCESSFUL;
		}
		ver_rsp = (HBM_HOST_VERSION_RESPONSE *) msg->Data;
		if (ver_rsp->Command.r.Command != HBM_VERSION_COMMAND ||
		    ver_rsp->Command.r.IsResponse == FALSE) {
			itouch_dbg(idv,
				   "GETVERSION: Received rincorrect command response 0x%x and response status is 0x%x\n",
				   ver_rsp->Command.r.Command,
				   ver_rsp->Command.r.IsResponse);
			itouch_dbg(idv,
				   "GETVERSION: HostVerSupported 0x%x MEMajorVer is 0x%x MEMinorVer is 0x%x \n",
				   ver_rsp->Version.HostVerSupported,
				   ver_rsp->Version.MEMajorVer,
				   ver_rsp->Version.MEMinorVer);
			ExFreePoolWithTag(msg, IntTouch);
			return STATUS_UNSUCCESSFUL;
		}
		// Return the ME version
		*me_ver = ver_rsp->Version;

		ExFreePoolWithTag(msg, IntTouch);
		return STATUS_SUCCESS;
	} else {
		itouch_dbg(idv, "Failed to allocate memory for message\n");
		return STATUS_UNSUCCESSFUL;
	}
}

int MEI_StopMsg(struct itouch_device *idv, u8 reason)
{
	MEI_MESSAGE *msg;
	int status;
	HBM_HOST_STOP_REQUEST *stop_req;
	HBM_HOST_STOP_RESPONSE *stop_rsp;

	// Allocated memory for transmit/recevive message.
	msg =
	    (MEI_MESSAGE *) ExAllocatePoolWithTag(NonPagedPool, MAX_MSG_LEN,
						  IntTouch);
	if (msg) {
		// Intialize the Transaction Layer's Message buffer for a BUS Message
		memset(msg, 0, MAX_MSG_LEN);
		msg->Header.r.MEAddress = MEI_BUS_MSG_ADDRESS;
		msg->Header.r.HostAddress = MEI_BUS_MSG_ADDRESS;
		msg->Header.r.Length = sizeof(HBM_HOST_STOP_REQUEST);
		msg->Header.r.MessageComplete = TRUE;

		// Initialize the Bus Layer Message for a STOP Request
		stop_req = (HBM_HOST_STOP_REQUEST *) msg->Data;
		stop_req->Command.r.Command = HBM_HOST_STOP_COMMAND;
		stop_req->Command.r.IsResponse = FALSE;
		stop_req->Reason = reason;

		// Send the STOP Request to ME F/W
		status = MEI_HW_WriteMessage(idv, msg);
		if (status != MEI_STATUS_SUCCESS) {
			itouch_dbg(idv,
				   "STOPMSG: Failed to send STOP Request\n");
			ExFreePoolWithTag(msg, IntTouch);
			return status;
		}
		// Re-Initialize the Message Buffer for the STOP Response
		memset(msg, 0, MAX_MSG_LEN);
		status = MEI_HW_ReadMessage(idv, msg, READ_MSG_TIMEOUT);
		if (status != MEI_STATUS_SUCCESS) {
			itouch_dbg(idv,
				   "STOPMSG: Failed to get STOP response\n");
			ExFreePoolWithTag(msg, IntTouch);
			return STATUS_UNSUCCESSFUL;
		}
		// Validate the Response from the ME F/W
		if (msg->Header.r.HostAddress != MEI_BUS_MSG_ADDRESS) {
			ExFreePoolWithTag(msg, IntTouch);
			return STATUS_UNSUCCESSFUL;
		}
		stop_rsp = (HBM_HOST_STOP_RESPONSE *) msg->Data;
		if (stop_rsp->Command.r.Command != HBM_HOST_STOP_COMMAND ||
		    stop_rsp->Command.r.IsResponse == FALSE) {
			itouch_dbg(idv,
				   "STOPMSG: Received incorrect command response 0x%02x\n",
				   stop_rsp->Command.uc);
			ExFreePoolWithTag(msg, IntTouch);
			return STATUS_UNSUCCESSFUL;
		}

		ExFreePoolWithTag(msg, IntTouch);
		return STATUS_SUCCESS;
	} else {
		itouch_dbg(idv, "Failed to allocate memory for message\n");
		return STATUS_UNSUCCESSFUL;
	}
}

int MEI_EnumerateMsg(struct itouch_device *idv, u8 * valid_addresses, u32 len)
{
	MEI_MESSAGE *msg;
	int status;

	// Make sure the array is large enough to hold a maximum number of clients.
	if (len < VALID_ADDRESS_ELEMENTS) {
		itouch_dbg(idv,
			   "ENUMERATE_BUS: Valid Address Array too small to hold elements\n");
		return STATUS_UNSUCCESSFUL;
	}
	// Allocated memory for transmit/recevive message.
	msg =
	    (MEI_MESSAGE *) ExAllocatePoolWithTag(NonPagedPool, MAX_MSG_LEN,
						  IntTouch);
	if (msg) {
		HBM_HOST_ENUMERATION_REQUEST *enum_req;
		HBM_HOST_ENUMERATION_RESPONSE *enum_rsp;

		// Intialize the Transaction Layer's Message buffer for a BUS Message
		memset(msg, 0, MAX_MSG_LEN);
		msg->Header.r.MEAddress = MEI_BUS_MSG_ADDRESS;
		msg->Header.r.HostAddress = MEI_BUS_MSG_ADDRESS;
		msg->Header.r.Length = sizeof(HBM_HOST_ENUMERATION_REQUEST);
		msg->Header.r.MessageComplete = TRUE;

		// Initialize the Bus Layer Message for a ENUMERATE Request
		enum_req = (HBM_HOST_ENUMERATION_REQUEST *) msg->Data;
		enum_req->Command.r.Command = HBM_HOST_ENUM_COMMAND;
		enum_req->Command.r.IsResponse = FALSE;

		// Send the ENUMERATE Request to ME F/W
		status = MEI_HW_WriteMessage(idv, msg);
		if (status != MEI_STATUS_SUCCESS) {
			itouch_dbg(idv,
				   "ENUMERATE_BUS: Failed to send ENUM Request\n");
			ExFreePoolWithTag(msg, IntTouch);
			return status;
		}
		// Re-Initialize the Message Buffer for the ENUMERATE Response
		memset(msg, 0, MAX_MSG_LEN);
		status = MEI_HW_ReadMessage(idv, msg, READ_MSG_TIMEOUT);
		if (status != MEI_STATUS_SUCCESS) {
			itouch_dbg(idv,
				   "ENUMERATE_BUS: Failed to get ENUM response\n");
			ExFreePoolWithTag(msg, IntTouch);
			return STATUS_UNSUCCESSFUL;
		}
		// Validate the Response from the ME F/W
		if (msg->Header.r.HostAddress != MEI_BUS_MSG_ADDRESS) {
			itouch_dbg(idv,
				   "ENUMERATE_BUS: Received response from wrong address 0x%02x\n",
				   msg->Header.r.HostAddress);
			ExFreePoolWithTag(msg, IntTouch);
			return STATUS_UNSUCCESSFUL;
		}
		enum_rsp = (HBM_HOST_ENUMERATION_RESPONSE *) msg->Data;
		if (enum_rsp->Command.r.Command != HBM_HOST_ENUM_COMMAND ||
		    enum_rsp->Command.r.IsResponse == FALSE) {
			itouch_dbg(idv,
				   "ENUMERATE_BUS: Received incorrect command response 0x%02x\n",
				   enum_rsp->Command.uc);
			ExFreePoolWithTag(msg, IntTouch);
			return STATUS_UNSUCCESSFUL;
		}
		// Return the valid address array from the message into the array buffer
		memcpy(valid_addresses, enum_rsp->ValidAddresses,
		       VALID_ADDRESS_ELEMENTS);
		ExFreePoolWithTag(msg, IntTouch);
		return STATUS_SUCCESS;
	} else {
		itouch_dbg(idv, "Failed to allocate memory for message\n");
		return STATUS_UNSUCCESSFUL;
	}
}

#endif
