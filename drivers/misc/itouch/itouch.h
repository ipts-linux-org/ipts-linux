/*
 *
 * Intel Management Engine Interface (Intel MEI) Linux driver
 * Copyright (c) 2003-2016, Intel Corporation.
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

#ifndef _ITOUCH_H_
#define _ITOUCH_H_

#include <linux/hid.h>
#include "../mei/client.h" /*TODO:check this*/
#include "itouch-gfx-interface.h"
#include "itouch-binary-spec.h"
#include "itouch-regs.h"
#include "mei-itouch-messages.h"

//#define DEBUGITOUCH /*Enable General iTouch debug*/
//#define DEBUG_PTOUCH /*Enable iTouch ME debug thred*/
//#define DEBUG_THREAD_SLEEP 100
//#define DUMP_BUF /* Dump buffers if debug thread enabled*/
//#define DUMP_BYTES_OF_BUF 128 /*Specifies how many bytes of buf to Dump*/
#define SEND2OS /*This enables/disables sending output buffers to hidcore driver*/
#define MULTI_TOUCH_DEFAULT /*this enables multitouch default after bootup*/
#define REACQUIRE_DB_WORK_DELAY HZ /*1 sec*/
#define TAIL_THRESHOLD 2 /*Max 240 */

#ifdef DEBUGITOUCH
#define itouch_info(idv, format, arg...) do {                    \
        printk(KERN_INFO "itouch: f(%s): " format, __func__ , ##arg);        \
} while (0)

#define itouch_dbg(idv, format, arg...) do {					\
		printk(KERN_ERR "itouch: f(%s): " format, __func__ , ##arg);		\
} while (0)
#else
#define itouch_info(idv, format, arg...) do {}while(0);
#define itouch_dbg(idv, format, arg...) do {}while(0);
#endif

#if IS_ENABLED(CONFIG_INTEL_MEI_ITOUCH_DIRECT)
static inline void *itouch_get_clientdata(const struct mei_device *dev)
{
	return dev_get_drvdata(&dev);
}

static inline void itouch_set_clientdata(struct mei_device *dev, void *data)
{
	dev_set_drvdata(&dev, data);
}
#endif

#if IS_ENABLED(CONFIG_INTEL_MEI_ITOUCH)
static inline void *itouch_get_clientdata(const struct mei_cl_device *dev)
{
	return dev_get_drvdata(&dev->dev);
}

static inline void itouch_set_clientdata(struct mei_cl_device *dev, void *data)
{
	dev_set_drvdata(&dev->dev, data);
}
#endif

#define SDK04
#define BDW_SURFACE_HACK

#define OLD_ME_COMPATIBILITY

#define DOORBELL_SIZE 4
#define TAIL_SIZE 4
#define HID_PARALLEL_DATA_BUFFERS TOUCH_SENSOR_MAX_DATA_BUFFERS
#define WORK_QUEUE_SIZE  TOUCH_SENSOR_MAX_DATA_BUFFERS
#define WORK_QUEUE_ITEM_SIZE 16
#define LASTSUBMITID_DEFAULT_VALUE -1
#define DEVICE_STRING_INDEX 1

// Max Transaction Layer Message (CB can only go to 128 DWORDS)
#define MAX_MSG_LEN 128 * sizeof(u32)

//This is including the report_id
#define SINGLE_TOUCH_PROCESSED_SIZE 6
//0x1FF8 is the max size of Input Report we got
#define VENDOR_DATA_SIZE 0x200	//Got from the simple HID Write . This size is without report id.IntelDescriptorV6
#define MAX_TOUCH_PROCESSED_SIZE 0x3000
#define INPUT_REPORT_BYTES     (MAX_TOUCH_PROCESSED_SIZE -2)
//#define HEADER_SIZE 4
#define MAX_NUM_OUTPUTBUFFER 20

#define MILLISECOND 10000	// 100 nanosecs * 10,000 = 1 ms
#define RELATIVE_MILLISECOND (-MILLISECOND)	// minus means relative time

#define WRITE_CHANNEL_REPORT_ID 0xa
#define READ_CHANNEL_REPORT_ID 0xb
#define CONFIG_CHANNEL_REPORT_ID 0xd
#define VENDOR_INFO_REPORT_ID 0xF
#define SINGLE_TOUCH_REPORT_ID 0x40

#define PARAMETER_STATUS_OFFSET 1
#define PARAMETER_STATUS_SIZE 1
#define PARAM1_SIZE 1
#define PARAM2_SIZE 3
//Will need to repurpose the code once the header definition for the Data Header from OCL is defined

#define HECI2_REG_SIZE 32

//This is used to check the compatibility of the driver with the the Touch ME Client
#define TOUCH_PROTOCOL_VERSION 4

#define HECI2_TOUCH_CLIENT 0xC
#define HECI_TOUCH_GET_PROTOCOL_RESPONSE 0xC85

#define ITOUCH_HID_VERSION 0x01
#define ITOUCH_HID_VENDOR USB_VENDOR_ID_NTRIG
#define ITOUCH_HID_PRODUCT USB_VENDOR_ID_NTRIG_TOUCH_SCREEN

//itouch files
#define ITOUCH_VENDOR_KERNEL_BLOB "/itouch/vendor_kernel_skl.bin"
#define ITOUCH_VENDOR_KERNEL_CFG "/itouch/integ_sft_cfg_skl.bin"
//#define ITOUCH_VENDOR_KERNEL_CALIB "/itouch/integ_sft_clb_skl.bin"
#define ITOUCH_VENDOR_HID_DESCRIPTOR "/itouch/vendor_descriptor.bin"
#define ITOUCH_INTEG_KERNEL_HID_DESCRIPTOR "/itouch/integ_descriptor.bin"


typedef uint64_t HANDLE;

#pragma pack(1)

typedef struct _KERNEL_OUTPUT_BUFFER_HEADER{
	u16 Length;
	u8  PayloadType;
	u8  Reserved1;
	TOUCH_HID_PRIVATE_DATA HidPrivateData;
	u8 Reserved2[28];
}KERNEL_OUTPUT_BUFFER_HEADER;
C_ASSERT(sizeof(KERNEL_OUTPUT_BUFFER_HEADER) == 64);

#define HEADER_SIZE sizeof(KERNEL_OUTPUT_BUFFER_HEADER)

#define ERROR_SEVERITY_FATAL                1
#define ERROR_SEVERITY_NONFATAL             2

#define ERROR_SOURCE_KERNEL                 1
#define ERROR_SOURCE_TOUCH_IC_FW            2

typedef struct _KERNEL_ERROR_PAYLOAD{
	u16 ErrorSeverity;     // 0 - INVALID. HID driver will
	                       // ignore the error data below.
			       // 1 - FATAL
				// 2 - NON-FATAL
        u16 ErrorSource;       // 1 - KERNEL. 2 - Touch IC FW
        u8  ErrorCode[4];      // Error Code from Error Source
	char ErrorString[128];  // Vendor Defined Error String
}KERNEL_ERROR_PAYLOAD;
C_ASSERT(sizeof(KERNEL_ERROR_PAYLOAD) == 136);

#pragma pack()

typedef struct _hid_loopback_report {
	u8  ReportId;                 //Report ID for the collection.
        u8  Data[INPUT_REPORT_BYTES];
	u16 collection_size;
} hid_loopback_report, *phid_loopback_report;

typedef char hid_report_descriptor, *phid_report_descriptor;

typedef struct _properties {
	char *itouch_dir;
	char *default_kernel;
	ulong manufacturer;
	ulong product;
	ulong version;
	char *vendor_descriptor_file_name;
	char *intel_descriptor_file_name;
	char *calibration_file_name;
	char *confi_file_name;
	char *touch_blob_file;
	ulong reg_sensor_mode;
	ulong max_touch_report_id;
	ulong graphics_debug_mode;
	ulong hid_debug_mode;
	ulong touch_time_log_mode;
	ulong touch_blob_report_id;
	ulong dozetimer;            // Sensor DozeTimer value in seconds that
                                    // HID driver needs to send to ME as part of Set Policies Command
        ulong HECI_power_gate_timer;   // HECI PowerGate Timer value
                                       // in seconds that HID driver needs to send
                                       // "OK to Power Gate HECI" command to ME via MMIO access.
        ulong spi_frequency_override; // SpiFrequencyOverride value that HID driver
                                      // sends to ME to set the SPI frequency.
        ulong spi_io_override;        // SpiIoOverride value that HID driver
                                      //sends to ME to set the SPI IO Mode.
        ulong debug_override;
	ulong touch_THQA_blob_reportId;
	ulong stylus_THQA_Blob_reportId;

} properties;

typedef struct _hid_get_feature_report {
	u8 report_id;
	u8 feature_data;
} hid_get_feature_report, *phid_get_feature_report;

typedef struct _component_state {
	uint comp_state;
	bool error_status;
} component_state;

typedef struct _hid_gfx_mode {
	//Number of Input Reports received
	u16 hid_input_reports;
	u16 hid_get_feature_reports;
	//Number of times Feedback data sent to ME
	u16 FeedbackData;
	//CReview::Counter to track NumEngineResets
	u16 GfxEngineResetCount;
	//CReview::Num Kernel FATAL Error and NON FATAL Error
	u16 kernel_fatal_error_count;
	u16 lernel_nonfatal_error_count;
	//Number of lost touch packets
	u32 lost_packet_count;
} hid_gfx_mode;

typedef struct _hid_non_gfx_mode {
	//Number of Input Reports received
	u16 hid_input_reports;
	u16 hid_get_feature_reports;
	u16 kernel_switching;
	//Number of times Feedback data sent to ME
	u16 FeedbackData;
        //CReview::Counter to track NumEngineResets
        u16 GfxEngineResetCount;
        //CReview::Num Kernel FATAL Error and NON FATAL Error
        u16 kernel_fatal_error_count;
        u16 lernel_nonfatal_error_count;
        //Number of lost touch packets
        u32 lost_packet_count;
} hid_non_gfx_mode;

typedef struct _hid_debug_info {
	hid_gfx_mode GfxMode;
	hid_non_gfx_mode NonGfxMode;
	// Number of times a Mode transitions happens between Single Touch to MT Mode and Vice Versa.
	u16 mode_transitions;
	u16 set_feature_fw_count;
	u16 get_feature_fw_count;
	u16 output_report_fw_count;
	component_state graphics_state;
	component_state me_state;
	component_state hid_state;
	component_state driver_state;
	u8 Reserved[22];
} hid_debug_info;

typedef struct _sensor_touchbuffer {
	u64 pbatch_buffer;
	u32 pbatch_bufferSize;
	char *raw_touch_data_buffer;
	dma_addr_t phy_raw_touch_data_buffer;
	char *pfeedback_buffer;
	//Physical address for the Feedback touch buffer
	dma_addr_t phy_feedback_buffer;
	//Proceesed Touch Buffer
	char *processed_touch_buffer[MAX_NUM_OUTPUT_BUFFERS];
	//HID(Single touch buffers)
	char *hid_touch_buffer;
	dma_addr_t phy_hid_touch_buffer;
	char *hid_feedback_buffer;
	dma_addr_t phy_hid_feedback_buffer;

} sensor_touchbuffer, *psensor_touchbuffer;

typedef struct _mode_buffer {
	bool memory_allocation_status;
	u8 work_queue_item_size;
	u16 work_queue_size;
	ulong *tail_offset;
	unsigned long long *door_bell;
	char *raw_touch_data_buffer[HID_PARALLEL_DATA_BUFFERS];
	char *processed_touch_buffer[HID_PARALLEL_DATA_BUFFERS]
	    [MAX_NUM_OUTPUT_BUFFERS];
} mode_buffer, *pmode_buffer;

struct pending {
        bool status;
        struct completion ready;
	u32 code; //0 - get feature 1-set feature,2 -get input
	spinlock_t lock;
};

struct itouch_device {
	struct hid_device *hid;
#if IS_ENABLED(CONFIG_INTEL_MEI_ITOUCH_DIRECT)
	struct mei_device *mdv;
#endif
#if IS_ENABLED(CONFIG_INTEL_MEI_ITOUCH)
	struct mei_cl_device *mdv;
#endif
#if IS_ENABLED(CONFIG_DEBUG_FS)
        struct dentry *dbgfs_dir;
#endif /* CONFIG_DEBUG_FS */
	struct mutex mutex;
	struct input_dev *input;
	struct device *dev;
	component_state graphics_state;
	component_state me_state;
	component_state hid_state;
	component_state driver_state;
	bool mode_transition;
	mode_buffer single_touch;
	mode_buffer multi_touch;
	hid_debug_info hid_debug_log_data;
	char *descriptor_name;
	//Data sent from the application which will be used for Loopback
	phid_loopback_report device_touch_data;
	bool hid_me_buf_busy;
	//GRAPHICS HID Interface Object
	PHidGfxCallbacks pHidGfxCallbacks;
	PHidGfxMapBuffer pHidGfxMapBuffer;
	//Graphics Link Status
	u64 TailPtrAddr;
	u64 DoorbellCookieAddr;
#if 0
	UK_SCHED_PROCESS_DESCRIPTOR *appPrcocessParams;
#else
	struct guc_process_desc *appPrcocessParams;
#endif
	//GPU Information
	QUERY_GPU_PARAMS gpuParams;
	u32 current_buffer_index;
	u32 last_buffer_completed;
	int *plast_submitted_id;
	u32 touch_buffer_index;
	u32 num_output_buffers;
	//Data Received from Sensor
	TOUCH_SENSOR_GET_DEVICE_INFO_RSP_DATA sensor_data;
	TOUCH_SENSOR_MODE sensor_mode;
	u32 present_data_fsize;
	//This is jsut for POC
	u32 read_num_bytes;
	u32 read_offset;
	u32 frame_index;
	u32 num_samples;
	u32 hid_report_size;
	u8 *rdesc;
	char report_descriptor[5000];
	char touch_blob[256];
	//char*  report_descriptor;
	ssize_t hid_descriptor_size;
	ssize_t touch_blob_size;
	sensor_touchbuffer touch_buffer[HID_PARALLEL_DATA_BUFFERS];
	//Buffer from HID to ME
	char *hid_me_buffer;
	u8 work_queue_item_size;
	u16 work_queue_size;
	//Buffer for storing the HID report completion data
	char *me_hid_report_buff;
	u64 phy_hid_me_buff;
	unsigned long long gpu_handle;
	properties vendor_entries;
	struct pending req_pending;
	struct work_struct multi_touch_work;
	//Delayed struct for prevent guc stall
	struct delayed_work reacquire_db_work;
	bool display_status;
};

typedef enum {
	TOUCH_HW_RESET,
	TOUCH_SET_SENSOR_MODE,
	TOUCH_GET_DEVICE_INFO,
	TOUCH_SET_MEM_ALLOCATION,
	TOUCH_SET_MEM_INITIALIZATION,
	TOUCH_SET_TOUCH_READY,
	TOUCH_CLEAR_MEM,
	TOUCH_SINGLE_TOUCH_READY,
	TOUCH_MLT_TOUCH_READY,
	TOUCH_FLASH_MLT_TOUCH_READY,
	TOUCH_FLASH_SINGLE_TOUCH_READY,
	TOUCH_GET_DOORBELL,
	TOUCH_SEND_FEEDBACK_DATA,
	TOUCH_READ_SENSOR,
	TOUCH_WRITE_SENSOR,
	TOUCH_GRAPHICS_STATUS_NOT_READY,
	TOUCH_TEST_HW_POC,
	TOUCH_IS_SENSOR_READY,
	TOUCH_SET_POLICIES_MODE,
	TOUCH_GET_PROTOCOL_VERSION
} MEITOUCH_FEATURE_COMMANDS;
/*
 * This data struct is now availiable in include/linux/hid.h and not required
typedef struct heci_hid_desc {
	u8 length;
	u8 desc_type;
	u16 bcd;
	u8 country;
	u8 num_descriptors;
	struct _hid_descriptor_DESC_LIST {
		u8 report_type;
		u16 report_length;
	} desc_list[1];
} hid_descriptor, *phid_descriptor;;
*/
int mei_hid_itouch_get_hid_descriptor(struct itouch_device *idv);

int int_touch_complete_hid_descriptor(struct itouch_device *idv);
int int_touch_get_hid_descriptor(struct itouch_device *idv);
int int_touch_get_report_descriptor(struct itouch_device *idv,
				    struct hid_report *rep);
int int_touch_get_device_attributes(struct itouch_device *idv,
				    struct hid_report *rep);

int int_touch_complete_read_report(struct itouch_device *idv,
				   phid_loopback_report rep);
int int_touch_complete_hid_read_report_request(struct itouch_device *idv,
					       int action_status,
					       bool get_feature_comp);
int int_touch_post_hid2me_complete(struct itouch_device *idv);
int int_touch_write_report(struct itouch_device *idv, struct hid_report *rep);
int int_touch_set_feature(struct itouch_device *idv, u8 report_id,
			  unsigned char *report_buff, int report_buff_len);
int int_touch_get_feature(struct itouch_device *idv, u8 report_id,
			  unsigned char *report_buff, int report_buff_len);
int int_touch_send_to_sensor(struct itouch_device *idv);
int send_single_touch_to_os(struct itouch_device *idv,
			    u32 touch_data_buf_index);
int send_vendor_data_to_os(struct itouch_device *idv, u32 touch_data_buf_index);
int send_touches_to_os(struct itouch_device *idv, u32 touch_data_buf_index);

int touch_device_intel_graphics_unload(struct itouch_device *idv);
int touch_device_mei_disconnect(struct itouch_device *idv);

int hid_touch_heci2_interface(struct itouch_device *idv,
			      MEITOUCH_FEATURE_COMMANDS type);

int complete_touch_operations(struct itouch_device *idv);
int perform_graphics_operations(struct itouch_device *idv);
int complete_hid_completion(struct itouch_device *idv);
int int_touch_graphics_procssing_complete_dpc(struct itouch_device *idv);

int touch_is_sensor_ready(struct itouch_device *idv);
int check_touch_client_version(struct itouch_device *idv);

int touch_get_device_info(struct itouch_device *idv);
int touch_set_mem_window(struct itouch_device *idv);

int touch_sensor_quiesce_io(struct itouch_device *idv);

int touch_set_sensor_mode(struct itouch_device *idv);
int touch_hid_driver_ready(struct itouch_device *idv);
int touch_feedback_ready(struct itouch_device *idv, u8 feedback_buf_index,
				u32 transactionId);

int single_touch_memory_allocation(struct itouch_device *idv);
int single_touch_memory_init(struct itouch_device *idv);
int multi_touch_feedback_buff_init(struct itouch_device *idv);
void free_memory_allocation(struct itouch_device *idv);

int read_mei_messages(struct itouch_device *idv, TOUCH_SENSOR_MSG_M2H in_msg,
		      u32 ilength);

int parse_touch_sensor_quiesce_io_rsp(struct itouch_device *idv,
                                                TOUCH_SENSOR_MSG_M2H in_msg,
                                                u32 ilength);

int parse_touch_sensor_get_device_info_rsp_data(struct itouch_device *idv,
						TOUCH_SENSOR_MSG_M2H in_msg,
						u32 ilength);
int parse_touch_sensor_set_mem_window_rsp(struct itouch_device *idv,
					  TOUCH_SENSOR_MSG_M2H in_msg,
					  u32 ilength);
int parse_touch_sensor_init_complete_rsp(struct itouch_device *idv,
					 TOUCH_SENSOR_MSG_M2H in_msg,
					 u32 ilength);

int parse_touch_sensor_notify_device_dev_ready_rsp_data(struct itouch_device
							*idv,
							TOUCH_SENSOR_MSG_M2H
							in_msg,
							u32 ilength);
int parse_touch_sensor_HID_ready_for_data_resp(struct itouch_device *idv,
					       TOUCH_SENSOR_MSG_M2H in_msg,
					       u32 ilength);

int parse_touch_sensor_feedback_ready_resp(struct itouch_device *idv,
					   TOUCH_SENSOR_MSG_M2H in_msg,
					   u32 ilength);
int parse_touch_sensor_get_num_samples_POC_resp(struct itouch_device *idv,
						TOUCH_SENSOR_MSG_M2H in_msg,
						u32 ilength);
int parse_touch_sensor_set_perf_mode_resp(struct itouch_device *idv,
					  TOUCH_SENSOR_MSG_M2H in_msg,
					  u32 ilength);

int parse_touch_sensor_set_mode_resp(struct itouch_device *idv,
				     TOUCH_SENSOR_MSG_M2H in_msg,
				     u32 ilength);
int parse_touch_sensor_clear_mem_window_rsp_data(struct itouch_device *idv,
						 TOUCH_SENSOR_MSG_M2H in_msg,
						 u32 ilength);
int parse_touch_client_details(struct itouch_device *idv,
			       TOUCH_SENSOR_MSG_M2H out_msg, u32 ilength);

int finish_touch_operations(struct itouch_device *idv);

int write_raw_data_to_file(struct itouch_device *idv);
int manage_multi_touch_data_received(struct itouch_device *idv);
int manage_single_touch_data_received_from_sensor(struct itouch_device *idv);
int post_data_received_from_sensor(struct itouch_device *idv);

//int touch_complete_frame_processing(struct itouch_device *idv,
//				    int parallel_buf_index);
int touch_complete_send_feedback(struct itouch_device *idv,
		int parallel_buf_index, int OutputBufferIndex, u32 transactionid,
		int BytesToCopy);
int touch_clear_mem_window(struct itouch_device *idv);
int set_log_mode(struct itouch_device *idv);

int itouch_init(void);
void itouch_exit(void);
int itouch_probe(struct itouch_device *idv);
int itouch_open(struct itouch_device *idv);
void itouch_close(struct itouch_device *idv);
int itouch_start(struct itouch_device *idv);
void itouch_stop(struct itouch_device *idv);
void itouch_print_hex_dump(const void* hex_data,
			const char* prefix_msg, unsigned int hex_len);

#if IS_ENABLED(CONFIG_INTEL_MEI_ITOUCH_DIRECT)
struct TOUCH_SENSOR_MSG_M2H *m2h_read_msg(struct mei_device *dev,
					  struct mei_msg_hdr *opt_hdr,
					  int *len);
int h2m_write_msg(struct mei_device *dev, size_t len, u8 * buf);
#endif
#if IS_ENABLED(CONFIG_DEBUG_FS)
int itouch_dbgfs_register(struct itouch_device *dev, const char *name);
void itouch_dbgfs_deregister(struct itouch_device *dev);
#else
static inline int itouch_dbgfs_register(struct itouch_device *dev, const char *name)
{
        return 0;
}
static inline void itouch_dbgfs_deregister(struct itouch_device *dev) {}
#endif /* CONFIG_DEBUG_FS */

#endif //_ITOUCH_H_
