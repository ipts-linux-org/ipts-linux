/*
 * Precise Touch HECI Message
 *
 * Copyright (c) 2013-2016, Intel Corporation.
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

#ifndef _TOUCH_HECI_MSGS_H
#define _TOUCH_HECI_MSGS_H


#pragma pack(1)


// Initial protocol version
#define TOUCH_HECI_CLIENT_PROTOCOL_VERSION      10

// GUID that identifies the Touch HECI client.
#define TOUCH_HECI_CLIENT_GUID  \
            {0x3e8d0870, 0x271a, 0x4208, {0x8e, 0xb5, 0x9a, 0xcb, 0x94, 0x02, 0xae, 0x04}}


// define C_ASSERT macro to check structure size and fail compile for unexpected mismatch
#ifndef C_ASSERT
#define C_ASSERT(e) typedef char __C_ASSERT__[(e)?1:-1]
#endif


// General Type Defines for compatibility with HID driver and BIOS
#ifndef BIT0
#define BIT0 1
#endif
#ifndef BIT1
#define BIT1 2
#endif
#ifndef BIT2
#define BIT2 4
#endif


#define TOUCH_SENSOR_GET_DEVICE_INFO_CMD        0x00000001
#define TOUCH_SENSOR_GET_DEVICE_INFO_RSP        0x80000001


#define TOUCH_SENSOR_SET_MODE_CMD               0x00000002
#define TOUCH_SENSOR_SET_MODE_RSP               0x80000002


#define TOUCH_SENSOR_SET_MEM_WINDOW_CMD         0x00000003
#define TOUCH_SENSOR_SET_MEM_WINDOW_RSP         0x80000003


#define TOUCH_SENSOR_QUIESCE_IO_CMD             0x00000004
#define TOUCH_SENSOR_QUIESCE_IO_RSP             0x80000004


#define TOUCH_SENSOR_HID_READY_FOR_DATA_CMD     0x00000005
#define TOUCH_SENSOR_HID_READY_FOR_DATA_RSP     0x80000005


#define TOUCH_SENSOR_FEEDBACK_READY_CMD         0x00000006
#define TOUCH_SENSOR_FEEDBACK_READY_RSP         0x80000006


#define TOUCH_SENSOR_CLEAR_MEM_WINDOW_CMD       0x00000007
#define TOUCH_SENSOR_CLEAR_MEM_WINDOW_RSP       0x80000007


#define TOUCH_SENSOR_NOTIFY_DEV_READY_CMD       0x00000008
#define TOUCH_SENSOR_NOTIFY_DEV_READY_RSP       0x80000008


#define TOUCH_SENSOR_SET_POLICIES_CMD           0x00000009
#define TOUCH_SENSOR_SET_POLICIES_RSP           0x80000009


#define TOUCH_SENSOR_GET_POLICIES_CMD           0x0000000A
#define TOUCH_SENSOR_GET_POLICIES_RSP           0x8000000A


#define TOUCH_SENSOR_RESET_CMD                  0x0000000B
#define TOUCH_SENSOR_RESET_RSP                  0x8000000B


#define TOUCH_SENSOR_READ_ALL_REGS_CMD          0x0000000C
#define TOUCH_SENSOR_READ_ALL_REGS_RSP          0x8000000C




#define TOUCH_SENSOR_CMD_ERROR_RSP              0x8FFFFFFF  // M2H: ME sends this message to indicate previous command was unrecognized/unsupported



//*******************************************************************
//
// Touch Sensor Status Codes
//
//*******************************************************************
typedef enum _TOUCH_STATUS
{
    TOUCH_STATUS_SUCCESS = 0,               //  0 Requested operation was successful
    TOUCH_STATUS_INVALID_PARAMS,            //  1 Invalid parameter(s) sent
    TOUCH_STATUS_ACCESS_DENIED,             //  2 Unable to validate address range
    TOUCH_STATUS_CMD_SIZE_ERROR,            //  3 HECI message incorrect size for specified command
    TOUCH_STATUS_NOT_READY,                 //  4 Memory window not set or device is not armed for operation
    TOUCH_STATUS_REQUEST_OUTSTANDING,       //  5 There is already an outstanding message of the same type, must wait for response before sending another request of that type
    TOUCH_STATUS_NO_SENSOR_FOUND,           //  6 Sensor could not be found. Either no sensor is connected, the sensor has not yet initialized, or the system is improperly configured.
    TOUCH_STATUS_OUT_OF_MEMORY,             //  7 Not enough memory/storage for requested operation
    TOUCH_STATUS_INTERNAL_ERROR,            //  8 Unexpected error occurred
    TOUCH_STATUS_SENSOR_DISABLED,           //  9 Used in TOUCH_SENSOR_HID_READY_FOR_DATA_RSP to indicate sensor has been disabled or reset and must be reinitialized.
    TOUCH_STATUS_COMPAT_CHECK_FAIL,         // 10 Used to indicate compatibility revision check between sensor and ME failed, or protocol ver between ME/HID/Kernels failed.
    TOUCH_STATUS_SENSOR_EXPECTED_RESET,     // 11 Indicates sensor went through a reset initiated by ME
    TOUCH_STATUS_SENSOR_UNEXPECTED_RESET,   // 12 Indicates sensor went through an unexpected reset
    TOUCH_STATUS_RESET_FAILED,              // 13 Requested sensor reset failed to complete
    TOUCH_STATUS_TIMEOUT,                   // 14 Operation timed out
    TOUCH_STATUS_TEST_MODE_FAIL,            // 15 Test mode pattern did not match expected values
    TOUCH_STATUS_SENSOR_FAIL_FATAL,         // 16 Indicates sensor reported fatal error during reset sequence. Further progress is not possible.
    TOUCH_STATUS_SENSOR_FAIL_NONFATAL,      // 17 Indicates sensor reported non-fatal error during reset sequence. HID/BIOS logs error and attempts to continue.
    TOUCH_STATUS_INVALID_DEVICE_CAPS,       // 18 Indicates sensor reported invalid capabilities, such as not supporting required minimum frequency or I/O mode.
    TOUCH_STATUS_QUIESCE_IO_IN_PROGRESS,    // 19 Indicates that command cannot be complete until ongoing Quiesce I/O flow has completed.
    TOUCH_STATUS_MAX                        // 20 Invalid value, never returned
} TOUCH_STATUS;
C_ASSERT(sizeof(TOUCH_STATUS) == 4);



//*******************************************************************
//
// Defines for message structures used for Host to ME communication
//
//*******************************************************************


typedef enum _TOUCH_SENSOR_MODE
{
    TOUCH_SENSOR_MODE_HID = 0,          // Set mode to HID mode
    TOUCH_SENSOR_MODE_RAW_DATA,         // Set mode to Raw Data mode
    TOUCH_SENSOR_MODE_MAX               // Invalid value
} TOUCH_SENSOR_MODE;
C_ASSERT(sizeof(TOUCH_SENSOR_MODE) == 4);

typedef struct _TOUCH_SENSOR_SET_MODE_CMD_DATA
{
    TOUCH_SENSOR_MODE   SensorMode;     // Indicate desired sensor mode
    u32              Reserved[3];    // For future expansion
} TOUCH_SENSOR_SET_MODE_CMD_DATA;
C_ASSERT(sizeof(TOUCH_SENSOR_SET_MODE_CMD_DATA) == 16);


#define TOUCH_SENSOR_MAX_DATA_BUFFERS   16
#define TOUCH_HID_2_ME_BUFFER_ID        TOUCH_SENSOR_MAX_DATA_BUFFERS
#define TOUCH_HID_2_ME_BUFFER_SIZE_MAX  1024
#define TOUCH_INVALID_BUFFER_ID         0xFF

typedef struct _TOUCH_SENSOR_SET_MEM_WINDOW_CMD_DATA
{
    u32  TouchDataBufferAddrLower[TOUCH_SENSOR_MAX_DATA_BUFFERS];    // Lower 32 bits of Touch Data Buffer physical address. Size of each buffer should be TOUCH_SENSOR_GET_DEVICE_INFO_RSP_DATA.FrameSize
    u32  TouchDataBufferAddrUpper[TOUCH_SENSOR_MAX_DATA_BUFFERS];    // Upper 32 bits of Touch Data Buffer physical address. Size of each buffer should be TOUCH_SENSOR_GET_DEVICE_INFO_RSP_DATA.FrameSize
    u32  TailOffsetAddrLower;                                        // Lower 32 bits of Tail Offset physical address
    u32  TailOffsetAddrUpper;                                        // Upper 32 bits of Tail Offset physical address, always 32 bit, increment by WorkQueueItemSize
    u32  DoorbellCookieAddrLower;                                    // Lower 32 bits of Doorbell register physical address
    u32  DoorbellCookieAddrUpper;                                    // Upper 32 bits of Doorbell register physical address, always 32 bit, increment as integer, rollover to 1
    u32  FeedbackBufferAddrLower[TOUCH_SENSOR_MAX_DATA_BUFFERS];     // Lower 32 bits of Feedback Buffer physical address. Size of each buffer should be TOUCH_SENSOR_GET_DEVICE_INFO_RSP_DATA.FeedbackSize
    u32  FeedbackBufferAddrUpper[TOUCH_SENSOR_MAX_DATA_BUFFERS];     // Upper 32 bits of Feedback Buffer physical address. Size of each buffer should be TOUCH_SENSOR_GET_DEVICE_INFO_RSP_DATA.FeedbackSize
    u32  Hid2MeBufferAddrLower;                                      // Lower 32 bits of dedicated HID to ME communication buffer. Size is Hid2MeBufferSize.
    u32  Hid2MeBufferAddrUpper;                                      // Upper 32 bits of dedicated HID to ME communication buffer. Size is Hid2MeBufferSize.
    u32  Hid2MeBufferSize;                                           // Size in bytes of Hid2MeBuffer, can be no bigger than TOUCH_HID_2_ME_BUFFER_SIZE_MAX
    u8   Reserved1;                                                  // For future expansion
    u8   WorkQueueItemSize;                                          // Size in bytes of the GuC Work Queue Item pointed to by TailOffset
    u16  WorkQueueSize;                                              // Size in bytes of the entire GuC Work Queue
    u32  Reserved[8];                                                // For future expansion
} TOUCH_SENSOR_SET_MEM_WINDOW_CMD_DATA;
C_ASSERT(sizeof(TOUCH_SENSOR_SET_MEM_WINDOW_CMD_DATA) == 320);


#define TOUCH_SENSOR_QUIESCE_FLAG_GUC_RESET BIT0   // indicates GuC got reset and ME must re-read GuC data such as TailOffset and Doorbell Cookie values

typedef struct _TOUCH_SENSOR_QUIESCE_IO_CMD_DATA
{
    u32  QuiesceFlags;   // Optionally set TOUCH_SENSOR_QUIESCE_FLAG_GUC_RESET
    u32  Reserved[2];
} TOUCH_SENSOR_QUIESCE_IO_CMD_DATA;
C_ASSERT(sizeof(TOUCH_SENSOR_QUIESCE_IO_CMD_DATA) == 12);


typedef struct _TOUCH_SENSOR_FEEDBACK_READY_CMD_DATA
{
    u8   FeedbackIndex;  // Index value from 0 to TOUCH_HID_2_ME_BUFFER_ID used to indicate which Feedback Buffer to use. Using special value TOUCH_HID_2_ME_BUFFER_ID
                            // is an indication to ME to get feedback data from the Hid2Me buffer instead of one of the standard Feedback buffers.
    u8   Reserved1[3];   // For future expansion
    u32  TransactionId;  // Transaction ID that was originally passed to host in TOUCH_HID_PRIVATE_DATA. Used to track round trip of a given transaction for performance measurements.
    u32  Reserved2[2];   // For future expansion
} TOUCH_SENSOR_FEEDBACK_READY_CMD_DATA;
C_ASSERT(sizeof(TOUCH_SENSOR_FEEDBACK_READY_CMD_DATA) == 16);


#define TOUCH_DEFAULT_DOZE_TIMER_SECONDS    30

typedef enum _TOUCH_FREQ_OVERRIDE
{
    TOUCH_FREQ_OVERRIDE_NONE,   // Do not apply any override
    TOUCH_FREQ_OVERRIDE_10MHZ,  // Force frequency to 10MHz (not currently supported)
    TOUCH_FREQ_OVERRIDE_17MHZ,  // Force frequency to 17MHz
    TOUCH_FREQ_OVERRIDE_30MHZ,  // Force frequency to 30MHz
    TOUCH_FREQ_OVERRIDE_50MHZ,  // Force frequency to 50MHz (not currently supported)
    TOUCH_FREQ_OVERRIDE_MAX     // Invalid value
} TOUCH_FREQ_OVERRIDE;
C_ASSERT(sizeof(TOUCH_FREQ_OVERRIDE) == 4);

typedef enum _TOUCH_SPI_IO_MODE_OVERRIDE
{
    TOUCH_SPI_IO_MODE_OVERRIDE_NONE,    // Do not apply any override
    TOUCH_SPI_IO_MODE_OVERRIDE_SINGLE,  // Force Single I/O
    TOUCH_SPI_IO_MODE_OVERRIDE_DUAL,    // Force Dual I/O
    TOUCH_SPI_IO_MODE_OVERRIDE_QUAD,    // Force Quad I/O
    TOUCH_SPI_IO_MODE_OVERRIDE_MAX      // Invalid value
} TOUCH_SPI_IO_MODE_OVERRIDE;
C_ASSERT(sizeof(TOUCH_SPI_IO_MODE_OVERRIDE) == 4);

// Debug Policy bits used by TOUCH_POLICY_DATA.DebugOverride
#define TOUCH_DBG_POLICY_OVERRIDE_STARTUP_TIMER_DIS BIT0    // Disable sensor startup timer
#define TOUCH_DBG_POLICY_OVERRIDE_SYNC_BYTE_DIS     BIT1    // Disable Sync Byte check
#define TOUCH_DBG_POLICY_OVERRIDE_ERR_RESET_DIS     BIT2    // Disable error resets

typedef struct _TOUCH_POLICY_DATA
{
    u32                      Reserved0;          // For future expansion.
    u32                      DozeTimer     :16;  // Value in seconds, after which ME will put the sensor into Doze power state if no activity occurs. Set
                                                    // to 0 to disable Doze mode (not recommended). Value will be set to TOUCH_DEFAULT_DOZE_TIMER_SECONDS by
                                                    // default.
    TOUCH_FREQ_OVERRIDE         FreqOverride  :3;   // Override frequency requested by sensor
    TOUCH_SPI_IO_MODE_OVERRIDE  SpiIoOverride :3;   // Override IO mode requested by sensor
    u32                      Reserved1     :10;  // For future expansion
    u32                      Reserved2;          // For future expansion
    u32                      DebugOverride;      // Normally all bits will be zero. Bits will be defined as needed for enabling special debug features
} TOUCH_POLICY_DATA;
C_ASSERT(sizeof(TOUCH_POLICY_DATA) == 16);

typedef struct _TOUCH_SENSOR_SET_POLICIES_CMD_DATA
{
    TOUCH_POLICY_DATA           PolicyData;         // Contains the desired policy to be set
} TOUCH_SENSOR_SET_POLICIES_CMD_DATA;
C_ASSERT(sizeof(TOUCH_SENSOR_SET_POLICIES_CMD_DATA) == 16);


typedef enum _TOUCH_SENSOR_RESET_TYPE
{
    TOUCH_SENSOR_RESET_TYPE_HARD,   // Hardware Reset using dedicated GPIO pin
    TOUCH_SENSOR_RESET_TYPE_SOFT,   // Software Reset using command written over SPI interface
    TOUCH_SENSOR_RESET_TYPE_MAX     // Invalid value
} TOUCH_SENSOR_RESET_TYPE;
C_ASSERT(sizeof(TOUCH_SENSOR_RESET_TYPE) == 4);

typedef struct _TOUCH_SENSOR_RESET_CMD_DATA
{
    TOUCH_SENSOR_RESET_TYPE ResetType;  // Indicate desired reset type
    u32                  Reserved;   // For future expansion
} TOUCH_SENSOR_RESET_CMD_DATA;
C_ASSERT(sizeof(TOUCH_SENSOR_RESET_CMD_DATA) == 8);


//
// Host to ME message
//
typedef struct _TOUCH_SENSOR_MSG_H2M
{
    u32  CommandCode;
    union
    {
        TOUCH_SENSOR_SET_MODE_CMD_DATA          SetModeCmdData;
        TOUCH_SENSOR_SET_MEM_WINDOW_CMD_DATA    SetMemWindowCmdData;
        TOUCH_SENSOR_QUIESCE_IO_CMD_DATA        QuiesceIoCmdData;
        TOUCH_SENSOR_FEEDBACK_READY_CMD_DATA    FeedbackReadyCmdData;
        TOUCH_SENSOR_SET_POLICIES_CMD_DATA      SetPoliciesCmdData;
        TOUCH_SENSOR_RESET_CMD_DATA             ResetCmdData;
    } H2MData;
} TOUCH_SENSOR_MSG_H2M;
C_ASSERT(sizeof(TOUCH_SENSOR_MSG_H2M) == 324);


//*******************************************************************
//
// Defines for message structures used for ME to Host communication
//
//*******************************************************************

// I/O mode values used by TOUCH_SENSOR_GET_DEVICE_INFO_RSP_DATA.
typedef enum _TOUCH_SPI_IO_MODE
{
    TOUCH_SPI_IO_MODE_SINGLE = 0,   // Sensor set for Single I/O SPI
    TOUCH_SPI_IO_MODE_DUAL,         // Sensor set for Dual I/O SPI
    TOUCH_SPI_IO_MODE_QUAD,         // Sensor set for Quad I/O SPI
    TOUCH_SPI_IO_MODE_MAX           // Invalid value
} TOUCH_SPI_IO_MODE;
C_ASSERT(sizeof(TOUCH_SPI_IO_MODE) == 4);

//
// TOUCH_SENSOR_GET_DEVICE_INFO_RSP code is sent in response to TOUCH_SENSOR_GET_DEVICE_INFO_CMD. This code will be followed
// by TOUCH_SENSOR_GET_DEVICE_INFO_RSP_DATA.
//
// Possible Status values:
//      TOUCH_STATUS_SUCCESS:               Command was processed successfully and sensor details are reported.
//      TOUCH_STATUS_CMD_SIZE_ERROR:        Command sent did not match expected size. Other fields will not contain valid data.
//      TOUCH_STATUS_NO_SENSOR_FOUND:       Sensor has not yet been detected. Other fields will not contain valid data.
//      TOUCH_STATUS_INVALID_DEVICE_CAPS:   Indicates sensor does not support minimum required Frequency or I/O Mode. ME firmware will choose best possible option for the errant
//                                          field. Caller should attempt to continue.
//      TOUCH_STATUS_COMPAT_CHECK_FAIL:     Indicates TouchIC/ME compatibility mismatch. Caller should attempt to continue.
//
typedef struct _TOUCH_SENSOR_GET_DEVICE_INFO_RSP_DATA
{
    TOUCH_STATUS        Status;                 // See description above for possible Status values
    u16              VendorId;               // Touch Sensor vendor ID
    u16              DeviceId;               // Touch Sensor device ID
    u32              HwRev;                  // Touch Sensor Hardware Revision
    u32              FwRev;                  // Touch Sensor Firmware Revision
    u32              FrameSize;              // Max size of one frame returned by Touch IC in bytes. This data will be TOUCH_RAW_DATA_HDR followed
                                                // by a payload. The payload can be raw data or a HID structure depending on mode.
    u32              FeedbackSize;           // Max size of one Feedback structure in bytes
    TOUCH_SENSOR_MODE   SensorMode;             // Current operating mode of the sensor
    u32              MaxTouchPoints  :8;     // Maximum number of simultaneous touch points that can be reported by sensor
    TOUCH_FREQ          SpiFrequency    :8;     // SPI bus Frequency supported by sensor and ME firmware
    TOUCH_SPI_IO_MODE   SpiIoMode       :8;     // SPI bus I/O Mode supported by sensor and ME firmware
    u32              Reserved0       :8;     // For future expansion
    u8               SensorMinorEdsRev;      // Minor version number of EDS spec supported by sensor (from Compat Rev ID Reg)
    u8               SensorMajorEdsRev;      // Major version number of EDS spec supported by sensor (from Compat Rev ID Reg)
    u8               MeMinorEdsRev;          // Minor version number of EDS spec supported by ME
    u8               MeMajorEdsRev;          // Major version number of EDS spec supported by ME
    u8               SensorEdsIntfRev;       // EDS Interface Revision Number supported by sensor (from Compat Rev ID Reg)
    u8               MeEdsIntfRev;           // EDS Interface Revision Number supported by ME
    u8               KernelCompatVer;        // EU Kernel Compatibility Version  (from Compat Rev ID Reg)
    u8               Reserved1;              // For future expansion
    u32              Reserved2[2];           // For future expansion
} TOUCH_SENSOR_GET_DEVICE_INFO_RSP_DATA;
C_ASSERT(sizeof(TOUCH_SENSOR_GET_DEVICE_INFO_RSP_DATA) == 48);


//
// TOUCH_SENSOR_SET_MODE_RSP code is sent in response to TOUCH_SENSOR_SET_MODE_CMD. This code will be followed
// by TOUCH_SENSOR_SET_MODE_RSP_DATA.
//
// Possible Status values:
//      TOUCH_STATUS_SUCCESS:           Command was processed successfully and mode was set.
//      TOUCH_STATUS_CMD_SIZE_ERROR:    Command sent did not match expected size. Other fields will not contain valid data.
//      TOUCH_STATUS_INVALID_PARAMS:    Input parameters are out of range.
//
typedef struct _TOUCH_SENSOR_SET_MODE_RSP_DATA
{
    TOUCH_STATUS    Status;         // See description above for possible Status values
    u32          Reserved[3];    // For future expansion
} TOUCH_SENSOR_SET_MODE_RSP_DATA;
C_ASSERT(sizeof(TOUCH_SENSOR_SET_MODE_RSP_DATA) == 16);


//
// TOUCH_SENSOR_SET_MEM_WINDOW_RSP code is sent in response to TOUCH_SENSOR_SET_MEM_WINDOW_CMD. This code will be followed
// by TOUCH_SENSOR_SET_MEM_WINDOW_RSP_DATA.
//
// Possible Status values:
//      TOUCH_STATUS_SUCCESS:           Command was processed successfully and memory window was set.
//      TOUCH_STATUS_CMD_SIZE_ERROR:    Command sent did not match expected size. Other fields will not contain valid data.
//      TOUCH_STATUS_INVALID_PARAMS:    Input parameters are out of range.
//      TOUCH_STATUS_ACCESS_DENIED:     Unable to map host address ranges for DMA.
//      TOUCH_STATUS_OUT_OF_MEMORY:     Unable to allocate enough space for needed buffers.
//
typedef struct _TOUCH_SENSOR_SET_MEM_WINDOW_RSP_DATA
{
    TOUCH_STATUS    Status;         // See description above for possible Status values
    u32          Reserved[3];    // For future expansion
} TOUCH_SENSOR_SET_MEM_WINDOW_RSP_DATA;
C_ASSERT(sizeof(TOUCH_SENSOR_SET_MEM_WINDOW_RSP_DATA) == 16);


//
// TOUCH_SENSOR_QUIESCE_IO_RSP code is sent in response to TOUCH_SENSOR_QUIESCE_IO_CMD. This code will be followed
// by TOUCH_SENSOR_QUIESCE_IO_RSP_DATA.
//
// Possible Status values:
//      TOUCH_STATUS_SUCCESS:                   Command was processed successfully and touch flow has stopped.
//      TOUCH_STATUS_CMD_SIZE_ERROR:            Command sent did not match expected size. Other fields will not contain valid data.
//      TOUCH_STATUS_QUIESCE_IO_IN_PROGRESS:    Indicates that Quiesce I/O is already in progress and this command cannot be accepted at this time.
//      TOUCH_STATIS_TIMEOUT:                   Indicates ME timed out waiting for Quiesce I/O flow to complete.
//
typedef struct _TOUCH_SENSOR_QUIESCE_IO_RSP_DATA
{
    TOUCH_STATUS    Status;         // See description above for possible Status values
    u32          Reserved[3];    // For future expansion
} TOUCH_SENSOR_QUIESCE_IO_RSP_DATA;
C_ASSERT(sizeof(TOUCH_SENSOR_QUIESCE_IO_RSP_DATA) == 16);


// Reset Reason values used in TOUCH_SENSOR_HID_READY_FOR_DATA_RSP_DATA
typedef enum _TOUCH_RESET_REASON
{
    TOUCH_RESET_REASON_UNKNOWN = 0,         // Reason for sensor reset is not known
    TOUCH_RESET_REASON_FEEDBACK_REQUEST,    // Reset was requested as part of TOUCH_SENSOR_FEEDBACK_READY_CMD
    TOUCH_RESET_REASON_HECI_REQUEST,        // Reset was requested via TOUCH_SENSOR_RESET_CMD
    TOUCH_RESET_REASON_MAX
} TOUCH_RESET_REASON;
C_ASSERT(sizeof(TOUCH_RESET_REASON) == 4);

//
// TOUCH_SENSOR_HID_READY_FOR_DATA_RSP code is sent in response to TOUCH_SENSOR_HID_READY_FOR_DATA_CMD. This code will be followed
// by TOUCH_SENSOR_HID_READY_FOR_DATA_RSP_DATA.
//
// Possible Status values:
//      TOUCH_STATUS_SUCCESS:                   Command was processed successfully and HID data was sent by DMA. This will only be sent in HID mode.
//      TOUCH_STATUS_CMD_SIZE_ERROR:            Command sent did not match expected size. Other fields will not contain valid data.
//      TOUCH_STATUS_REQUEST_OUTSTANDING:       Previous request is still outstanding, ME FW cannot handle another request for the same command.
//      TOUCH_STATUS_NOT_READY:                 Indicates memory window has not yet been set by BIOS/HID.
//      TOUCH_STATUS_SENSOR_DISABLED:           Indicates that ME to HID communication has been stopped either by TOUCH_SENSOR_QUIESCE_IO_CMD or TOUCH_SENSOR_CLEAR_MEM_WINDOW_CMD.
//      TOUCH_STATUS_SENSOR_UNEXPECTED_RESET:   Sensor signaled a Reset Interrupt. ME did not expect this and has no info about why this occurred.
//      TOUCH_STATUS_SENSOR_EXPECTED_RESET:     Sensor signaled a Reset Interrupt. ME either directly requested this reset, or it was expected as part of a defined flow in the EDS.
//      TOUCH_STATUS_QUIESCE_IO_IN_PROGRESS:    Indicates that Quiesce I/O is already in progress and this command cannot be accepted at this time.
//      TOUCH_STATUS_TIMEOUT:                   Sensor did not generate a reset interrupt in the time allotted. Could indicate sensor is not connected or malfunctioning.
//
typedef struct _TOUCH_SENSOR_HID_READY_FOR_DATA_RSP_DATA
{
    TOUCH_STATUS    Status;                 // See description above for possible Status values
    u32          DataSize;               // Size of the data the ME DMA'd into a RawDataBuffer. Valid only when Status == TOUCH_STATUS_SUCCESS
    u8           TouchDataBufferIndex;   // Index to indicate which RawDataBuffer was used. Valid only when Status == TOUCH_STATUS_SUCCESS
    u8           ResetReason;            // If Status is TOUCH_STATUS_SENSOR_EXPECTED_RESET, ME will provide the cause. See TOUCH_RESET_REASON.
    u8           Reserved1[2];           // For future expansion
    u32          Reserved2[5];           // For future expansion
} TOUCH_SENSOR_HID_READY_FOR_DATA_RSP_DATA;
C_ASSERT(sizeof(TOUCH_SENSOR_HID_READY_FOR_DATA_RSP_DATA) == 32);


//
// TOUCH_SENSOR_FEEDBACK_READY_RSP code is sent in response to TOUCH_SENSOR_FEEDBACK_READY_CMD. This code will be followed
// by TOUCH_SENSOR_FEEDBACK_READY_RSP_DATA.
//
// Possible Status values:
//      TOUCH_STATUS_SUCCESS:           Command was processed successfully and any feedback or commands were sent to sensor.
//      TOUCH_STATUS_CMD_SIZE_ERROR:    Command sent did not match expected size. Other fields will not contain valid data.
//      TOUCH_STATUS_INVALID_PARAMS:    Input parameters are out of range.
//      TOUCH_STATUS_COMPAT_CHECK_FAIL  Indicates ProtocolVer does not match ME supported version. (non-fatal error)
//      TOUCH_STATUS_INTERNAL_ERROR:    Unexpected error occurred. This should not normally be seen.
//      TOUCH_STATUS_OUT_OF_MEMORY:     Insufficient space to store Calibration Data
//
typedef struct _TOUCH_SENSOR_FEEDBACK_READY_RSP_DATA
{
    TOUCH_STATUS    Status;         // See description above for possible Status values
    u8           FeedbackIndex;  // Index value from 0 to TOUCH_SENSOR_MAX_DATA_BUFFERS used to indicate which Feedback Buffer to use
    u8           Reserved1[3];   // For future expansion
    u32          Reserved2[6];   // For future expansion
} TOUCH_SENSOR_FEEDBACK_READY_RSP_DATA;
C_ASSERT(sizeof(TOUCH_SENSOR_FEEDBACK_READY_RSP_DATA) == 32);


//
// TOUCH_SENSOR_CLEAR_MEM_WINDOW_RSP code is sent in response to TOUCH_SENSOR_CLEAR_MEM_WINDOW_CMD. This code will be followed
// by TOUCH_SENSOR_CLEAR_MEM_WINDOW_RSP_DATA.
//
// Possible Status values:
//      TOUCH_STATUS_SUCCESS:                   Command was processed successfully and memory window was set.
//      TOUCH_STATUS_CMD_SIZE_ERROR:            Command sent did not match expected size. Other fields will not contain valid data.
//      TOUCH_STATUS_INVALID_PARAMS:            Input parameters are out of range.
//      TOUCH_STATUS_QUIESCE_IO_IN_PROGRESS:    Indicates that Quiesce I/O is already in progress and this command cannot be accepted at this time.
//
typedef struct _TOUCH_SENSOR_CLEAR_MEM_WINDOW_RSP_DATA
{
    TOUCH_STATUS    Status;
    u32          Reserved[3];    // For future expansion
} TOUCH_SENSOR_CLEAR_MEM_WINDOW_RSP_DATA;
C_ASSERT(sizeof(TOUCH_SENSOR_CLEAR_MEM_WINDOW_RSP_DATA) == 16);


//
// TOUCH_SENSOR_NOTIFY_DEV_READY_RSP code is sent in response to TOUCH_SENSOR_NOTIFY_DEV_READY_CMD. This code will be followed
// by TOUCH_SENSOR_NOTIFY_DEV_READY_RSP_DATA.
//
// Possible Status values:
//      TOUCH_STATUS_SUCCESS:               Command was processed successfully and sensor has been detected by ME FW.
//      TOUCH_STATUS_CMD_SIZE_ERROR:        Command sent did not match expected size.
//      TOUCH_STATUS_REQUEST_OUTSTANDING:   Previous request is still outstanding, ME FW cannot handle another request for the same command.
//      TOUCH_STATUS_TIMEOUT:               Sensor did not generate a reset interrupt in the time allotted. Could indicate sensor is not connected or malfunctioning.
//      TOUCH_STATUS_SENSOR_FAIL_FATAL:     Sensor indicated a fatal error, further operation is not possible. Error details can be found in ErrReg.
//      TOUCH_STATUS_SENSOR_FAIL_NONFATAL:  Sensor indicated a non-fatal error. Error should be logged by caller and init flow can continue. Error details can be found in ErrReg.
//
typedef struct _TOUCH_SENSOR_NOTIFY_DEV_READY_RSP_DATA
{
    TOUCH_STATUS    Status;         // See description above for possible Status values
    TOUCH_ERR_REG   ErrReg;         // Value of sensor Error Register, field is only valid for Status == TOUCH_STATUS_SENSOR_FAIL_FATAL or TOUCH_STATUS_SENSOR_FAIL_NONFATAL
    u32          Reserved[2];    // For future expansion
} TOUCH_SENSOR_NOTIFY_DEV_READY_RSP_DATA;
C_ASSERT(sizeof(TOUCH_SENSOR_NOTIFY_DEV_READY_RSP_DATA) == 16);


//
// TOUCH_SENSOR_SET_POLICIES_RSP code is sent in response to TOUCH_SENSOR_SET_POLICIES_CMD. This code will be followed
// by TOUCH_SENSOR_SET_POLICIES_RSP_DATA.
//
// Possible Status values:
//      TOUCH_STATUS_SUCCESS:           Command was processed successfully and new policies were set.
//      TOUCH_STATUS_CMD_SIZE_ERROR:    Command sent did not match expected size. Other fields will not contain valid data.
//      TOUCH_STATUS_INVALID_PARAMS:    Input parameters are out of range.
//
typedef struct _TOUCH_SENSOR_SET_POLICIES_RSP_DATA
{
    TOUCH_STATUS    Status;         // See description above for possible Status values
    u32          Reserved[3];    // For future expansion
} TOUCH_SENSOR_SET_POLICIES_RSP_DATA;
C_ASSERT(sizeof(TOUCH_SENSOR_SET_POLICIES_RSP_DATA) == 16);


//
// TOUCH_SENSOR_GET_POLICIES_RSP code is sent in response to TOUCH_SENSOR_GET_POLICIES_CMD. This code will be followed
// by TOUCH_SENSOR_GET_POLICIES_RSP_DATA.
//
// Possible Status values:
//      TOUCH_STATUS_SUCCESS:           Command was processed successfully and new policies were set.
//      TOUCH_STATUS_CMD_SIZE_ERROR:    Command sent did not match expected size. Other fields will not contain valid data.
//
typedef struct _TOUCH_SENSOR_GET_POLICIES_RSP_DATA
{
    TOUCH_STATUS        Status;             // See description above for possible Status values
    TOUCH_POLICY_DATA   PolicyData;         // Contains the current policy
} TOUCH_SENSOR_GET_POLICIES_RSP_DATA;
C_ASSERT(sizeof(TOUCH_SENSOR_GET_POLICIES_RSP_DATA) == 20);


//
// TOUCH_SENSOR_RESET_RSP code is sent in response to TOUCH_SENSOR_RESET_CMD. This code will be followed
// by TOUCH_SENSOR_RESET_RSP_DATA.
//
// Possible Status values:
//      TOUCH_STATUS_SUCCESS:                   Command was processed successfully and sensor reset was completed.
//      TOUCH_STATUS_CMD_SIZE_ERROR:            Command sent did not match expected size. Other fields will not contain valid data.
//      TOUCH_STATUS_INVALID_PARAMS:            Input parameters are out of range.
//      TOUCH_STATUS_TIMEOUT:                   Sensor did not generate a reset interrupt in the time allotted. Could indicate sensor is not connected or malfunctioning.
//      TOUCH_STATUS_RESET_FAILED:              Sensor generated an invalid or unexpected interrupt.
//      TOUCH_STATUS_QUIESCE_IO_IN_PROGRESS:    Indicates that Quiesce I/O is already in progress and this command cannot be accepted at this time.
//
typedef struct _TOUCH_SENSOR_RESET_RSP_DATA
{
    TOUCH_STATUS    Status;
    u32          Reserved[3];    // For future expansion
} TOUCH_SENSOR_RESET_RSP_DATA;
C_ASSERT(sizeof(TOUCH_SENSOR_RESET_RSP_DATA) == 16);


//
// TOUCH_SENSOR_READ_ALL_REGS_RSP code is sent in response to TOUCH_SENSOR_READ_ALL_REGS_CMD. This code will be followed
// by TOUCH_SENSOR_READ_ALL_REGS_RSP_DATA.
//
// Possible Status values:
//      TOUCH_STATUS_SUCCESS:           Command was processed successfully and new policies were set.
//      TOUCH_STATUS_CMD_SIZE_ERROR:    Command sent did not match expected size. Other fields will not contain valid data.
//
typedef struct _TOUCH_SENSOR_READ_ALL_REGS_RSP_DATA
{
    TOUCH_STATUS    Status;
    TOUCH_REG_BLOCK SensorRegs; // Returns first 64 bytes of register space used for normal touch operation. Does not include test mode register.
    u32          Reserved[4];
} TOUCH_SENSOR_READ_ALL_REGS_RSP_DATA;
C_ASSERT(sizeof(TOUCH_SENSOR_READ_ALL_REGS_RSP_DATA) == 84);

//
// ME to Host Message
//
typedef struct _TOUCH_SENSOR_MSG_M2H
{
    u32  CommandCode;
    union
    {
        TOUCH_SENSOR_GET_DEVICE_INFO_RSP_DATA       DeviceInfoRspData;
        TOUCH_SENSOR_SET_MODE_RSP_DATA              SetModeRspData;
        TOUCH_SENSOR_SET_MEM_WINDOW_RSP_DATA        SetMemWindowRspData;
        TOUCH_SENSOR_QUIESCE_IO_RSP_DATA            QuiesceIoRspData;
        TOUCH_SENSOR_HID_READY_FOR_DATA_RSP_DATA    HidReadyForDataRspData;
        TOUCH_SENSOR_FEEDBACK_READY_RSP_DATA        FeedbackReadyRspData;
        TOUCH_SENSOR_CLEAR_MEM_WINDOW_RSP_DATA      ClearMemWindowRspData;
        TOUCH_SENSOR_NOTIFY_DEV_READY_RSP_DATA      NotifyDevReadyRspData;
        TOUCH_SENSOR_SET_POLICIES_RSP_DATA          SetPoliciesRspData;
        TOUCH_SENSOR_GET_POLICIES_RSP_DATA          GetPoliciesRspData;
        TOUCH_SENSOR_RESET_RSP_DATA                 ResetRspData;
		TOUCH_SENSOR_READ_ALL_REGS_RSP_DATA         ReadAllRegsRspData;
    } M2HData;
} TOUCH_SENSOR_MSG_M2H;
C_ASSERT(sizeof(TOUCH_SENSOR_MSG_M2H) == 88);


#define TOUCH_MSG_SIZE_MAX_BYTES    (MAX(sizeof(TOUCH_SENSOR_MSG_M2H), sizeof(TOUCH_SENSOR_MSG_H2M)))

#pragma pack()

#endif //_TOUCH_HECI_MSGS_H
