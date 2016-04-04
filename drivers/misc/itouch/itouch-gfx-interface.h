/*
 *
 * Intel Management Engine Interface (Intel MEI) Integrated Touch Gfx Interface
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

#ifndef _ITOUCH_GFX_INTERFACE_H_
#define _ITOUCH_GFX_INTERFACE_H_

#include "../../gpu/drm/i915/i915_drv.h"
#include "../../gpu/drm/i915/intel_guc.h"

#define ITOUCH_INTERFACE_VERSION 9

#define BOOL bool
#define UCHAR u8
#define SHORT s16
#define USHORT u16
#define ULONG	u32
#define DWORD ULONG
#define ULONG64 u64
#define ULONGLONG u64
#define ADDRESS32 u32
#define ADDRESS64 u64
#define KM_BIT_RANGE(endbit, startbit)  (endbit-startbit+1)
#define KM_BIT(bit) (1)


#pragma pack(push, 1)

//*****************************************************************************
// Struct:  UK_DOORBELL_QW
// PURPOSE: Doorbell QW description
//*****************************************************************************
typedef struct UK_DOORBELL_QW_REC {
	union {
		struct {
			ULONG DoorBellStatus;	// 0 / 1
			ULONG Cookie;	// Only KMD/uKernel should write 0 here. App should rollover to 1.
		};
		ULONG64 QuadPart;
	};

} UK_DOORBELL_QW;
#define KM_DOORBELL_ENABLED (1)
#define KM_DOORBELL_DISABLED (0)

typedef struct UK_ENGINES_USED_REC {
	union {
		struct {
			// This sequence of engines (i.e bit positions) must match the corresponding IGFX_ENGINE enum value
			UCHAR RenderEngine:KM_BIT(0);
			UCHAR VideoEngine:KM_BIT(1);
			UCHAR BltEngine:KM_BIT(2);
			UCHAR VEEngine:KM_BIT(3);
			UCHAR VideoEngine2:KM_BIT(4);
			 UCHAR:KM_BIT_RANGE(7, 5);
		};
		UCHAR Value;
	};
} IGFX_ENGINES_USED;

//*****************************************************************************
// Enum:  UK_CONTEXT_PRIORITY
// PURPOSE:  To indicate the priority of UK context.
//*****************************************************************************
typedef enum {
	UK_CONTEXT_PRIORITY_KMD_HIGH = 0,	// KMD priority is only for SKL+
	UK_CONTEXT_PRIORITY_HIGH = 1,
	UK_CONTEXT_PRIORITY_KMD_NORMAL = 2,	// KMD priority is only for SKL+
	UK_CONTEXT_PRIORITY_NORMAL = 3,
	UK_CONTEXT_PRIORITY_ABSOLUTE_MAX_COUNT,
	UK_CONTEXT_PRIORITY_INVALID = UK_CONTEXT_PRIORITY_ABSOLUTE_MAX_COUNT
} UK_CONTEXT_PRIORITY;

//*****************************************************************************
// Enum:  UK_OPEN_GPU_FLAGS
// PURPOSE: A bit flag for each engines used. Used for making some memory optimizations.
//*****************************************************************************
typedef struct UK_OPEN_GPU_FLAGS_REC {
	union {
		struct {
			// This sequence of engines (i.e bit positions) must match the corresponding IGFX_ENGINE enum value,
			ULONG AcquireDoorBell:1;
			ULONG InhibitSyncContextSwitchOnWaitEvents:1;
			ULONG DoorBellForPCHUse:1;
			 ULONG:29;
		};
		ULONG Value;
	};
} UK_OPEN_GPU_FLAGS;

//*****************************************************************************
// Enum:  KM_GUC_OP_CHECK_CODE
// PURPOSE: A non random pattern used to check input
//*****************************************************************************
typedef enum KM_GUC_OP_CHECK_CODE_ENUM {
	KM_GUC_OP_CHECK_CODE_OPEN_GPU = 0xAA229121,
	KM_GUC_OP_CHECK_CODE_CLOSE_GPU = 0x9120AA22,
	KM_GUC_OP_CHECK_CODE_ACQUIRE_DOORBELL = 0xCDDEBBF3,

} KM_GUC_OP_CHECK_CODE;

//*****************************************************************************
// Enum:  KM_GUC_STATUS
// PURPOSE: Status codes
//*****************************************************************************
typedef enum KM_GUC_STATUS_ENUM {
	KM_GUC_STATUS_SUCCESS = 0x0,
	KM_GUC_STATUS_INVALID_PARAMS,
	KM_GUC_STATUS_INVALID_CHECK_CODE,
	KM_GUC_STATUS_ERROR,

	KM_GUC_STATUS_ERROR_MAX
} KM_GUC_STATUS;

//*****************************************************************************
// Enum:  UK_QUEUE_STATUS
// PURPOSE:  to indicate the status of a Micro Kernel's Work/Submit Queue.
//  TBD: Define what state means what...
//*****************************************************************************
typedef enum {
	UK_QSTATUS_ACTIVE = 1,	//Work queue will be serviced if doorbell is owned by app
	UK_QSTATUS_SUSPENDED,	//Work queue will not be serviced.
	UK_QSTATUS_CMD_ERROR,	//Error in work queue
	UK_QUEUE_STATUS_ENGINE_ID_NOT_USED,
	UK_QSTATUS_SUSPENDED_FROM_ENGINE_RESET,	//Suspended due to engine reset. App must consider all work before WQ tail as in error and resubmit necessary work.
	UK_QSTATUS_INVALID_STATUS
} UK_QUEUE_STATUS;

//*****************************************************************************
// Struct:  SCHED_CONTEXT_ENGINE_PRESENCE
// PURPOSE: Per Context Engine Status
//*****************************************************************************
typedef struct SCHED_CONTEXT_ENGINE_PRESENCE_STRUCT {
	union {
		struct {
			ULONG IsContextInRcsSubmitQueue:KM_BIT(0);
			ULONG IsContextInVcsSubmitQueue:KM_BIT(1);
			ULONG IsContextInBcsSubmitQueue:KM_BIT(2);
			ULONG IsContextInVecsSubmitQueue:KM_BIT(3);
			ULONG IsContextInVcs2SubmitQueue:KM_BIT(4);
			 ULONG:KM_BIT_RANGE(30, 5);
			 ULONG:KM_BIT(31);	//Used to be IsSubmitQueueElementValid
		};
		ULONG SubmitQueuePresenceValue;
	};

} SCHED_CONTEXT_ENGINE_PRESENCE;

//*****************************************************************************
// Struct:  UK_SCHED_PROCESS_DESCRIPTOR
// PURPOSE: A shared structure between app and uKernel communication.
//*****************************************************************************
typedef struct UK_SCHED_PROCESS_DESCRIPTOR_REC {
	ULONG ContextId;
	// Trigger / doorbell address for App - Reserved in uK space
	ADDRESS64 pDoorbell;	// For Read-Write access by App. Ptr to UK_KM_APP_DOORBELL_INFO
	ULONG HeadOffset;	// Byte Offset - App must not write here.
	ULONG TailOffset;	// Byte Offset - uKernel will not write here.
	ULONG ErrorOffsetByte;
	ADDRESS64 WQVBaseAddress;	// pVirt in app - for use only by application
	ULONG WQSizeBytes;
	UK_QUEUE_STATUS WQStatus;	// Read by App. Written by uKernel/KMD.
	SCHED_CONTEXT_ENGINE_PRESENCE ContextEnginePresence;	// Read by App. Written by uKernel. A snapshot of context descriptor's copy. Updates only at Context Complete and WI Process complete.
	UK_CONTEXT_PRIORITY PriorityAssigned;	// Read only by app
	ULONG Reserved[4];	// Reserved

	//Tracking variables for debug
	ULONG WorkItemTrackingEnabled;	// Status set by KMD, Read only for App. For speed reasons, tracking is enabled for all contexts or none.
	ULONG AppTotalNumberOfWorkItemsAdded;	// App must set to 0 upon return from OpenGPU, before using it. Written only by Application - Total number of Work Items added to WQ so far (all Engines, monotonic increase)
	ULONG AppReserved2[3];

	//uKernel side tracking for debug
	ULONG TotalWorkItemsParsedByGuC;	// Written by uKernel at the time of parsing and successfull removed from WQ (implies ring tail was updated).
	ULONG TotalWorkItemsCollapsedByGuC;	// Written by uKernel if a WI was collapsed if next WI is the same LRCA (optimization applies only to Secure/KMD contexts)
	ULONG TotalWorkItemsCancelled;	// Written by uKernel if a WI was cancelled due to preempt.

	ULONG IsContextInEngineReset;	// Tells if the context is affected in Engine Reset. UMD needs to clear it after taking appropriate Action(TBD).
	ULONG EngineResetSampledWQTail;	// WQ Sampled tail at Engine Reset Time. Valid only if IsContextInEngineReset = TRUE
	ULONG EngineResetSampledWQTailValid;	// Valid from Engine reset until all the affected Work Items are processed

	ULONG GuCReserved3[15];	// Reserved

} UK_SCHED_PROCESS_DESCRIPTOR;

struct KM_GUC_OPEN_GPU_PARAM_REC {

	IGFX_ENGINES_USED EnginesUsed;
	UK_CONTEXT_PRIORITY PriorityRequested;
	UK_OPEN_GPU_FLAGS Flags;

	KM_GUC_OP_CHECK_CODE CheckCode;

	ULONG64 pAppProcessDesc;
	ULONG ProcessDescSizeBytes;
	UK_CONTEXT_PRIORITY PriorityAssigned;
	ULONG64 GpuHandle;
	KM_GUC_STATUS Status;

	/*TBD: This should not be required */
	struct page *db_page;
	ULONG64 pDoorbell;
	ULONG64 pTailOffset;
};

struct KM_GUC_CLOSE_GPU_PARAM_REC {

	void *pAppProcessDesc;	// In: User Mode accessible pointer to UK_SCHED_PROCESS_DESCRIPTOR
	ULONG64 GpuHandle;	// In: Handle to the GPU allocated.

	KM_GUC_OP_CHECK_CODE CheckCode;	// In: CheckCode == KM_GUC_OP_CHECK_CODE_CLOSE_GPU.
	//     A well defined cookie so random input can be
	//     rejected (fuzzed input security tests)

	KM_GUC_STATUS Status;	// Out: Status of operation
};

typedef struct KM_GUC_OPEN_GPU_PARAM_REC KM_GUC_OPEN_GPU_PARAM;
typedef struct KM_GUC_CLOSE_GPU_PARAM_REC KM_GUC_CLOSE_GPU_PARAM;

typedef uint64_t HANDLE;

#define ITOUCH_ALLOC_CONTIGUOUS	0x1

typedef struct {
	HANDLE hContextID;	/* Input: Context handle received from pfnOpenGPU */
	unsigned int size;	/* Input: Size of buffer in bytes */

	HANDLE hGmmBlock;	/* Output: Handle to memory block allocated on gfx */
	void *pDevVA;		/* Output: Pointer to device VA */
	void *pCpuVA;		/* Output: Pointer to cpu accessible VA */
	u32 flags;		/* Flags to control type of  allocation */
	void *pCpuPA;
} HidGfxMapBuffer, *PHidGfxMapBuffer;

typedef int (*PFN_OPEN_GPU) (HANDLE, KM_GUC_OPEN_GPU_PARAM *);
typedef int (*PFN_CLOSE_GPU) (HANDLE, KM_GUC_CLOSE_GPU_PARAM *);
typedef int (*PFN_MAP_BUFFER) (HANDLE, HidGfxMapBuffer *);
typedef int (*PFN_UNMAP_BUFFER) (HANDLE, HANDLE);
typedef int (*PFN_SIGNAL_UNLOAD) (HANDLE);
typedef int (*PFN_SUBMIT_KERNEL) (HANDLE, uint64_t, unsigned int,
				  unsigned long);

typedef struct {
	uint gfxCore;
	uint revId;
} QUERY_GPU_PARAMS, *PQUERY_GPU_PARAMS;

typedef int (*PFN_QUERY_GPU) (HANDLE, PQUERY_GPU_PARAMS);
typedef struct {
	PFN_OPEN_GPU pfnOpenGPU;	/* Initialize the GUC and get a doorbell */
	PFN_CLOSE_GPU pfnCloseGPU;	/* Close the GPU */
	PFN_MAP_BUFFER pfnMapBuffer;	/* Map a buffer into GFX address space */
	PFN_UNMAP_BUFFER pfnUnmapBuffer;	/* Unmap buffer */
	PFN_SIGNAL_UNLOAD pfnSignalUnload;	/* Signal GFX when HID driver is unloaded */
	PFN_SUBMIT_KERNEL pfnSubmitKernel;	/* Submit specified kernel to gfx */
} GfxFunctionTable, *PGfxFunctionTable;

typedef int (*PFN_SIGNAL_COMPLETE) (HANDLE, unsigned long);	/* Signal GFX is done with last touch processing */

typedef struct {
	PFN_SIGNAL_COMPLETE pfnSignalComplete;
} HidFunctionTable, *PHidFunctionTable;

typedef struct {
	/* hidVersion and gfxVersion should always be the first two members */
	unsigned int hidVersion;	/* Input: Version of header used by HID */
	unsigned int gfxVersion;	/* Output: Version of header used by GFX */

	HidFunctionTable *pHidTable;	/* Input: Passed from Hid to Gfx */
	HANDLE hHidContext;	/* Input: handle to HID context */

	GfxFunctionTable *pGfxTable;	/* Output: Passed back from Gfx to Hid */
	HANDLE hGfxContext;	/* Output: handle to graphics context */
} HidGfxCallbacks, *PHidGfxCallbacks;

typedef enum {
	IGFX_UNKNOWN_CORE = 0,
	IGFX_GEN3_CORE = 1,	//Gen3 Family
	IGFX_GEN3_5_CORE = 2,	//Gen3.5 Family
	IGFX_GEN4_CORE = 3,	//Gen4 Family
	IGFX_GEN4_5_CORE = 4,	//Gen4.5 Family
	IGFX_GEN5_CORE = 5,	//Gen5 Family
	IGFX_GEN5_5_CORE = 6,	//Gen5.5 Family
	IGFX_GEN5_75_CORE = 7,	//Gen5.75 Family
	IGFX_GEN6_CORE = 8,	//Gen6 Family
	IGFX_GEN7_CORE = 9,	//Gen7 Family
	IGFX_GEN7_5_CORE = 10,	//Gen7.5 Family
	IGFX_GEN8_CORE = 11,	//Gen8 Family
	IGFX_GEN9_CORE = 12,	//Gen9 Family
	IGFX_GEN10_CORE = 13,	//Gen10 Family

	IGFX_GENNEXT_CORE = 0x7ffffffe,	//GenNext
	GFXCORE_FAMILY_FORCE_ULONG = 0x7fffffff
} GFXCORE_FAMILY;

/******************************
KM_EXECLIST_RING_CONTEXT_REGISTER - Must Match BSpec
******************************/
#define KM_MAX_ELEMENTS_PER_EXEC_LIST  (2)

typedef struct
{
    ULONG   Offset;
    ULONG   Data;
} KM_EXECLIST_RING_CONTEXT_REGISTER;

/******************************
KM_EXECLIST_RING_CONTEXT - Must Match BSpec
******************************/

typedef struct KM_EXECLIST_RING_CONTEXT_REC
{
    ULONG   Noop;
    ULONG   RingInfoLriHeader;

    KM_EXECLIST_RING_CONTEXT_REGISTER    ContextControl;
    KM_EXECLIST_RING_CONTEXT_REGISTER    RingHead;
    KM_EXECLIST_RING_CONTEXT_REGISTER    RingTail;
    KM_EXECLIST_RING_CONTEXT_REGISTER    RingBufferStart;
    KM_EXECLIST_RING_CONTEXT_REGISTER    RingBufferControl;
    KM_EXECLIST_RING_CONTEXT_REGISTER    BatchBufferCurrentHead_Udw;
    KM_EXECLIST_RING_CONTEXT_REGISTER    BatchBufferCurrentHead;
    KM_EXECLIST_RING_CONTEXT_REGISTER    BatchBufferState;

    KM_EXECLIST_RING_CONTEXT_REGISTER    SecondBatchBufferAddr_Udw;
    KM_EXECLIST_RING_CONTEXT_REGISTER    SecondBatchBufferAddr;
    KM_EXECLIST_RING_CONTEXT_REGISTER    SecondBatchBufferState;

    ULONG   Padding1[9];                // 9 Noops

    ULONG   PdpInfoLriHeader;

    KM_EXECLIST_RING_CONTEXT_REGISTER    ContextTimestamp;

    KM_EXECLIST_RING_CONTEXT_REGISTER    Pdp3_UDW;
    KM_EXECLIST_RING_CONTEXT_REGISTER    Pdp3_LDW;

    KM_EXECLIST_RING_CONTEXT_REGISTER    Pdp2_UDW;
    KM_EXECLIST_RING_CONTEXT_REGISTER    Pdp2_LDW;

    KM_EXECLIST_RING_CONTEXT_REGISTER    Pdp1_UDW;
    KM_EXECLIST_RING_CONTEXT_REGISTER    Pdp1_LDW;

    KM_EXECLIST_RING_CONTEXT_REGISTER    Pdp0_UDW;
    KM_EXECLIST_RING_CONTEXT_REGISTER    Pdp0_LDW;

    ULONG   Padding2[14];

    KM_EXECLIST_RING_CONTEXT_REGISTER    RcsPwrClkState;

    ULONG   Unique_Tbd[3];  // EXECLIST_TODO: Unique registers in context?

    ULONG   Padding3[9];
} KM_EXECLIST_HW_RING_CONTEXT;

//*****************************************************************************
// Enum:  IGFX_ENGINE
// PURPOSE: Engine IDs available in the GFX HW
//*****************************************************************************
typedef enum
{
    // The Enum values are important as some macros and loops depend on them being sequential
    IGFX_RENDER_ENGINE       = 0,
    IGFX_VIDEO_ENGINE        = 1,
    IGFX_BLITTER_ENGINE      = 2,
    IGFX_VIDEOENHANCE_ENGINE = 3,
    IGFX_VIDEO_ENGINE2       = 4,
    IGFX_ABSOLUTE_MAX_ENGINES,
    IGFX_ENGINE_INVALID = IGFX_ABSOLUTE_MAX_ENGINES
}IGFX_ENGINE;

//*****************************************************************************
// Struct:  UK_CONTEXT_ID_MAP
// PURPOSE: To represent the context ID structure and execlist/submit queues
//*****************************************************************************
typedef struct UK_CONTEXT_ID_MAP_REC
{
    union
    {
        struct
        {
            ULONG    ContextIndex          : KM_BIT_RANGE(  19,  0);  // Index in the app context pool
			ULONG	 SubmissionByProxy	   : KM_BIT_RANGE(	20, 20);  // If KMD or other context submitted this context. This means, ContextID is LRCA[31:20]
			ULONG	 Reserved			   : KM_BIT_RANGE(	22, 21);  // Required by HW
			ULONG	 SWCounter			   : KM_BIT_RANGE(	28, 23);  // Used for tracking IOMMU group resubmits (or if submit by proxy is true, lower 6 bits QWIndex)
			ULONG    EngineId              : KM_BIT_RANGE(  31, 29);
        };
        ULONG                        ContextIdDword;
    };

}UK_CONTEXT_ID_MAP;

//*****************************************************************************
// Struct:  SCHED_CONTEXT_DESCRIPTOR_LD_REC
// PURPOSE: The default setting for Context Descriptor Format to be set in UK_ELEMENT_DESC
//*****************************************************************************
typedef struct SCHED_ELEMENT_DESC_REC
{
   union
    {
        struct
        {
            ULONG    Valid                  : KM_BIT      (      0);
            ULONG    ForcePDRestore         : KM_BIT      (      1);  //Force Page Directory
            ULONG    ForceRestore           : KM_BIT      (      2);
	    ULONG    LegacyContext          : KM_BIT      (      3);  //False only if SVM is suooported
            ULONG    CtxtAddrMode           : KM_BIT_RANGE(  4,  4);  //Addressing Mode & Legacy Context (KM_CONTEXT_ADDRESSING_MODE)
            ULONG    L3LLCCoherencySupport  : KM_BIT      (      5);
            ULONG    FaultSupportType       : KM_BIT_RANGE(  7,  6);  //Type: KM_PAGE_FAULT_MODE
            ULONG    Privilege              : KM_BIT      (      8);
            ULONG    MappedFunctionNumber   : KM_BIT_RANGE( 11,  9);
            ULONG    LRCA                   : KM_BIT_RANGE( 31,  12);
        };
        ULONG Value;
    };

}SCHED_CONTEXT_DESCRIPTOR_LD;

//*****************************************************************************
// Enum:  KM_RING_STATUS
// PURPOSE: To describe status of current ring
//*****************************************************************************

typedef enum KM_RING_STATUS_REC
{
    RING_NOT_PRESENT,
    RING_NOT_ALLOCATED,
    RING_NOT_INITALIZED,
    RING_AVAILABLE,
    RING_CURRENTLY_USED,
    RING_STOPPED,
    RING_STATUS_UNKOWN,
} KM_RING_STATUS;

//*****************************************************************************
// Struct:  UK_EXECLIST_RING_BUFFER
// PURPOSE: To describe status and access information of current ring buffer for a given EL context
//*****************************************************************************
typedef struct
{
    KM_RING_STATUS              RingStatus;                // Status
    ADDRESS32                   pExeclistRingContext;      // uKernel Pointer to ExeclistRingContext (KM_EXECLIST_HW_RING_CONTEXT)

    ADDRESS32                   pRingBegin;                // uKernel address for RingBegin
    ADDRESS32                   pRingEnd;                  // uKernel final byte address that is valid for this ring
    ADDRESS32                   pNextFreeLocation;         // uKernel address for next location in ring
                                                           //
    ULONG                       CurrentTailPointerValue;   // Last value written by software for tracking (just in case HW corrupts the tail in its context)

} UK_EXECLIST_RING_BUFFER;

//*****************************************************************************
// Struct:  UK_ELEMENT_DESC
// PURPOSE: The Execution list's element descriptor. Also in Submit Queue
//*****************************************************************************
typedef struct UK_ELEMENT_DESC_REC
{
    union
    {
        SCHED_CONTEXT_DESCRIPTOR_LD ContextDesc;
    };

    union
    {
        struct
        {
            UK_CONTEXT_ID_MAP    ContextId;
        };

        ULONG    HighDW;
    };

}UK_ELEMENT_DESC;


//*****************************************************************************
// Enum:  UK_ENGINE_EXECLIST_CONTEXT
// PURPOSE:  State of the context - this is per engine.
//*****************************************************************************
typedef struct UK_ENGINE_CONTEXT_STATE_REC
{
    union
    {
        struct
        {
            UCHAR    Submitted     : KM_BIT      (      0);
            UCHAR                  : KM_BIT_RANGE(  7,  1);
        };
        UCHAR SubmitValue;
    };
    union
    {
        struct
        {
            UCHAR    WaitForDisplayEvent    : KM_BIT      (      0);
            UCHAR    WaitForSemaphore       : KM_BIT      (      1);
            UCHAR    WaitForFaultFulfill    : KM_BIT      (      2);
            UCHAR    CATError               : KM_BIT      (      3);
            UCHAR    ReEnqueueToSubmitQueue : KM_BIT      (      4);
            UCHAR                           : KM_BIT_RANGE(  7,  5);
        };
        UCHAR WaitValue;
    };

} UK_ENGINE_CONTEXT_STATE;

//*****************************************************************************
// Struct:  UK_KM_COMMON_CONTEXT_AREA_DESCRIPTOR
// PURPOSE: A common area for communication from uKernel OS to Scheduler.
//*****************************************************************************
typedef struct UK_KM_COMMON_CONTEXT_AREA_DESCRIPTOR_REC
{

    ULONG a;

}UK_KM_COMMON_CONTEXT_AREA_DESCRIPTOR;

//*****************************************************************************
// Struct:  UK_EXECLIST_CONTEXT
// PURPOSE: The entire execlist context including software and HW information
//*****************************************************************************
typedef struct UK_EXECLIST_CONTEXT_REC
{
   UK_ELEMENT_DESC            ExecElement;
   UK_EXECLIST_RING_BUFFER    RingBufferObj;
   UK_ENGINE_CONTEXT_STATE    State;          // State holds same info as SwitchReason (removed)
   SHORT                      PageFaultCount; // Number of pagefaults outstanding (possible negative numbers)
   SHORT                      EngineSubmitQueueCount;
} UK_EXECLIST_CONTEXT;


//*****************************************************************************
// Struct: UK_KM_ADDRESS
// PURPOSE: Used to hold ptr to CPU virtual address and Ptr to Ukernel address
//*****************************************************************************
typedef struct UK_KM_ADDRESS_REC
{
    ADDRESS64  pCpuAddress; //Cpu / big core address (virtual)
    ADDRESS32  pUkAddress;  //uKernel address (gfx)
}UK_KM_ADDRESS;


//*****************************************************************************
// Struct:  UK_KM_CONTEXT_DESCRIPTOR
// PURPOSE: Context descriptor for communicating between uKernel and KM Driver
//*****************************************************************************
typedef struct UK_KM_CONTEXT_DESCRIPTOR_REC
{
   UK_KM_COMMON_CONTEXT_AREA_DESCRIPTOR UkSchedCommonArea;
   ULONG                ContextID;
   ULONG                PASID;
   IGFX_ENGINES_USED    EnginesUsed;
   UK_KM_ADDRESS        DoorbellTriggerAddress;   // The doorbell page's trigger cacheline (Ptr to UK_KM_APP_DOORBELL_INFO)
   ADDRESS64            DoorbellTriggerAddressGPA;
   USHORT               DoorbellID;               // Do not modify manually once allocated

   UK_EXECLIST_CONTEXT  ExecListContext[IGFX_ABSOLUTE_MAX_ENGINES];

   union
   {
    struct
    {
     UCHAR      IsContextActive         : KM_BIT      (    0); // Is this context actively assigned to an app
     UCHAR      IsPendingDoorbell       : KM_BIT      (    1);
     UCHAR      IsKMDCreatedContext     : KM_BIT      (    2); // KMD Special WorkItems can be processed.
     UCHAR      IsKMDPreemptContext     : KM_BIT      (    3); // KMD Context used for preemption. This flag is used for tracking if preemption workload is complete.
     UCHAR      IsContextEngReset       : KM_BIT      (    4); // Context was part of engine reset. KMD must take appropriate action (this context will not be resubmitted until this bit is cleared)
     UCHAR      WqProcessingLocked      : KM_BIT      (    5); // Set it = 1 to prevent other code paths to do work queue processing as we use sampled values for WQ processing. Allowing multiple code paths to do WQ processing will cause same workload to execute multiple times.
     UCHAR      IsDoorbellForPCHUse     : KM_BIT      (    6); // If set to 1 at acquire doorbell time, this doorbell address will be programmed in appropriate XTM register so that PCH can ring the doorbell
     UCHAR                              : KM_BIT_RANGE(7,  7);
    };
    UCHAR      BoolValues;
   };

   UK_CONTEXT_PRIORITY  Priority;

   ULONG                WQSampledTailOffset;       // Sampled and set during doorbell ISR handler
   ULONG                TotalSubmitQueueEnqueues;  // Global (across all submit queues) pending enqueues.

   ADDRESS32            pProcessDescriptor;        // pProcessDescriptor is ptr to (UK_SCHED_PROCESS_DESCRIPTOR *)
   ADDRESS32            pWorkQueueAddress;
   ULONG                WorkQueueSizeBytes;        // WQ address & size is here because we do not trust
                                                   // addresses in UK_SCHED_PROCESS_DESCRIPTOR

   //With the new Submit Queue Implementation also we do not do duplicate entries in the submit queue
   //for a context if it is already present in the submit queue.
   //The flag below are set if a context is present in the submit queue of the engine
   SCHED_CONTEXT_ENGINE_PRESENCE EnginePresence;   // Note that a duplicate copy is in UK_SCHED_PROCESS_DESCRIPTOR is used for letting app know the status.
                                                   // It is duplicated since app is not trusted that it will not write to that location and
                                                   // Enginepresence is used in scheduling decisions.

   ULONG                Reserved0[1];              // For future use.
   ULONG64              Reserved1[1];              // For GuC Internal use. MBZ at init time.

}UK_KM_CONTEXT_DESCRIPTOR, ContextDescriptor;      //* ContextDescriptor this is for Fulsim */

//*****************************************************************************
// Struct:  KM_GUC_CONTEXT_INFO
// PURPOSE: Information about the actual contexts stored in the pool
//*****************************************************************************
typedef struct KM_GUC_POOLED_CONTEXT_INFO_REC
{

   UK_KM_CONTEXT_DESCRIPTOR  Context;               // UK_KM_CONTEXT_DESCRIPTORs
   ULONG64                   AssignedGuCGPUDesc;    // CPU back pointer to the GPU descriptor (KM_GUC_GPU_DESC) to which this context is assigned to.
}KM_GUC_POOLED_CONTEXT_INFO;

#pragma pack(pop)

/* Cleaned up interfaces below */
struct i915_itouch_opengpu {
	u8 engines_used;
	u32 priority_requested;
	u32 flags;
	u32 checkcode;
	uint64_t process_desc;
	u32 process_desc_size_bytes;
	u32 priority_assigned;
	uint64_t gpu_handle;
	u32 guc_status;

	/*TBD: Are these really required?? */
	struct page *db_page;
	uint64_t db_phy_addr;
	uint64_t tail_phy_addr;
};

struct i915_itouch_closegpu {
	uint64_t process_desc;
	uint64_t gpu_handle;
	u32 checkcode;
	u32 guc_status;
};

struct i915_itouch_mapbuffer {
	uint64_t	contextid;
	u32		size;
	uint64_t	buf_handle;
	void		*gfx_addr;
	void		*cpu_addr;
	u32		flags;
	uint64_t	cpu_phy_addr;
};

struct i915_itouch_ops {

	int (*open_gpu)(uint64_t gfx_handle,
			struct i915_itouch_opengpu *opengpu);

	int (*close_gpu)(uint64_t gfx_handle,
			struct i915_itouch_closegpu *closegpu);

	int (*map_buffer)(uint64_t gfx_handle,
			struct i915_itouch_mapbuffer *mapbuffer);

	int (*unmap_buffer)(uint64_t gfx_handle, uint64_t buf_handle);

	int (*signal_unload)(uint64_t gfx_handle);

	int (*submit_kernel)(uint64_t gfx_handle, uint64_t batch_buf_handle,
			u32 size, u32 id);
};

struct hid_itouch_ops {
	void (*signal_complete)(uint64_t hid_handle, u32 id);
	void (*display_on_off)(uint64_t hid_handle, bool enable);
};


struct i915_itouch_register_args {

	/* Input: Version of header used by HID */
	u32 hid_version;

	/* Output: Version of header used by GFX */
	u32 gfx_version;

	/* Input: Passed from Hid to Gfx */
	struct hid_itouch_ops hid_ops;

	/* Input: handle to HID context */
	uint64_t hid_handle;

	/* Output: Passed back from Gfx to Hid */
	struct i915_itouch_ops i915_ops;

	/* Output: handle to graphics context */
	uint64_t gfx_handle;
};

extern int i915_itouch_client_register(struct i915_itouch_register_args *args);
extern void i915_print_itouch_guc_info(struct guc_process_desc *p_desc);
extern void i915_itouch_reacquire_db(uint64_t gfx_handle);
#endif //_ITOUCH_GFX_INTERFACE_H_
