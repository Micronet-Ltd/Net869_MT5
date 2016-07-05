/*
 * FlexCanDevice.h
 *
 *  Created on: Nov 17, 2015
 *      Author: ruslans
 */

#ifndef SOURCES_FLEXCAN_FLEXCANDEVICE_H_
#define SOURCES_FLEXCAN_FLEXCANDEVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*Definition*/

#define MAX_MB_NUMBER           		(16)

#define RX_FLEXCAN_MSGQ_MESAGES         (128)
#define RX_FLEXCAN_MSGQ_TRESHOLD_MAX    (RX_FLEXCAN_MSGQ_MESAGES-5)
#define RX_FLEXCAN_MSGQ_TRESHOLD_MIN    (RX_FLEXCAN_MSGQ_MESAGES>>2)

#define FLEXCAN_DEVICE_TX_TIMEOUT       (200)

#define FLEXCAN_MAX_MSG_STR_SIZE        (32)

typedef enum _flexcan_device_status
{
	fcStatus_FLEXCAN_Error = -1,
	fcStatus_FLEXCAN_Success = 0,
	fcStatus_FLEXCAN_OutOfRange,
	fcStatus_FLEXCAN_UnknownProperty,
	fcStatus_FLEXCAN_InvalidArgument,
	fcStatus_FLEXCAN_Fail,
	fcStatus_FLEXCAN_TimeOut,
	fcStatus_FLEXCAN_TxBusy,
	fcStatus_FLEXCAN_RxBusy,
	fcStatus_FLEXCAN_NoTransmitInProgress,
	fcStatus_FLEXCAN_NoReceiveInProgress
} flexcan_device_status_t;

typedef enum _flexcan_device_bitrate
{
	fdBitrate_Error = -1,
	fdBitrate_10_kHz = 0,
	fdBitrate_20_kHz,
	fdBitrate_33_kHz,
	fdBitrate_50_kHz,
	fdBitrate_100_kHz,
	fdBitrate_125_kHz,
	fdBitrate_250_kHz,
	fdBitrate_500_kHz,
	fdBitrate_750_kHz,
	fdBitrate_1_mHz,
	fdBitrate_MAX
}flexcan_device_bitrate_t;

typedef struct _flexcan_device_msgRX {
	union {
		struct {
			uint32_t canInstance:16;
			uint32_t mb_id:16;
		};
		uint32_t    mbID_data;
	}mb_inst_id;
	uint32_t    msgID;
	uint8_t     data[kFlexCanMessageSize];
}flexcan_device_msgRX_t, *pflexcan_device_msgRX_t;

typedef struct flexcanInstance {
	uint8_t                         instance;                   // Instance of CAN0 or CAN1
	flexcan_state_t                 canState;                   // Internal driver state information.
	flexcan_user_config_t           flexcanData;                // FlexCan configuration
	bool                            initialize;                 // Indicate that configuration is set
	flexcan_device_bitrate_t        instanceBitrate;            // Instance Bitrate
	bool                        	bScanInstanceStarted;       // Indicate that Instance is started
	uint32_t                        canPeClk;                   // HW clock value
	MUTEX_STRUCT                    mutex_MB_sync;              // sync MB configuration
	bool                            bIsInstanceStop;            // Indicate Stop instance process
	QUEUE_STRUCT                    Rx_FreeMSGQueue;            // Queue free messages for recieve
	QUEUE_STRUCT                    Rx_ReadyMSGQueue;           // Queue of ready messsages
	void*                           pMesagebuff;                // Mssages allocated pool pointer
	flexcan_id_table_t*             pFIFOIdFilterTable;         // FIFO ID filter table
	uint32_t                        FIFOFilterTableSize;        // FIFO ID filter table size
	uint32_t                        FIFOTableIndx;              // FIFO ID filter table next index
	flexcan_id_table_t              FIFOAceptableMask;          // FIFO Aceptable mask
}flexcanInstance_t, *pflexcanInstance_t;

typedef enum _flexcaninstance_operation_modes {
	fdFlexCanNormalMode,        // Normal mode or user mode
	fdFlexCanListenOnlyMode,    // Listen-only mode
	fdFlexCanLoopBackMode,      // Loop-back mode
	fdFlexCanFreezeMode,        // Freeze mode
	fdFlexCanDisableMode,       // Module disable mode
	fdFlexCanMode_MAX
} flexcaninstance_operation_modes_t, *pflexcaninstance_operation_modes_t;

typedef struct _flexcandevice_initparams {
	uint32_t                            max_num_mb;         // The maximum number of Message Buffers
	flexcan_rx_fifo_id_filter_num_t     num_id_filters;     // The number of RX FIFO ID filters needed
	bool                                is_rx_fifo_needed;  // 1 if needed; 0 if not. This controls whether the Rx FIFO feature is enabled or not
	flexcaninstance_operation_modes_t   flexcanMode;        // Can instance operation mode
	flexcan_device_bitrate_t            instanceBitrate;    // Instance Bitrate
	uint32_t                            RX_queue_num;       // The recieve queue elements
	uint32_t                            TX_queue_num;       // The transmit queue elements
	flexcan_rx_fifo_id_element_format_t fifoElemFormat;     // Format ID for FIFO filtering
}flexcandevice_initparams_t, *pflexcandevice_initparams_t;

typedef enum _flexcandevice_module {
	fdcandevice_CAN0 = BSP_CAN_DEVICE_0,
	fdcandevice_CAN1 = BSP_CAN_DEVICE_1,
	fdcandevice_CAN1_MAX
}flexcandevice_module_t;

typedef struct _flexcandevice_TX_data {
	uint32_t                        msgID;
	flexcan_msgbuff_id_type_t       msgbuffType;
	uint32_t                        msgSize;
	uint8_t                         msgData[8];
}flexcandevice_TX_data_t, *pflexcandevice_TX_data_t;

/* Tasks */

extern void FLEXCAN_Tx_Task( uint32_t param );
extern void FLEXCAN_Rx_Task( uint32_t param );

/* Functions */
extern void     print_result( uint32_t );
extern void     get_string( char *, uint32_t * );

/* Global variables */

extern pflexcanInstance_t can_Device_0;
extern pflexcanInstance_t can_Device_1;

/* init pin outs and configuration the perepherials */
extern void FlexCanDevice_InitHW ( void );

/* Get CAN instance */
extern pflexcanInstance_t FlexCanDevice_GetInstance( flexcandevice_module_t );

/* The initalized driver and HW */
extern flexcan_device_status_t FlexCanDevice_Init( pflexcandevice_initparams_t pinstance_Can0, pflexcandevice_initparams_t pinstance_Can1 );

/* Stop HW and TASKS for coresponded CAN instance and flush all mailboxes */
extern flexcan_device_status_t FlexCanDevice_DeInit( pflexcanInstance_t pInstance );

/* Start HW and TASKS for coresponded CAN instance */
extern flexcan_device_status_t FlexCanDevice_Start( pflexcanInstance_t pInstance );

/* STOP HW and TASKS for coresponded CAN instance */
extern flexcan_device_status_t FlexCanDevice_Stop( pflexcanInstance_t pInstance );

/* Set the Baudrate */
extern flexcan_device_status_t FlexCanDevice_SetBitrate( pflexcanInstance_t pinstance, flexcan_device_bitrate_t bitRate );

/* Get the configured Baudrate */
extern flexcan_device_status_t FlexCanDevice_GetBitrate( pflexcanInstance_t pinstance, flexcan_device_bitrate_t *pbitRate );

/* Register-Unregister the mailbox  */
extern flexcan_device_status_t FlexCanDevice_setMailbox( pflexcanInstance_t pinstance, flexcan_msgbuff_id_type_t id_type, uint32_t id, uint32_t mask, bool enabled );

/* Transfer message over */
extern flexcan_device_status_t FlexCanDevice_TxMessage ( pflexcanInstance_t pinstance, uint32_t MbId, pflexcandevice_TX_data_t pTxData );

/* Enable/Disable termination*/
extern flexcan_device_status_t FlexCanDevice_SetTermination( pflexcanInstance_t, bool isSet );

/* Set CAN instance operation mode*/
extern flexcan_device_status_t FlexCanDevice_SetOperationMode( pflexcanInstance_t pinstance, flexcaninstance_operation_modes_t mode );

/* Set individual or global RX masking type*/
extern flexcan_device_status_t FlexCanDevice_SetRxMaskType( pflexcanInstance_t pinstance, bool isGlobal );

/* Sets the FlexCAN RX FIFO global standard or extended mask */
extern flexcan_device_status_t FlexCanDevice_SetRxFifoGlobalMask( pflexcanInstance_t pinstance, flexcan_msgbuff_id_type_t id_type, uint32_t mask );

/* Sets the FlexCAN RX MB global standard or extended mask */
extern flexcan_device_status_t FlexCanDevice_SetRxMbGlobalMask( pflexcanInstance_t pinstance, flexcan_msgbuff_id_type_t id_type, uint32_t mask );

/* Sets the FlexCAN RX individual standard or extended mask */
extern flexcan_device_status_t FlexCanDevice_SetRxIndividualMask( pflexcanInstance_t pinstance, flexcan_msgbuff_id_type_t id_type, uint32_t mb_idx, uint32_t mask );


#ifdef __cplusplus
}
#endif


#endif /* SOURCES_FLEXCAN_FLEXCANDEVICE_H_ */
