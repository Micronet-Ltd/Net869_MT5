/*
 * FlexCanDevice.c
 *
 *  Created on: Nov 17, 2015
 *      Author: ruslans
 */

#include <string.h>
#include <math.h>
#include <stdio.h>

#include <mqx.h>
#include <bsp.h>
#include <fsl_flexcan_driver.h>
#include <fsl_flexcan_hal.h>
#include <lwmsgq.h>
#include <mutex.h>
#include <mqx_str.h>
#include <queue.h>

#include "board.h"
#include "gpio_pins.h"
//#include "fsl_debug_console.h"
#include "Uart_debugTerminal.h"
#include "tasks_list.h"
#include "FlexCanDevice.h"
#include "mic_typedef.h"
#include "FlexCanMsg_queue.h"

/* Definition */

#define CAN_OK_RESPONCE 	0x0D
#define CAN_ERROR_RESPONCE	0x07

//#define FLEXCAN_DEVICE_DEBUG_

/* The following tables are the CAN bit timing parameters that are calculated by using the method
 * outlined in AN1798, section 4.1.
 */
/*
 * The table contains propseg, pseg1, pseg2, pre_divider, and rjw. The values are calculated for
 * a protocol engine clock of 60MHz
 */
flexcan_time_segment_t bitRateTable60Mhz[] = {
	{ 6, 7, 7, 19, 3 },  /* 125 kHz */
	{ 6, 7, 7,  9, 3 },  /* 250 kHz */
	{ 6, 7, 7,  4, 3 },  /* 500 kHz */
	{ 6, 5, 5,  3, 3 },  /* 750 kHz */
	{ 6, 5, 5,  2, 3 },  /* 1   MHz */
};

/*
 * The table contains propseg, pseg1, pseg2, pre_divider, and rjw. The values are calculated for
 * a protocol engine clock of 48MHz
 */
flexcan_time_segment_t bitRateTable48Mhz[] = {
	{ 7, 7, 2, 0xEF, 3}, /* 10 kHz */
	{ 4, 7, 1, 0x95, 3}, /* 20 KHz */
	{ 2, 3, 3, 0x77, 2}, /* 33.33 KHz */
	{ 4, 7, 1, 0x3B, 3}, /* 50 KHz */
	{ 4, 7, 1, 0x1D, 3}, /* 100 KHz */
	{ 6, 7, 7, 15, 3 },  /* 125 kHz */
	{ 6, 7, 7,  7, 3 },  /* 250 kHz */
	{ 2, 3, 3,  7, 1 },  /* 500 kHz */
	//{ 6, 7, 7,  3, 3 },  /* 500 kHz */
	{ 6, 3, 3,  3, 3 },  /* 750 kHz */
	{ 6, 3, 3,  2, 3 },  /* 1   MHz */
};

/*
 * The table contains propseg, pseg1, pseg2, pre_divider, and rjw. The values are calculated for
 * a protocol engine clock of 75MHz
 */
flexcan_time_segment_t bitRateTable75Mhz[] = {
	{ 6, 7, 7, 25, 3 },  /* 125 kHz */
	{ 6, 7, 7, 12, 3 },  /* 250 kHz */
	{ 6, 6, 6,  6, 3 },  /* 500 kHz */
	{ 6, 4, 4,  5, 3 },  /* 750 kHz */
	{ 6, 3, 3,  4, 3 },  /* 1   MHz */
};

typedef enum _get_message_type_enum
{
	GET_MSG_MASK = 0,
	GET_MSG_FILTER_CODE,
	GET_MSG_AUTO_FLOW,
} get_message_type_enum;


typedef struct get_message_s
{
	get_message_type_enum message_type;
	uint8_t index;
}get_message_t;

flexcanInstance_t g_flexcanDeviceInstance[BOARD_CAN_INSTANCE];
pflexcanInstance_t can_Device_0 = &g_flexcanDeviceInstance[BSP_CAN_DEVICE_0];
pflexcanInstance_t can_Device_1 = &g_flexcanDeviceInstance[BSP_CAN_DEVICE_1];

uint32_t g_flexacandeviceSeted = 0;
//flowcontrol_t pinst->g_flowcontrol;

extern FLEXCAN_Debug_t g_Flexdebug;

flexcan_device_bitrate_t    ConvertTimetoBitRate( flexcan_time_segment_t *ptimeSegmentTable,  flexcan_time_segment_t *ptimeSegment );
flexcan_device_status_t     FlexCanDevice_InitInstance( uint8_t instNum, pflexcandevice_initparams_t pinstance_Can );
flexcan_device_status_t		DecodeSendTxMessage ( const char* buff, uint32_t bufflen, pflexcandevice_TX_data_t pTxData, flexcan_msgbuff_id_type_t msg_type, bool is_remote_frame );
flexcan_device_status_t		DecodeFlowCmd ( const char* buff, uint32_t bufflen, p_flowcontrol_t p_flowCmdTable,  uint8_t* bytes_read );
flexcan_device_status_t 	parseHex(int8_t * line, uint8_t len, uint8_t * value);
bool                        parseAsciToUInt (const int8_t * line, uint8_t len, uint32_t *val);
bool 						parseAsciToShort (const uint8_t* pbuff, uint16_t* val);
bool                        AllocateFIFOFilterTable (pflexcanInstance_t pinst,flexcan_rx_fifo_id_filter_num_t filtnum, flexcan_rx_fifo_id_element_format_t tableFormat);
bool 						AllocateFIFOMaskTable (pflexcanInstance_t pinst);

int32_t                     ParseCanMessToString (pFLEXCAN_queue_element_t pCanMess, const uint8_t *DestBuff, flowcontrol_t * flowcontrol);
bool						CheckCommandIdSupp ( const uint8_t* buff, uint16_t* IdVal);
int8_t 						get_msg_response(pflexcanInstance_t pinst, get_message_t * msg_req, char * resp, uint8_t resp_max_size);

bool flow_control_init(pflexcanInstance_t pinst)
{
	uint8_t idx;
	bool ret;
	
	if (!pinst) {
		return false;
	}

	if (0 != pinst->flowcontrol.p_response[0]) {
		ret = _mem_free ((void*)pinst->flowcontrol.p_response[0]);
		if (MQX_OK != ret) {
			printf("%s:ERROR(%d) free flow control response table\n", __func__, ret);
			return false;
		}
	}
	
	pinst->flowcontrol.p_response[0] = _mem_alloc_zero(FLOW_CONTROL_ARR_SIZE * FLEXCAN_FLOW_CTR_COMMAND_MAX_SIZE);
	if (NULL == pinst->flowcontrol.p_response[0] ){
        return false;
    }
	/*fill the msg_ids with a default ID */
	for (idx = 0; idx < FLOW_CONTROL_ARR_SIZE; idx++){
		pinst->flowcontrol.msg_id[idx] = FLOW_CONTROL_INVALID_ID;
        pinst->flowcontrol.resp_size[idx] = 0;
        pinst->flowcontrol.bisExtended[idx] = false;
		pinst->flowcontrol.p_response[idx] = pinst->flowcontrol.p_response[0] + (FLEXCAN_FLOW_CTR_COMMAND_MAX_SIZE*idx);
	}
	pinst->flowcontrol.match_position = FLOW_CONTROL_INVALID_POS;
	pinst->flowcontrol.idx = 0; 
	return true;
}
	

void FlexCanDevice_InitHW ( )
{
	uint8_t idx;
	//Unset CAN termination
	GPIO_DRV_SetPinOutput(CAN1_TERM_ENABLE);
	GPIO_DRV_SetPinOutput(CAN2_TERM_ENABLE);

	//Set CAN2 to regular mode twisted
	GPIO_DRV_ClearPinOutput(CAN2_SWC_SELECT);

	//Sleep mode for NCV7356
	GPIO_DRV_ClearPinOutput(SWC_MODE0);
	GPIO_DRV_ClearPinOutput(SWC_MODE1);

	//Disable SWC
	GPIO_DRV_SetPinOutput(SWC_ENABLE);

	//Configure CAN1 Pins
	configure_can_pins(0);
	configure_can_pins(1);

	_mem_zero((void *)&g_flexcanDeviceInstance[BSP_CAN_DEVICE_0], sizeof(flexcanInstance_t));
	_mem_zero((void *)&g_flexcanDeviceInstance[BSP_CAN_DEVICE_1], sizeof(flexcanInstance_t));

	g_flexcanDeviceInstance[BSP_CAN_DEVICE_0].instance = BSP_CAN_DEVICE_0;
	g_flexcanDeviceInstance[BSP_CAN_DEVICE_1].instance = BSP_CAN_DEVICE_1;
}

void FLEXCAN_Tx_Task( uint32_t param_in ) {
	uint32_t                    result;
	uint16_t					msg_ID;
	_queue_id               	msg_qid;
	APPLICATION_MESSAGE_PTR_T   msg_ptr;
	flexcan_device_status_t     ret;
	flexcandevice_TX_data_t     Tx_data;
	uint8_t                     termination, baudrate, Baudrate_notSet = 1;
	char*                       pbuff;
	int32_t                     msg_size = 0;
	char                        erroResp;
	pcdc_mic_queue_element_t    pqMemElem;
	bool 						is_remote_frame = false;
	flexcan_msgbuff_id_type_t 	msg_type = kFlexCanMsgIdStd;

	bool 						flowcontrol_msg, flowcontrol_msg_extended;

	flexcandevice_initparams_t  initCan;
	pflexcanInstance_t pcan = (pflexcanInstance_t)param_in;

	get_message_t msg_req;
	char get_resp[50]= {0};
	uint32_t resp_msg_size = 0;
	uint8_t bytes_read = 0;

	if ( NULL == (pcan) || BOARD_CAN_INSTANCE <= (pcan)->instance ) {
		printf( "%s:thread wrong param %u\n", __func__, (pcan)->instance );
		return;
	}

	initCan.flexcanMode         = fdFlexCanDisableMode;
	initCan.instanceBitrate     = fdBitrate_500_kHz;
	initCan.is_rx_fifo_needed   = true;
	initCan.max_num_mb          = MAX_MB_NUMBER;
	initCan.num_id_filters      = kFlexCanRxFifoIDFilters_24;
	initCan.fifoElemFormat      = kFlexCanRxFifoIdElementFormatA;

	pcan->pMesagebuff = _mem_alloc(RX_FLEXCAN_MSGQ_MESAGES * sizeof(FLEXCAN_queue_element_t));
	if (NULL == (pcan)->pMesagebuff) {
		printf( "%s:ERROR allocating  message queue buffer\n", __func__);
		_task_block();
	}

	if (!AllocateFIFOFilterTable(pcan, initCan.num_id_filters, initCan.fifoElemFormat)) {
		printf( "%s:ERROR allocating FIFO ID filter table\n", __func__);
		_task_block();
	}

	if (!AllocateFIFOMaskTable(pcan)){
		printf( "%s:ERROR allocating FIFO ID mask table\n", __func__);
		_task_block();
	}

	if (!flow_control_init(pcan) ){
        printf("%s:ERROR allocate flow control messages buffer\n", __func__);
        _task_block();
    }

	FlexCanMsg_queue_init(&(pcan->Rx_FreeMSGQueue), 0);
	FlexCanMsg_queue_init(&(pcan->Rx_ReadyMSGQueue), 0);

	if ( BSP_CAN_DEVICE_0 == pcan->instance ) {
		msg_qid = _msgq_open(CAN1_TX_QUEUE, 0);
	}
	else {
		msg_qid = _msgq_open(CAN2_TX_QUEUE, 0);
	}

	if (MSGQ_NULL_QUEUE_ID == msg_qid)
	{
	   printf("\n%s:ERROR Could not create a message pool CAN %u _TX_QUEU\n", __func__, pcan->instance);
	   _task_block();
	}

//	if (MQX_OK != _lwevent_create(&(((pflexcanInstance_t)param)->canState.event_ISR), LWEVENT_AUTO_CLEAR)) // Not set auto clean bits
//    {
//        printf("Make event failed\n");
//        _task_block();// ( kStatus_FLEXCAN_Fail );
//    }

	printf("%s:INFO Task: Loop instance %u \n", __func__, pcan->instance);

	do {
		msg_ptr = _msgq_receive(msg_qid, 1);
		if(NULL == msg_ptr) { _time_delay(1); continue; }

		pbuff = (char *)msg_ptr->data;
		msg_size = msg_ptr->header.SIZE - APP_MESSAGE_NO_ARRAY_SIZE;
		erroResp = CAN_OK_RESPONCE;
		pqMemElem = NULL;
		//printf("Cmd rec %c=%d, s %d, Inst %d\n", (char)pbuff[0], (char)pbuff[0], msg_size, ((pflexcanInstance_t)param)->instance );
		do {
			msg_ID = 0;
			switch ( *pbuff ) {
			case 'S':
				if ( msg_size > 2 ) {
					if (true == pcan->bScanInstanceStarted && fdFlexCanListenOnlyMode != initCan.flexcanMode) {
						printf("%s:ERROR, case S, Tried to change baudrate when CAN is open\n", __func__);
						erroResp = CAN_ERROR_RESPONCE;
						break;
					}
					pbuff++;
					msg_size--;

					if (fcStatus_FLEXCAN_Success != parseHex((int8_t *)pbuff, 1, &baudrate)) {
						printf("%s:ERROR, case S, parsing the Baudrate value %x %x\n", __func__, *pbuff, baudrate );
						Baudrate_notSet = 1;
						erroResp = CAN_ERROR_RESPONCE;
						break;
					}
					pbuff++;
					msg_size--;

					if (CheckCommandIdSupp ( (const uint8_t*)pbuff, &msg_ID )) {
						pbuff += 5;
						msg_size -= 5;
					}

					if ( (fdBitrate_10_kHz <= baudrate) && ( fdBitrate_MAX > baudrate ) ) {
						if ( BSP_CAN_DEVICE_0 == pcan->instance && fdBitrate_33_kHz == baudrate )
						{
							printf("%s:ERROR, case S, set baudrate 33Khz to instance %d\n", __func__, pcan->instance);
							erroResp = CAN_ERROR_RESPONCE;
							break;
						}
						else {
							initCan.instanceBitrate = ( flexcan_device_bitrate_t )baudrate;
							printf("%s:INFO, case S, Set baudrate command %x \n", __func__, baudrate );
							Baudrate_notSet = 0;
						}
					}
					else {
						printf("%s:ERROR, case S, incorrect baudrate value %c \n", __func__, baudrate );
						erroResp = CAN_ERROR_RESPONCE;
						break;
					}
					if (CAN_OK_RESPONCE != *pbuff) {
						printf("%s:ERROR, case S, Baud rate command, \r not found\n", __func__);
						Baudrate_notSet = 1;
						erroResp = CAN_ERROR_RESPONCE;
						break;
					}
					pbuff++;
					msg_size--;
				}
				break;
			case 'L':
				initCan.flexcanMode = fdFlexCanListenOnlyMode;
			case 'O':
				pbuff++;
				msg_size--;

				if (1 == Baudrate_notSet) {
					printf("%s:ERROR, case O, The baudrate not configured Open command terminated\n", __func__);
					erroResp = CAN_ERROR_RESPONCE;
					break;
				}

				if ( fcStatus_FLEXCAN_Success != parseHex((int8_t*)pbuff, 1, &termination)) {
					printf("%s:ERROR, case O, parse the Termination value %x %x\n", __func__, *pbuff, termination );
					erroResp = CAN_ERROR_RESPONCE;
					break;
				}
				pbuff++;
				msg_size--;

				if (CheckCommandIdSupp ( (const uint8_t*)pbuff, &msg_ID )) {
						pbuff += 5;
						msg_size -= 5;
				}

				if (CAN_OK_RESPONCE != *pbuff) {
					printf("%s:ERROR, case O, Open command, \r not found\n", __func__);
					erroResp = CAN_ERROR_RESPONCE;
					break;
				}
				pbuff++;
				msg_size--;

				if ( true == pcan->bScanInstanceStarted ) {
					printf("%s:INFO, case O, CAN%d already opened\n", __func__, pcan->instance);
					break;
				}

				if ( BSP_CAN_DEVICE_0 == pcan->instance ) {
					ret = FlexCanDevice_Init(&initCan, NULL);
				}
				else {
					ret = FlexCanDevice_Init(NULL, &initCan);
				}
				printf("%s:INFO, case O, FlexCanDevice_Init( ) return %d\n", __func__, ret);

				//((pflexcanInstance_t)param)->canState.pevent_ISR = &event_ISR;
				pcan->canState.fifo_free_messages = &(pcan->Rx_FreeMSGQueue);
				pcan->canState.fifo_ready_messages = &(pcan->Rx_ReadyMSGQueue);

				if ( 0 == termination ){
					ret = FlexCanDevice_SetTermination(pcan, false);
				}
				else{
					ret = FlexCanDevice_SetTermination(pcan, true);
				}
				printf("%s:INFO, case O, FlexCanDevice_SetTermination( ) return %d on set %d\n", __func__, ret, termination);

				ret = FlexCanDevice_Start(pcan);
				printf("%s:INFO, case O, FlexCanDevice_Start( ) return %d\n", __func__, ret);

				pcan->bScanInstanceStarted = true;

				if (initCan.is_rx_fifo_needed)
				{
					// TODO: Setting the global mask might not be necessary
					ret = FlexCanDevice_SetRxMaskType(pcan, true);
					printf("%s:INFO: case O, FlexCanDevice_SetRxMaskType( ) to global return %d\n", __func__, ret);
					FlexCanDevice_SetRxFifoGlobalMask(pcan, kFlexCanMsgIdExt, 0x7fffffff);

					ret = FlexCanDevice_SetRxMaskType(pcan, false);
					for (uint32_t i = 0; i < pcan->FIFOMaskTableSize; i++){

						/* set default values if the mask is not set */
						if (i >= pcan->FIFOMaskTableIndx){
							if (i%2){
								(pcan->pFIFOIdMaskTable + i)->MaskId = 0x7ff;
								(pcan->pFIFOIdMaskTable + i)->isExtendedFrame = false;
								(pcan->pFIFOIdMaskTable + i)->isRemoteFrame = false;
							}
							else{
								(pcan->pFIFOIdMaskTable + i)->MaskId = 0x1fffffff;
								(pcan->pFIFOIdMaskTable + i)->isExtendedFrame = true;
								(pcan->pFIFOIdMaskTable + i)->isRemoteFrame = false;
							}
						}

						ret = FlexCanDevice_SetRxIndividualMask (pcan,\
								(flexcan_msgbuff_id_type_t)(pcan->pFIFOIdMaskTable + i)->isExtendedFrame,\
								i,\
								(pcan->pFIFOIdMaskTable + i)->MaskId);
					}

					/* print all the masks and filter */
					printf("\n%s:INFO, case O, FIFO table[ii]: Mask     : ext : filter : ext \n", __func__);
					for (uint32_t i = 0; i < 24; i++){
						printf("%s:INFO, case O, FIFO table[%02d]: %08x : %s : %08x : %s \n", __func__, i,
								(i < pcan->FIFOMaskTableSize) ? ((pcan->pFIFOIdMaskTable + i)->MaskId) : 0x1fffffff ,
								(pcan->pFIFOIdMaskTable + i)->isExtendedFrame ? "ext": "std",
								(pcan->pFIFOIdFilterTable + i)->idFilter,
								(pcan->pFIFOIdFilterTable + i)->isExtendedFrame ? "ext": "std");
					}

					/* print all the flowcontrol responses*/
					printf("\n\n%s:INFO, case O, Flowcontrol table[i]: ext : ID       : response \n", __func__);
					for (uint32_t i = 0; i < FLOW_CONTROL_ARR_SIZE; i++){
						printf("%s:INFO, case O, Flowcontrol table[%01d]: %s : %08x : %s\n", __func__, 
							   i,
							   pcan->flowcontrol.bisExtended[i] ? "ext" : "std",
							   pcan->flowcontrol.msg_id[i],
							   pcan->flowcontrol.p_response[i]);
					}


					ret = (flexcan_device_status_t)FLEXCAN_DRV_ConfigRxFifo(pcan->instance, initCan.fifoElemFormat, pcan->pFIFOIdFilterTable);
					if (ret)
					{
						printf("%s:ERROR, case O, \r\n FLEXCAN_DRV_ConfigRxFifo failed. result: 0x%lx", __func__, result);
					}

					FLEXCAN_DRV_RxFifo(pcan->instance, NULL);
				}
				else {

					ret = FlexCanDevice_SetRxMaskType(pcan, false);
					printf("%s:INFO, case O, FlexCanDevice_SetRxMaskType( ) return %d\n", __func__, ret);
					//ret = FlexCanDevice_SetRxMaskType(((pflexcanInstance_t)param), true);
					//printf("FlexCanDevice_SetRxMaskType( ) return %d\n", ret);

					//ret = FlexCanDevice_SetRxMbGlobalMask ( ((pflexcanInstance_t)param), kFlexCanMsgIdStd , 0x120 );
					//printf("FlexCanDevice_SetRxMbGlobalMask( ) return %d\n", ret);

					for (int i = 0; i < 14; i++) {
						//ret = FlexCanDevice_SetRxIndividualMask ( ((pflexcanInstance_t)param), kFlexCanMsgIdStd, i, 0xF00 );
						ret = FlexCanDevice_SetRxIndividualMask (pcan, kFlexCanMsgIdStd, i, 0x700 ); // Mask Value
						printf("%s:INFO, case O, FlexCanDevice_SetRxIndividualMask %d return %d\n", __func__, i, ret);
						ret = FlexCanDevice_setMailbox(pcan, kFlexCanMsgIdStd, i, 0x700, true); //Filter value
						printf("%s:INFO, case O, FlexCanDevice_setMailbox %d return %d\n", __func__, i, ret);
					}

				}

				//Enable CAN
				if ( BSP_CAN_DEVICE_0 == pcan->instance ){
					GPIO_DRV_SetPinOutput(CAN1_J1708_PWR_ENABLE);
				}
				else {
					if ( fdBitrate_33_kHz == initCan.instanceBitrate) {
						printf("%s:INFO, case O, Set elec fo 33.33 baudrate\n", __func__);
						//Set SWC to operation mode
						//Set CAN2 to regular mode twisted
						GPIO_DRV_SetPinOutput(CAN2_SWC_SELECT);

						//Set Speed to Normal 33KHz of NCV7356
						GPIO_DRV_SetPinOutput(SWC_MODE0);
						GPIO_DRV_SetPinOutput(SWC_MODE1); // Configuration Normal 33KHz Mode Mode0 High Mode1 High
						//GPIO_DRV_ClearPinOutput(SWC_MODE1); // Configuration High up to 100Khz Mode Mode0 High Mode1 Low

						//Enable SWC
						GPIO_DRV_ClearPinOutput(SWC_ENABLE);
						//GPIO_DRV_SetPinOutput(SWC_ENABLE);
					}
					else {
						//Set CAN2 to regular mode twisted
						GPIO_DRV_ClearPinOutput(CAN2_SWC_SELECT);

						//Set to sleep mode NCV7356
						GPIO_DRV_ClearPinOutput(SWC_MODE0);
						GPIO_DRV_ClearPinOutput(SWC_MODE1); // Configuration Sleep

						//Enable SWC
						GPIO_DRV_SetPinOutput(SWC_ENABLE);
					}

					_time_delay (20);

					GPIO_DRV_SetPinOutput(CAN2_SWC_PWR_ENABLE);
				}

				break;
			case 'C':
				pbuff++;
				msg_size--;

				if (CheckCommandIdSupp ( (const uint8_t*)pbuff, &msg_ID )) {
					pbuff += 5;
					msg_size -= 5;
				}

				if (CAN_OK_RESPONCE != *pbuff) {
					printf("%s:ERROR, case C, close command, \r not found\n", __func__);
					erroResp = CAN_ERROR_RESPONCE;
					break;
				}
				pbuff++;
				msg_size--;

				//Disable CAN
				Baudrate_notSet = 1;
				if ( BSP_CAN_DEVICE_0 == pcan->instance ) {
					GPIO_DRV_ClearPinOutput(CAN1_J1708_PWR_ENABLE);
				}
				else {
					GPIO_DRV_ClearPinOutput(CAN2_SWC_PWR_ENABLE);
				}

				ret = FlexCanDevice_SetTermination(pcan, false);
				printf("%s:INFO, case C, FlexCanDevice_SetTermination( ) return %d\n", __func__, ret);

				ret = FlexCanDevice_DeInit(pcan);
				printf("%s:INFO, case C, FlexCanDevice_DeInit( ) return %d\n", __func__, ret);

				pcan->bScanInstanceStarted = false;
				
				if (!AllocateFIFOFilterTable(pcan, initCan.num_id_filters, initCan.fifoElemFormat)) {
					printf( "%s:ERROR, case C, allocating FIFO ID filter table\n", __func__);
					_task_block();
				}

				if (!AllocateFIFOMaskTable(pcan)){
					printf( "%s:ERROR, case C, allocating FIFO ID mask table\n", __func__);
					_task_block();
				}

				if (!flow_control_init(pcan) ){
					printf("%s:ERROR, case C, allocating flow control messages buffer\n", __func__);
					_task_block();
				}

				initCan.flexcanMode         = fdFlexCanNormalMode;
				initCan.instanceBitrate     = fdBitrate_500_kHz;
				initCan.is_rx_fifo_needed   = true;
				initCan.max_num_mb          = MAX_MB_NUMBER;
				initCan.num_id_filters      = kFlexCanRxFifoIDFilters_24;
				initCan.fifoElemFormat      = kFlexCanRxFifoIdElementFormatA;

				_time_delay(100);

				break;
			case 't':
			case 'r':
				msg_type = kFlexCanMsgIdStd;
			case 'T':
			case 'R':
				do {
					if (fdFlexCanListenOnlyMode == initCan.flexcanMode) {
						printf("%s:ERROR, case trTR, CAN in listener mode\n", __func__);
						erroResp = CAN_ERROR_RESPONCE;
						break;
					}
					if ('T' == *pbuff || 'R' == *pbuff) {
						msg_type = kFlexCanMsgIdExt;
					}
					if ('R' == *pbuff || 'r' == *pbuff){
						is_remote_frame = true;	
					}
					else{
						is_remote_frame = false;	
					}
					if ( ((4 > msg_size) && (kFlexCanMsgIdStd == msg_type)) || ((9 > msg_size) && (kFlexCanMsgIdExt == msg_type)) ) {
						printf("%s:ERROR, case trTR, CAN transmit format\n", __func__);
						_msg_free(msg_ptr);
						msg_ptr = NULL;
						msg_size = 0;
						break;
					}
					pbuff++;
					msg_size--;
					if ( fcStatus_FLEXCAN_Success == DecodeSendTxMessage ( (const char*) pbuff, msg_size, &Tx_data , msg_type, is_remote_frame) ) {
						if ( fcStatus_FLEXCAN_Success != FlexCanDevice_TxMessage (pcan, 14, &Tx_data, is_remote_frame) ) {
							printf("%s:ERROR, case trTR, FlexCanDevice_TxMessage failed\n", __func__);
							erroResp = CAN_ERROR_RESPONCE;
						}

						_msg_free(msg_ptr);
						msg_ptr = NULL;
#if 0
						pqMemElem = GetUSBWriteBuffer (pcan->instance + 2);
						if (NULL == pqMemElem) {
							printf("%s:ERROR, case trTR, get mem for USB responce\n", __func__);
							break;
						}

						pbuff = (char *)pqMemElem->data_buff;
						if (CAN_ERROR_RESPONCE != erroResp) {
							if (kFlexCanMsgIdExt == Tx_data.msgbuffType)
								*pbuff ='N';
							else
								*pbuff ='n';
							pbuff++;
						}
						*pbuff = erroResp;
						pqMemElem->send_size = 2;

						if (!SetUSBWriteBuffer(pqMemElem, (pcan->instance + 2)) ) {
							printf("%s:ERROR, case trTR, send data to CDC_%d\n", __func__, (uint32_t)(pcan->instance + 2));
						}
						pqMemElem = NULL;
#endif
					}//if ( fcStatus_FLEXCAN_Success == DecodeSendTxMessage ( (const char*) pbuff, msg_size, &Tx_data ) )
					else {
						printf("%s:ERROR, case trTR, decode message\n", __func__);
						erroResp = CAN_ERROR_RESPONCE;
					}
				} while (0);

				msg_size = 0;
				break;
			case 'm': //FIFO acceptable mask set
				do {
					if ((10 > msg_size) || (true == pcan->bScanInstanceStarted)) {
						printf("%s:ERROR, case m, incorrect msg size, canInst %d\n", __func__, pcan->bScanInstanceStarted);
						erroResp = CAN_ERROR_RESPONCE;
						break;
					}
					pbuff++;
					msg_size--;

					if (NULL == pcan->pFIFOIdMaskTable) {
						printf("%s:ERROR, case m, pFIFOIdMaskTable NULL\n", __func__);
						erroResp = CAN_ERROR_RESPONCE;
						break;
					}

					if (pcan->FIFOMaskTableIndx >= pcan->FIFOMaskTableSize) {
						printf ("%s:INFO, case m, Overwrite FIFO Mask table, FIFOMaskTableIndx=0\n", __func__);
						pcan->FIFOMaskTableIndx = 0;
					}

					_mem_zero ((void*)(pcan->pFIFOIdMaskTable + pcan->FIFOMaskTableIndx), sizeof(flexcan_mask_id_table_t) );

					switch (*pbuff) {
					case 'R':
						(pcan->pFIFOIdMaskTable + pcan->FIFOMaskTableIndx)->isExtendedFrame = true;
					case 'r':
						(pcan->pFIFOIdMaskTable + pcan->FIFOMaskTableIndx)->isRemoteFrame = true;
						break;
					case 'T':
						(pcan->pFIFOIdMaskTable + pcan->FIFOMaskTableIndx)->isExtendedFrame = true;
					case 't':
						/* assumed as not extended since we memzeroed the FIFOIdMaskTable */
						break;
					default:
						printf("%s:ERROR, case m, incorrect msg type\n", __func__);
						erroResp = CAN_ERROR_RESPONCE;
						break;
					}
					pbuff++;
					msg_size--;

					if (!parseAsciToUInt((const int8_t*)pbuff, (msg_size - 1), &(pcan->pFIFOIdMaskTable + pcan->FIFOMaskTableIndx)->MaskId)) {
						printf("%s:ERROR, case m, parsing FIFO ID MASK\n", __func__);
						erroResp = CAN_ERROR_RESPONCE;
						break;
					}
					pbuff += 8;
					msg_size -= 8;

					printf("%s:INFO, case m, Set FIFO Mask[%d] RTR %d IDE %d val %x\n", __func__, pcan->FIFOMaskTableIndx,
												   (pcan->pFIFOIdMaskTable + pcan->FIFOMaskTableIndx)->isRemoteFrame,
												   (pcan->pFIFOIdMaskTable + pcan->FIFOMaskTableIndx)->isExtendedFrame,
												   (pcan->pFIFOIdMaskTable + pcan->FIFOMaskTableIndx)->MaskId);

					if (CheckCommandIdSupp ( (const uint8_t*)pbuff, &msg_ID )) {
						pbuff += 5;
						msg_size -= 5;
					}

					if (CAN_OK_RESPONCE != *pbuff) {
						printf("%s:ERROR, case m, set mask command, \r not found\n", __func__);
						erroResp = CAN_ERROR_RESPONCE;
						_mem_zero ((void*)(pcan->pFIFOIdMaskTable + pcan->FIFOMaskTableIndx), sizeof(flexcan_id_table_t) );
						break;
					}
					pcan->FIFOMaskTableIndx++;
					pbuff++;
					msg_size--;
				} while (0);
				break;
			case 'M':
				do {
					if ((10 > msg_size) || (true == pcan->bScanInstanceStarted)) {
						printf("%s:ERROR, case M, incorrect msg size, canInst %d\n", __func__, pcan->bScanInstanceStarted);
						erroResp = CAN_ERROR_RESPONCE;
						break;
					}
					pbuff++;
					msg_size--;
					/* only init filterTable memory if is not a flow control req */
					if (*pbuff != 'F' && (*pbuff != 'f')){
						if (NULL == pcan->pFIFOIdFilterTable) {
							printf("%s:ERROR, case M, set FIFO ID table NULL\n", __func__);
							erroResp = CAN_ERROR_RESPONCE;
							break;
						}
						
						if (pcan->FIFOTableIndx >= (pcan->FIFOFilterTableSize/sizeof(flexcan_id_table_t))) {
							printf ("%s:INFO, case M, Overwriting FIFO ID Filter table FIFOTableIndx=0\n", __func__);
							pcan->FIFOTableIndx = 0;
						}
						_mem_zero ((void*)(pcan->pFIFOIdFilterTable + pcan->FIFOTableIndx), sizeof(flexcan_id_table_t) );
					}
                    flowcontrol_msg_extended = flowcontrol_msg = FALSE;

					switch (*pbuff) {
					case 'R':
						(pcan->pFIFOIdFilterTable + pcan->FIFOTableIndx)->isExtendedFrame = true;
					case 'r':
						(pcan->pFIFOIdFilterTable + pcan->FIFOTableIndx)->isRemoteFrame = true;
						break;
					case 'T':
						(pcan->pFIFOIdFilterTable + pcan->FIFOTableIndx)->isExtendedFrame = true;
					case 't':
						break;
                    case 'F':
						flowcontrol_msg = TRUE;
                        flowcontrol_msg_extended = TRUE;
						break;
					case 'f':
						flowcontrol_msg = TRUE;
						flowcontrol_msg_extended = FALSE;
						break;
					default:
						printf("%s:ERROR, case M, incorrect msg type\n", __func__);
						erroResp = CAN_ERROR_RESPONCE;
						break;
					}
					pbuff++;
					msg_size--;

                    if (flowcontrol_msg) {
						//WARNING: looping over if all elements are used
						pcan->flowcontrol.idx = (pcan->flowcontrol.idx)%FLOW_CONTROL_ARR_SIZE;
                        if (flowcontrol_msg_extended) {
                            pcan->flowcontrol.bisExtended[pcan->flowcontrol.idx] = TRUE;
                        }
                        bytes_read = 0;
                        if (DecodeFlowCmd ( (char const*)pbuff, (uint32_t) msg_size, &pcan->flowcontrol, &bytes_read )){
							printf("%s:ERROR, case M fF, decoding flow control message \n", __func__);
							erroResp = CAN_ERROR_RESPONCE;
							break;
                        }
						pbuff += bytes_read;
                        msg_size -= bytes_read;

						printf("%s:INFO, case M TtRr, Set flowcontrol table[%d] IDE=%d, msgId=%x response=%s\n", __func__, 
							   pcan->flowcontrol.idx,
							   pcan->flowcontrol.bisExtended[pcan->flowcontrol.idx],
							   pcan->flowcontrol.msg_id[pcan->flowcontrol.idx],
							   pcan->flowcontrol.p_response[pcan->flowcontrol.idx]);
                    }
                    else {

                        if (!parseAsciToUInt((const int8_t*)pbuff, (msg_size - 1), &(pcan->pFIFOIdFilterTable + pcan->FIFOTableIndx)->idFilter)) {
                            printf("%s:ERROR, case M TtRr, parse FIFO ID table value\n", __func__);
                            erroResp = CAN_ERROR_RESPONCE;
                            break;
                        }

                        pbuff += 8;
                        msg_size -= 8;
						
						printf("%s:INFO, case M TtRr, Set FIFO table[%d] RTR=%d, IDE=%d, val=%x\n", __func__, pcan->FIFOTableIndx,
							   (pcan->pFIFOIdFilterTable + pcan->FIFOTableIndx)->isRemoteFrame,
							   (pcan->pFIFOIdFilterTable + pcan->FIFOTableIndx)->isExtendedFrame,
							   (pcan->pFIFOIdFilterTable + pcan->FIFOTableIndx)->idFilter);
						
                    }

					if (CheckCommandIdSupp ( (const uint8_t*)pbuff, &msg_ID )) {
						pbuff += 5;
						msg_size -= 5;
					}

					if (CAN_OK_RESPONCE != *pbuff) {
						if (flowcontrol_msg){
							printf("%s:ERROR, case M fF, set flowcontrol command, \r not found \n", __func__);
							_mem_zero ((void*)pcan->flowcontrol.p_response[pcan->flowcontrol.idx ], FLEXCAN_FLOW_CTR_COMMAND_MAX_SIZE);
						}
						else{
							printf("%s:ERROR, case M TtRr, set FIFO ID filter table, \r not found\n", __func__);
							_mem_zero ((void*)(pcan->pFIFOIdFilterTable + pcan->FIFOTableIndx), sizeof(flexcan_id_table_t) );
						}
						erroResp = CAN_ERROR_RESPONCE;
						break;
					}
						
					if (flowcontrol_msg){
						pcan->flowcontrol.idx++;
					}
					else{
						pcan->FIFOTableIndx++;
					}

					pbuff++;
					msg_size--;
				} while (0);
				break;

			case 'G': /* get info */
				/*
				 *  Possible messages and responses
				 *  Gmii where ii is index of mask list:  0 - 15
				 *  	Response: Gm<t/T/r/R><iiiiiiii>\r
				 *  GMii where ii is index of filter ID list: 0 - 23
				 *   	Response: GM<t/T/r/R><iiiiiiii>\r
				 *  Gfii where ii is index of auto flow list. ii : 0 - 7
				 *  	Response: G<f/F>iiiiiiiiaaaaaaaaLbbbbbbbbbbbbbbbb\r
				 *  				f: standard ID
				 *  				F: Extended ID
				 *  				iiiiiiii: search ID
				 *  				aaaaaaaa: response ID
				 *  				L: response length (0-8)
				 *  				bbbbbbbbbbbbbbbb: Data byte pairs
				 * */
				do {
					if (4 > msg_size) {
						printf("%s: ERROR, case G, incorrect size of get request\n" , __func__);
						erroResp = CAN_ERROR_RESPONCE;
						break;
					}
					pbuff++;
					msg_size--;

					switch (*pbuff) {
					case 'm':
						msg_req.message_type = GET_MSG_MASK;
						break;
					case 'M':
						msg_req.message_type = GET_MSG_FILTER_CODE;
						break;
					case 'f':
						msg_req.message_type = GET_MSG_AUTO_FLOW;
						break;
					default:
						printf("%s:ERROR, case G, get_message Invalid type\n", __func__);
						erroResp = CAN_ERROR_RESPONCE;
						break;
					}
					pbuff++;
					msg_size--;

					/* get ii char */
					if (fcStatus_FLEXCAN_Success != parseHex((int8_t *)pbuff, 2, &(msg_req.index))) {
						printf("%s:ERROR, case G, get_message parsing index\n", __func__);
						erroResp = CAN_ERROR_RESPONCE;
						break;
					}

					pbuff += 2;
					msg_size -= 2;

					if (CheckCommandIdSupp ( (const uint8_t*)pbuff, &msg_ID )) {
						pbuff += 5;
						msg_size -= 5;
					}

					if (CAN_OK_RESPONCE != *pbuff) {
						printf("%s:ERROR, case G, get_message, \r not found\n", __func__);
						erroResp = CAN_ERROR_RESPONCE;
						break;
					}
					pbuff++;
					msg_size--;

					/* send the appropriate response */
					resp_msg_size = get_msg_response(pcan, &msg_req, get_resp, sizeof(get_resp));
					if (resp_msg_size > 0) {
						pqMemElem = GetUSBWriteBuffer(pcan->instance + 2);
						if (NULL == pqMemElem) {
							printf("%s:ERROR, case G, get mem for USB response\n", __func__);
							erroResp = CAN_ERROR_RESPONCE;
							break;
						}
						
						pbuff = (char*)pqMemElem->data_buff;
						memcpy(pbuff, get_resp, resp_msg_size);
						pqMemElem->send_size = resp_msg_size;

						if (!SetUSBWriteBuffer(pqMemElem, (pcan->instance + 2)) ) {
							printf("%s:ERROR, case G, sending get_message resp data to CDC_%d\n", __func__, (uint32_t)(pcan->instance + 2));
							erroResp = CAN_ERROR_RESPONCE;
							break;
						}
						erroResp = CAN_OK_RESPONCE;
						pqMemElem = NULL;
						pbuff = NULL;
					}
					else{
						printf("%s:ERROR, case G, message_type=%d ,index=%d\n",__func__, msg_req.message_type, msg_req.index);
						erroResp = CAN_ERROR_RESPONCE;
						break;
					}
				} while (0);
				
				break;
			case 'F':
				pbuff++;
				msg_size--;

				if (CheckCommandIdSupp ( (const uint8_t*)pbuff, &msg_ID )) {
					pbuff += 5;
					msg_size -= 5;
				}

				if (CAN_OK_RESPONCE != *pbuff) {
					printf("%s:ERROR:F command, \r not found\n", __func__);
					erroResp = CAN_ERROR_RESPONCE;
					break;
				}

				pbuff++;
				msg_size--;
				break;
//  		case CAN_OK_RESPONCE:
//  			pbuff++;
//  			msg_size--;
//  			break;
//  		case CAN_ERROR_RESPONCE:
//  			pbuff++;
//  			msg_size--;
//  			break;
			default:
				printf( "%s:ERROR, case default, Command not recognized %c %d size %d\n", __func__, *pbuff, *pbuff, msg_size  );
				_msg_free(msg_ptr);
				msg_ptr = NULL;
				msg_size = 0;
				erroResp = CAN_ERROR_RESPONCE;
			}

			if ( (0 >= msg_size) || (CAN_OK_RESPONCE != erroResp) ) {
				break;
			}
		}while (0 == g_flag_Exit);

		if ( msg_ptr ) {
#if 1
			_msg_free(msg_ptr);
			msg_ptr = NULL;

			do {
				pqMemElem = GetUSBWriteBuffer(pcan->instance + 2);
				if (NULL == pqMemElem) {
					printf("%s:ERROR, get mem for USB response\n", __func__);
					break;
				}
				pqMemElem->send_size = 0;
				pbuff = (char*)pqMemElem->data_buff;

				if (0 != msg_ID) {
					sprintf ( (char*)(pbuff), "u%04x", msg_ID);
					pqMemElem->send_size = 5;
					pbuff += 5;
				}
				*pbuff = erroResp;
				pqMemElem->send_size += 1;

				if (!SetUSBWriteBuffer(pqMemElem, (pcan->instance + 2)) ) {
					printf("%s:ERROR, send data to CDC_%d\n", __func__, (uint32_t)(pcan->instance + 2));
				}
				pqMemElem = NULL;
				pbuff = NULL;
			} while (0);
#else
			msg_ptr->header.SOURCE_QID = msg_qid;
			msg_ptr->header.TARGET_QID = _msgq_get_id(0, USB_QUEUE);
			//_mem_copy ( &erroResp, msg_ptr->data, 1 );
			msg_ptr->data[0] = (uint8_t)erroResp;
			msg_ptr->data[1] = 0;
			msg_ptr->header.SIZE = APP_MESSAGE_NO_ARRAY_SIZE + 1;
			if ( BSP_CAN_DEVICE_0 == (pcan->instance ) {
				msg_ptr->portNum = MIC_CDC_USB_3;
			}
			else {
				msg_ptr->portNum = MIC_CDC_USB_4;
			}
			_msgq_send (msg_ptr);
#endif
		}
		else
			_time_delay(1);

	} while ( 1 );

	//TODO
//	if (MQX_OK != _lwevent_destroy(&(((pflexcanInstance_t)param)->canState.event_ISR)))
//	{
//		printf("_lwevent_destroy event failed\n");
//		return;// ( kStatus_FLEXCAN_Fail );
//	}
}

//#define MIC_LED_TEST

//#define FLEXCAN_DEVICE_USB_PACKET_AGGREGATION

void FLEXCAN_Rx_Task( uint32_t param_in ) {

	uint32_t                    result, icount;
	_mqx_uint                   queue_count;
	pFLEXCAN_queue_element_t    pqueue_msg;
	uint8_t                     *pmsg_str;
	uint32_t                    idx, i;
	int32_t                     curr_msg_len;
	pcdc_mic_queue_element_t    pqMemElem;
	
	pflexcanInstance_t pcan = (pflexcanInstance_t)param_in;

	if ( (NULL == pcan) || (BOARD_CAN_INSTANCE <= pcan->instance) ) {
		printf( "%s:ERROR,CAN_RX thread incorrect param %u\n", __func__, pcan->instance );
		return;
	}

	printf("%s:INFO Loop instance %u \n", __func__, ((pflexcanInstance_t)pcan)->instance);

	do {
		if (false == pcan->bScanInstanceStarted )
		{
			_time_delay(2);
			continue;
		}
		result = FLEXCAN_DRV_GetReceiveStatusBlocking(pcan->instance, &idx, 0);

		if ( !result ) {
			pmsg_str = NULL;
			curr_msg_len = 0;
			icount = 0;
			do {
                //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                //RS Add latency to send flow control need to be changed
                //
				//Accamulate the CAN messages before USB send
				queue_count = FlexCanMsg_queue_get_size(&(pcan->Rx_ReadyMSGQueue));
				if ( (RX_FLEXCAN_MSGQ_TRESHOLD_MIN < queue_count || (1 < icount++))/* && ( USB_CAN_MAX_USABLE > _msg_available(g_out_message_pool) )*/ ) {
					//printf("CanRxT: RB_%d URB_%d\n", queue_count, GetUSBFreeBufferCount(((pflexcanInstance_t)param)->instance + 2)/*, _msg_available(g_out_message_pool)*/);
					break;
				}

				_time_delay(1);
			} while (1);

			if (queue_count) {

				for ( i = 0; i < queue_count; i++ ) {
					if (NULL == pqMemElem) {
						pqMemElem = GetUSBWriteBuffer (pcan->instance + 2);
						if (NULL == pqMemElem) {
							printf("%s:ERROR, get mem for USB response\n", __func__);
							continue;
						}
						else {
							pmsg_str = pqMemElem->data_buff;
							pqMemElem->send_size = 0;
						}
					}

					if (NULL != pqMemElem) {
#ifdef FLEXCAN_DEVICE_DEBUG_
						printf("%s:ERROR, Get from queue %x elm %x\n",__func__, (uint32_t)(&(pcan->Rx_ReadyMSGQueue)), (uint32_t)pqueue_msg );
#endif
						pqueue_msg = (pFLEXCAN_queue_element_t)FlexCanMsg_queue_dequeue(&(pcan->Rx_ReadyMSGQueue));
					}

					if ( pqMemElem && pqueue_msg ) {

						APPLICATION_MESSAGE_T *app_msg;
						curr_msg_len = ParseCanMessToString (pqueue_msg, (const uint8_t*)pmsg_str, &pcan->flowcontrol);
						FlexCanMsg_queue_enqueue (&(pcan->Rx_FreeMSGQueue), (QUEUE_ELEMENT_STRUCT_PTR)pqueue_msg);
#ifdef FLEXCAN_DEVICE_DEBUG_
						printf("%s:INFO, Return to queue %x elm %x\n", __func__, (uint32_t)(&(pcan->Rx_FreeMSGQueue)), (uint32_t)pqueue_msg );
#endif
						pqueue_msg = NULL;

						if (pcan->flowcontrol.match_position != FLOW_CONTROL_INVALID_POS){
							do {
								if ((app_msg = (APPLICATION_MESSAGE_PTR_T)_msg_alloc(g_in_message_pool)) == NULL)
								{
									printf("%s:ERROR: app message allocation failed\n", __func__);
									break;
								}
								_mem_copy ( pcan->flowcontrol.p_response[pcan->flowcontrol.match_position], app_msg->data, pcan->flowcontrol.resp_size[pcan->flowcontrol.match_position] );
								app_msg->header.SOURCE_QID = _msgq_get_id( 0, USB_QUEUE );
								//((pflexcanInstance_t)param)->suppress_tx_msg = TRUE;
								if ( (pcan->instance) ) {
									app_msg->header.TARGET_QID = _msgq_get_id(0, CAN2_TX_QUEUE);
								}
								else {
									app_msg->header.TARGET_QID = _msgq_get_id(0, CAN1_TX_QUEUE);
								}
								app_msg->header.SIZE = pcan->flowcontrol.resp_size[pcan->flowcontrol.match_position] + APP_MESSAGE_NO_ARRAY_SIZE;
								app_msg->portNum = MIC_CDC_USB_3;
								_msgq_send (app_msg);
							} while (0);
						}

						if (0 > curr_msg_len) {
							printf("%s:ERROR pars CAN to String\n", __func__);
							//TODO
							//Add Error handler
						}
						else {
							pqMemElem->send_size += curr_msg_len;
							if ((g_CanCDCPacketsize - pqMemElem->send_size) < FLEXCAN_MAX_MSG_STR_SIZE) {
								pmsg_str = NULL;

								//put message to USB send
								if (!SetUSBWriteBuffer(pqMemElem, (pcan->instance + 2))) {
									printf("%s:ERROR send data to CDC1\n", __func__);
									//TODO
									//ADD error handler
								}
								pqMemElem = NULL;
							}
							else {
								pmsg_str += curr_msg_len;
							}
						}
						g_Flexdebug.sendUSB++;
						//printf("Send USB %d\n", g_Flexdebug.sendUSB);

					}//End if ( pqMemElem && pqueue_msg )
					else {
						if (pqMemElem) {
							if (!SetUSBWriteBuffer(pqMemElem, (pcan->instance + 2))) {
								printf("%s:ERROR send data to CDC1\n", __func__);
								//TODO
								//ADD error handler
							}
							pqMemElem = NULL;
						}
						if (pqueue_msg) {
							FlexCanMsg_queue_enqueue (&(pcan->Rx_FreeMSGQueue), (QUEUE_ELEMENT_STRUCT_PTR)pqueue_msg);
							pqueue_msg = NULL;
						}
						_time_delay(2);
					} //End if ( pmsg_data && pqueue_msg ) else

				}//End for ( i = 0; i < queue_count; i++ )
				if (pqMemElem) {
					if (!SetUSBWriteBuffer(pqMemElem, (pcan->instance + 2))) {
						printf("%s:ERROR send data to CDC1\n", __func__);
						//TODO
						//ADD error handler
					}
					pqMemElem = NULL;
				}

			}//End if (queue_count)

		}//End if ( !result ) CAN message axxeptable in queue

		_time_delay(1);

	} while (0 == g_flag_Exit);
}

pflexcanInstance_t FlexCanDevice_GetInstance( flexcandevice_module_t moduleID ) {
	if ( fdcandevice_CAN1_MAX > moduleID ) {
		return NULL;
	}

	return &g_flexcanDeviceInstance[moduleID];
}

flexcan_device_status_t FlexCanDevice_InitInstance(  uint8_t instNum, pflexcandevice_initparams_t pinstance_Can ) {
	flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;
	int i;
	pFLEXCAN_queue_element_t pqueue_elem;

	if ( !pinstance_Can || (BOARD_CAN_INSTANCE <= instNum) ) {
		return fcStatus_FLEXCAN_Error;
	}

	_mem_zero((void *)(g_flexcanDeviceInstance[instNum].canState.MB_config), sizeof(g_flexcanDeviceInstance[instNum].canState.MB_config));

	//g_flexcanDeviceInstance[instNum].instance                         = (uint32_t)instNum;
	g_flexcanDeviceInstance[instNum].flexcanData.flexcanMode          = (flexcan_operation_modes_t)pinstance_Can->flexcanMode; //kFlexCanDisableMode;
	g_flexcanDeviceInstance[instNum].flexcanData.is_rx_fifo_needed    = pinstance_Can->is_rx_fifo_needed; //false;
	g_flexcanDeviceInstance[instNum].flexcanData.max_num_mb           = pinstance_Can->max_num_mb - 1; //16;
	g_flexcanDeviceInstance[instNum].flexcanData.num_id_filters       = pinstance_Can->num_id_filters; //kFlexCanRxFifoIDFilters_8;
	g_flexcanDeviceInstance[instNum].instanceBitrate                  = pinstance_Can->instanceBitrate;
	g_flexcanDeviceInstance[instNum].initialize                       = true;

	if ( g_flexcanDeviceInstance[instNum].initialize ) {
		ret = (flexcan_device_status_t)_mutex_init(&(g_flexcanDeviceInstance[instNum].mutex_MB_sync), NULL);
		if ( MQX_EOK != ret ) {
			printf("%s:ERROR, init mutex for instance %u - %x\n", __func__, instNum, ret);
			g_flexcanDeviceInstance[instNum].initialize = false;
			ret = fcStatus_FLEXCAN_Error;
		}
	}

	if (NULL != g_flexcanDeviceInstance[instNum].pMesagebuff) {
		pqueue_elem = (pFLEXCAN_queue_element_t)(g_flexcanDeviceInstance[instNum].pMesagebuff);

		while (!FlexCanMsg_queue_is_empty(&(g_flexcanDeviceInstance[instNum].Rx_ReadyMSGQueue))) {
			FlexCanMsg_queue_dequeue (&(g_flexcanDeviceInstance[instNum].Rx_ReadyMSGQueue));
		}

		while (!FlexCanMsg_queue_is_empty(&(g_flexcanDeviceInstance[instNum].Rx_FreeMSGQueue))) {
			FlexCanMsg_queue_dequeue (&(g_flexcanDeviceInstance[instNum].Rx_FreeMSGQueue));
		}

		for (i = 0; i < RX_FLEXCAN_MSGQ_MESAGES; i++) {
			if (!FlexCanMsg_queue_enqueue (&(g_flexcanDeviceInstance[instNum].Rx_FreeMSGQueue), (QUEUE_ELEMENT_STRUCT_PTR)pqueue_elem)) {
				printf("%s:ERROR add element to queue\n", __func__);
				ret = fcStatus_FLEXCAN_Error;
				break;
			}
			pqueue_elem++;
		}
	}

	return ret;
}

flexcan_device_status_t FlexCanDevice_Init( pflexcandevice_initparams_t pinstance_Can0, pflexcandevice_initparams_t pinstance_Can1 ) {
	flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;

	if ( NULL != pinstance_Can0 ) {
		ret = FlexCanDevice_InitInstance(BSP_CAN_DEVICE_0, pinstance_Can0);
		if ( 0 > ret ) {
			printf("%s:ERROR Initializing instance 0\n", __func__);
		}
	}

	if ( NULL != pinstance_Can1 ) {
		ret = FlexCanDevice_InitInstance(BSP_CAN_DEVICE_1, pinstance_Can1);
		if ( 0 > ret ) {
			printf("%s:ERROR Initializing instance 1\n", __func__);
		}
	}

	return ret;
}

flexcan_device_status_t FlexCanDevice_DeInit( pflexcanInstance_t pInstance ) {
	flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;

	if ( !pInstance ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	if ( !pInstance->initialize ) {
		return fcStatus_FLEXCAN_Success;
	}

	pInstance->bIsInstanceStop = true;

	ret = FlexCanDevice_Stop(pInstance);
	if ( fcStatus_FLEXCAN_Success != ret ) {
		printf("%s:ERROR FlexCanDevice_Stop - %x\n", __func__, ret);
	}

	if ( MQX_EOK != _mutex_destroy(&(pInstance->mutex_MB_sync)) ) {
		printf("%s:ERROR _mutex_destroy\n", __func__);
	}

	pInstance->initialize = false;

	return ret;
}

flexcan_device_status_t FlexCanDevice_Start( pflexcanInstance_t pInstance ) {
	flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;

	if ( !pInstance ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	if ( !pInstance->initialize ) {
		printf("%s:ERROR, Instance not initialized correctly\n", __func__);
		return fcStatus_FLEXCAN_Error;
	}

	ret = (flexcan_device_status_t)FLEXCAN_DRV_Init(pInstance->instance, &(pInstance->canState), &(pInstance->flexcanData));
	if ( fcStatus_FLEXCAN_Success < ret ) {
		printf("\n%s:ERROR:FLEXCAN initialization failed. result: 0x%x \n", __func__, ret);
		return ret;
	}
	//pInstance->bScanInstanceStarted = (uint32_t)true;

	ret = FlexCanDevice_SetBitrate(pInstance, pInstance->instanceBitrate);

	return ret;
}

flexcan_device_status_t FlexCanDevice_Stop( pflexcanInstance_t pInstance ) {
	flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;

	if ( !pInstance ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	if ( !pInstance->bScanInstanceStarted ) {
		return ret;
	}

	FLEXCAN_DRV_Deinit(pInstance->instance);
	pInstance->bScanInstanceStarted = false;
	return ret;
}

flexcan_device_status_t FlexCanDevice_SetBitrate( pflexcanInstance_t pInstance, flexcan_device_bitrate_t bitRate ) {
	flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;
	uint32_t canPeClk;

	if ( !pInstance ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	if ( !pInstance->initialize || fdBitrate_MAX <= bitRate ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	/* Get the CAN clock used to feed the CAN protocol engine */
	if ( FLEXCAN_HAL_GetClock(g_flexcanBase[pInstance->instance]) ) {
		canPeClk = CLOCK_SYS_GetFlexcanFreq(0, kClockFlexcanSrcBusClk);
	} else {
		canPeClk = CLOCK_SYS_GetFlexcanFreq(0, kClockFlexcanSrcOsc0erClk);
	}

	pInstance->canPeClk = canPeClk;
	printf( "%s:INFO:FlexCan clock %u\n", __func__, canPeClk );

	/* Decide which table to use */
	switch ( pInstance->canPeClk ) {
	case 60000000:
		ret = (flexcan_device_status_t)FLEXCAN_DRV_SetBitrate(pInstance->instance, &bitRateTable60Mhz[bitRate]);
		break;
	case 48000000:
		ret = (flexcan_device_status_t)FLEXCAN_DRV_SetBitrate(pInstance->instance, &bitRateTable48Mhz[bitRate]);
		break;
	default:
		if ( (canPeClk > 74990000) && (canPeClk <= 75000000) ) {
			ret = (flexcan_device_status_t)FLEXCAN_DRV_SetBitrate(pInstance->instance, &bitRateTable75Mhz[bitRate]); // 125kbps
		} else {
			// printf("\r\nFLEXCAN bitrate table not available for PE clock: %d", canPeClk);
			return fcStatus_FLEXCAN_Fail;
		}
	}

	return ret;
}

flexcan_device_status_t FlexCanDevice_GetBitrate( pflexcanInstance_t pInstance, flexcan_device_bitrate_t *pbitRate ) {
	flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;
	flexcan_time_segment_t  instanceTimeSegment;

	if ( !pInstance ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	if ( !pInstance->initialize || NULL != pbitRate ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	ret = (flexcan_device_status_t)FLEXCAN_DRV_GetBitrate(pInstance->instance, &instanceTimeSegment);

	switch ( pInstance->canPeClk ) {
	case 60000000:
		*pbitRate = ConvertTimetoBitRate(bitRateTable60Mhz, &instanceTimeSegment);
		if ( -1 == *pbitRate ) {
			ret = fcStatus_FLEXCAN_Error;
		}
		break;
	case 48000000:
		*pbitRate = ConvertTimetoBitRate(bitRateTable48Mhz, &instanceTimeSegment);
		if ( -1 == *pbitRate ) {
			ret = fcStatus_FLEXCAN_Error;
		}
		break;
	default:
		if ( (pInstance->canPeClk > 74990000) && (pInstance->canPeClk <= 75000000) ) {
			*pbitRate = ConvertTimetoBitRate(bitRateTable75Mhz, &instanceTimeSegment);
			if ( -1 == *pbitRate ) {
				ret = fcStatus_FLEXCAN_Error;
			}
		} else {
			printf("\n%s:ERROR, FLEXCAN bitrate table not available for PE clock: %d \n", __func__, (int)pInstance->canPeClk);
			return fcStatus_FLEXCAN_Fail;
		}
	}
	return ret;
}

flexcan_device_bitrate_t ConvertTimetoBitRate( flexcan_time_segment_t *ptimeSegmentTable,  flexcan_time_segment_t *ptimeSegment ) {
	int32_t i = 0;

	for ( i = 0; i < fdBitrate_MAX; i++ ) {
		if ( ptimeSegment->phaseSeg1 == ptimeSegmentTable[i].phaseSeg1 && ptimeSegment->phaseSeg2 == ptimeSegmentTable[i].phaseSeg2 &&
			 ptimeSegment->preDivider == ptimeSegmentTable[i].preDivider && ptimeSegment->propSeg == ptimeSegmentTable[i].propSeg &&
			 ptimeSegment->rJumpwidth == ptimeSegmentTable[i].rJumpwidth ) {
			return (flexcan_device_bitrate_t)i;
		}
	}
	return fdBitrate_Error;
}

flexcan_device_status_t FlexCanDevice_setMailbox( pflexcanInstance_t pinstance, flexcan_msgbuff_id_type_t id_type, uint32_t id, uint32_t mask, bool enabled ) {
	flexcan_data_info_t rxInfo;
	//_mqx_uint mRet;
	bool bprev;
	flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;

	if ( !pinstance ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	if ( !pinstance->bScanInstanceStarted ) {
		return fcStatus_FLEXCAN_Error;
	}

	if ( MAX_MB_NUMBER <= id  ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	if ( (pinstance->canState.MB_config[id].iD == id) && (pinstance->canState.MB_config[id].iD_Mask == mask) &&
		 (pinstance->canState.MB_config[id].iD_type == id_type) && (pinstance->canState.MB_config[id].isEnable == enabled) ) {
		return ret;
	}

	//mRet = _mutex_lock(&(pinstance->mutex_MB_sync));

	bprev = pinstance->canState.MB_config[id].isEnable;
	pinstance->canState.MB_config[id].iD         = id;
	pinstance->canState.MB_config[id].iD_Mask    = mask;
	pinstance->canState.MB_config[id].iD_type    = id_type;
	pinstance->canState.MB_config[id].isEnable   = enabled;

	//mRet = _mutex_unlock( &(pinstance->mutex_MB_sync) );

	if ( enabled ) {
		_mutex_lock(&(pinstance->mutex_MB_sync));
		ret = FlexCanDevice_SetRxIndividualMask(pinstance, id_type, id, mask);
		printf("\n%s:INFO, FlexCAN SetRxIndividualMask ret %u MB %x\n", __func__, ret, id);

		if ( bprev != enabled ) {
			rxInfo.msg_id_type = pinstance->canState.MB_config[id].iD_type;
			rxInfo.data_length = kFlexCanMessageSize;

			printf("\n%s:INFO, FlexCAN MB receive config MB %x\n", __func__, id);

			/* Configure RX MB fields*/
			ret = (flexcan_device_status_t)FLEXCAN_DRV_ConfigRxMb(pinstance->instance, id, &rxInfo, pinstance->canState.MB_config[id].iD_Mask);
			if ( ret ) {
				//numErrors++;
				printf("\n%s:ERROR, FlexCAN RX MB configuration failed. result: 0x%lx\n", __func__, ret);
			}
			//Enable Interrupt and start recieving
			ret = (flexcan_device_status_t)FLEXCAN_DRV_RxMessageBuffer(pinstance->instance, pinstance->canState.MB_config[id].iD, NULL);
			printf("\n%s:INFO, FLEXCAN_DRV_RxMessageBuffer. result: 0x%lx \n", __func__, ret);
		}
		_mutex_unlock(&(pinstance->mutex_MB_sync));

		return ret;
	}

	_mutex_lock(&(pinstance->mutex_MB_sync));
	ret = FlexCanDevice_SetRxIndividualMask(pinstance, id_type, id, 0);
	_mutex_unlock(&(pinstance->mutex_MB_sync));
	return ret;
}

flexcan_device_status_t FlexCanDevice_TxMessage ( pflexcanInstance_t pinstance, uint32_t MbId, pflexcandevice_TX_data_t pTxData, bool is_remote_frame ) {
	flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;
	flexcan_data_info_t txInfo;

	if ( !pinstance || !pTxData ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	if ( !pinstance->bScanInstanceStarted ) {
		return fcStatus_FLEXCAN_Error;
	}

	txInfo.msg_id_type = pTxData->msgbuffType;
	txInfo.data_length = pTxData->msgSize;

	ret = (flexcan_device_status_t)FLEXCAN_DRV_ConfigTxMb(pinstance->instance, MbId, &txInfo, pTxData->msgID);
	if ( fcStatus_FLEXCAN_Success == ret ) {
		//ret = (flexcan_device_status_t)FLEXCAN_DRV_SendBlocking(pinstance->instance, MbId, &txInfo, pTxData->msgID, pTxData->msgData, FLEXCAN_DEVICE_TX_TIMEOUT);
		ret = (flexcan_device_status_t)FLEXCAN_DRV_Send(pinstance->instance, MbId, &txInfo, pTxData->msgID, pTxData->msgData, is_remote_frame);
	}
	else {
		printf("%s:ERROR, FLEXCAN_DRV_ConfigTxMb\n", __func__);
	}

	return ret;
}

flexcan_device_status_t FlexCanDevice_SetTermination( pflexcanInstance_t pinstance, bool isSet ) {
	flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;

	if ( !pinstance ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	if ( BSP_CAN_DEVICE_0 == pinstance->instance ) {
		if ( isSet ) {
			GPIO_DRV_ClearPinOutput(CAN1_TERM_ENABLE);
			return ret;
		}
		GPIO_DRV_SetPinOutput(CAN1_TERM_ENABLE);
		return ret;
	}

	if ( BSP_CAN_DEVICE_1 == pinstance->instance ) {
		if ( isSet ) {
			GPIO_DRV_ClearPinOutput(CAN2_TERM_ENABLE);
			return ret;
		}
		GPIO_DRV_SetPinOutput(CAN2_TERM_ENABLE);
		return ret;
	}

	return fcStatus_FLEXCAN_InvalidArgument;
}

/* Set CAN instance operation mode*/
flexcan_device_status_t FlexCanDevice_SetOperationMode( pflexcanInstance_t pinstance, flexcaninstance_operation_modes_t mode ) {
	if ( !pinstance || fdFlexCanMode_MAX > mode ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	return (flexcan_device_status_t)FLEXCAN_HAL_SetOperationMode(g_flexcanBase[pinstance->instance], (flexcan_operation_modes_t)mode);
}

/* Set individual or global RX masking type*/
flexcan_device_status_t FlexCanDevice_SetRxMaskType( pflexcanInstance_t pinstance, bool isGlobal ) {
	if ( !pinstance ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	if ( isGlobal ) {
		FLEXCAN_DRV_SetRxMaskType(pinstance->instance, kFlexCanRxMaskGlobal);
		return fcStatus_FLEXCAN_Success;
	}

	FLEXCAN_DRV_SetRxMaskType(pinstance->instance, kFlexCanRxMaskIndividual);
	return fcStatus_FLEXCAN_Success;
}

/* Sets the FlexCAN RX FIFO global standard or extended mask */
flexcan_device_status_t FlexCanDevice_SetRxFifoGlobalMask( pflexcanInstance_t pinstance, flexcan_msgbuff_id_type_t id_type, uint32_t mask ) {
	if ( !pinstance ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	return (flexcan_device_status_t)FLEXCAN_DRV_SetRxFifoGlobalMask(pinstance->instance, id_type, mask);
}

/* Sets the FlexCAN RX MB global standard or extended mask */
flexcan_device_status_t FlexCanDevice_SetRxMbGlobalMask( pflexcanInstance_t pinstance, flexcan_msgbuff_id_type_t id_type, uint32_t mask ) {
	if ( !pinstance ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	return (flexcan_device_status_t)FLEXCAN_DRV_SetRxMbGlobalMask(pinstance->instance, id_type, mask);
}

/* Sets the FlexCAN RX individual standard or extended mask */
flexcan_device_status_t FlexCanDevice_SetRxIndividualMask( pflexcanInstance_t pinstance, flexcan_msgbuff_id_type_t id_type, uint32_t mb_idx, uint32_t mask ) {
	if ( !pinstance ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	return (flexcan_device_status_t)FLEXCAN_DRV_SetRxIndividualMask(pinstance->instance, id_type, mb_idx, mask);
}

flexcan_device_status_t DecodeSendTxMessage ( const char* buff, uint32_t bufflen, pflexcandevice_TX_data_t pTxData, flexcan_msgbuff_id_type_t msg_type, bool is_remote_frame ) {
	flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;
	uint32_t i, msg_limit = 3;
	uint8_t tmp;
	int8_t *pbuff =  (int8_t*)buff;

	if ( NULL == buff || 0 == bufflen ||  NULL == pTxData) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	_mem_zero ((void*)pTxData, sizeof(flexcandevice_TX_data_t));
	
	pTxData->msgbuffType = msg_type;
	if ( kFlexCanMsgIdExt == msg_type ){
		msg_limit = 8;
	}
	
	for ( i = 0; CAN_OK_RESPONCE != *pbuff && 1 < bufflen; i++ ) {
		if ( msg_limit > i ) {
			pTxData->msgID <<= 4;
			if ( fcStatus_FLEXCAN_Success != parseHex (pbuff, 1, &tmp) )
				return fcStatus_FLEXCAN_Error;
			pTxData->msgID |= (tmp & 0x0F);
			pbuff++;
			continue;
		}
		if ( msg_limit == i ) {
			if ( fcStatus_FLEXCAN_Success != parseHex (pbuff, 1, &tmp) )
				return fcStatus_FLEXCAN_Error;
			pTxData->msgSize = (tmp & 0x0F);
			pbuff++;
			continue;
		}
		if (!is_remote_frame){ /* no data if it is a remote frame */
			if ( fcStatus_FLEXCAN_Success != parseHex (pbuff, 2, &tmp) )
				return fcStatus_FLEXCAN_Error;
			pTxData->msgData[i-(msg_limit + 1)] = (tmp & 0xFF);
			pbuff += 2;
		}
		else{
			pbuff++; /* increment pbuff so we can continue search for CAN_OK_RESPONSE(\r) */
		}
		if (i > bufflen){ /* Never found CAN_OK_RESPONSE in packet*/
			return fcStatus_FLEXCAN_Error;
		}
	}
	pbuff++;
	return ret;
}

flexcan_device_status_t		DecodeFlowCmd ( const char* buff, uint32_t bufflen, p_flowcontrol_t p_flowCmdTable, uint8_t* bytes_read ){
    flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;
    uint32_t i, id_size = CAN_MSG_ID_SIZE;
	uint8_t tmp;
	int8_t *pbuff =  (int8_t*)buff;
	uint8_t idx = p_flowCmdTable->idx;
	uint8_t msg_size = 0;
	uint8_t resp_data_bytes_length = 0;

	if ( NULL == buff || 0 == bufflen ||  NULL == p_flowCmdTable ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	_mem_zero ((void*)p_flowCmdTable->p_response[idx], FLEXCAN_FLOW_CTR_COMMAND_MAX_SIZE);
	p_flowCmdTable->resp_size[idx] = 0;
	p_flowCmdTable->msg_id[idx] = 0;

	if ( TRUE ==  p_flowCmdTable->bisExtended[idx] ){
		id_size = CAN_MSG_ID_SIZE_EXT;
		p_flowCmdTable->p_response[idx][0] = 'T';
		p_flowCmdTable->resp_size[idx] += 1;
		parseHex(pbuff + 8 + 8, 1, &resp_data_bytes_length); /* Get 'L' char and convert to int */
		msg_size = 8 + (resp_data_bytes_length*2) + 1; /* msg ID + datachars + \r */
	}
	else{
		id_size = CAN_MSG_ID_SIZE;
		p_flowCmdTable->p_response[idx][0] = 't';
		p_flowCmdTable->resp_size[idx] += 1;
		parseHex(pbuff + 3 + 3, 1, &resp_data_bytes_length); /* Get 'L' char and convert to int */
		msg_size = 3 + (resp_data_bytes_length*2) + 1; /* msg ID + datachars + \r */
	}

	/* convert ID from ASCII to uint */
	for ( i = 0; CAN_OK_RESPONCE != *pbuff && i < id_size; i++ ) {
		p_flowCmdTable->msg_id[idx] <<= 4;
		if ( fcStatus_FLEXCAN_Success != parseHex (pbuff, 1, &tmp) )
			return fcStatus_FLEXCAN_Error;
		p_flowCmdTable->msg_id[idx] |= (tmp & 0x0F);
		pbuff++;
		(*bytes_read)++;
	}

	/* copy t or T message for flowcontrol response */
	_mem_copy((const void*)(pbuff), (void*)&(p_flowCmdTable->p_response[idx][1]), msg_size);
	p_flowCmdTable->resp_size[idx] += msg_size;
	p_flowCmdTable->p_response[idx][p_flowCmdTable->resp_size[idx]] = '\r';
	p_flowCmdTable->resp_size[idx] += 1;

	(*bytes_read) += msg_size;
    return ret;
}

flexcan_device_status_t parseHex(int8_t * line, uint8_t len, uint8_t * value) {
	*value = 0;

	if (0 == len) {
		return fcStatus_FLEXCAN_OutOfRange;
	}

	while (len--) {
		if (*line == 0) return fcStatus_FLEXCAN_OutOfRange;
		*value <<= 4;
		if ((*line >= '0') && (*line <= '9')) {
		   *value += *line - '0';
		} else if ((*line >= 'A') && (*line <= 'F')) {
		   *value += *line - 'A' + 10;
		} else if ((*line >= 'a') && (*line <= 'f')) {
		   *value += *line - 'a' + 10;
		} else return fcStatus_FLEXCAN_OutOfRange;
		line++;
	}
	return fcStatus_FLEXCAN_Success;
}

bool parseAsciToUInt (const int8_t* line, uint8_t len, uint32_t* val){
	int8_t* ptr = (int8_t*)line;
	uint16_t tmp;


	if (NULL == line || (8 > len && 0 != len) || NULL == val) {
		printf("%s:ERROR parseAsciToInt\n", __func__);
		return false;
	}
	*val = 0;

	if (!parseAsciToShort((const uint8_t*)ptr, &tmp)) {
		return false;
	}
	*val = tmp;
	ptr += 4;

	if (!parseAsciToShort((const uint8_t*)ptr, &tmp)) {
		return false;
	}

	*val = (*val << 16)|(tmp & 0x0000FFFF);

	return true;
}

bool parseAsciToShort (const uint8_t* pbuff, uint16_t* val) {
	uint8_t* ptr = (uint8_t*)pbuff;
	uint8_t len = 4;

	if (NULL == ptr || NULL == val) {
		printf("%s:ERROR parseAsciToShort\n", __func__);
		return false;
	}
	*val = 0;

	while (len--) {
		*val <<= 4;
		if ((*ptr >= '0') && (*ptr <= '9')) {
		   *val += *ptr - '0';
		} else if ((*ptr >= 'A') && (*ptr <= 'F')) {
		   *val += *ptr - 'A' + 10;
		} else if ((*ptr >= 'a') && (*ptr <= 'f')) {
		   *val += *ptr - 'a' + 10;
		} else return false;
		ptr++;
	}
	return true;
}

bool AllocateFIFOFilterTable (pflexcanInstance_t pinst, flexcan_rx_fifo_id_filter_num_t filtnum, flexcan_rx_fifo_id_element_format_t tableFormat)
{
	_mqx_uint ret;

	if (!pinst) {
		return false;
	}

	if (pinst->pFIFOIdFilterTable && 0 != pinst->FIFOFilterTableSize) {
		ret = _mem_free ((void*)pinst->pFIFOIdFilterTable);
		if (MQX_OK != ret) {
			printf("%s:ERROR(%d) free FIFO filter table\n", __func__, ret);
			return false;
		}
	}

	pinst->FIFOTableIndx = 0;
	switch (tableFormat) {
	case kFlexCanRxFifoIdElementFormatA:
	case kFlexCanRxFifoIdElementFormatD:
		pinst->FIFOFilterTableSize = ((filtnum + 1) << 3) * sizeof(flexcan_id_table_t);
		break;
	case kFlexCanRxFifoIdElementFormatB:
		pinst->FIFOFilterTableSize = ((filtnum + 1) << 4) * sizeof(flexcan_id_table_t);
		break;
	case kFlexCanRxFifoIdElementFormatC:
		pinst->FIFOFilterTableSize = ((filtnum + 1) << 6) * sizeof(flexcan_id_table_t);
		break;
	default:
		pinst->FIFOFilterTableSize = 0;
		pinst->pFIFOIdFilterTable = NULL;
		return false;
	}

	pinst->pFIFOIdFilterTable = (flexcan_id_table_t*)_mem_alloc_zero( pinst->FIFOFilterTableSize );
	if (NULL == pinst->pFIFOIdFilterTable) {
		printf( "%s:ERROR allocate FIFO ID filter table elem\n", __func__);
		pinst->FIFOFilterTableSize = 0;
		return false;
	}
	pinst->FIFOTableIndx = 0;

	return true;
}

bool AllocateFIFOMaskTable (pflexcanInstance_t pinst)
{
	_mqx_uint ret;

	if (!pinst) {
		return false;
	}

	if (pinst->pFIFOIdMaskTable && 0 != pinst->FIFOMaskTableSize) {
		ret = _mem_free ((void*)pinst->pFIFOIdMaskTable);
		if (MQX_OK != ret) {
			printf("%s:ERROR(%d) free FIFO mask table\n", __func__, ret);
			return false;
		}
	}

	pinst->FIFOMaskTableIndx = 0;

	pinst->FIFOMaskTableSize = 16; /* we can only set a maximum of 16 masks in FIFO mode */

	pinst->pFIFOIdMaskTable = (flexcan_mask_id_table_t*)_mem_alloc_zero( (pinst->FIFOMaskTableSize * sizeof(flexcan_mask_id_table_t)));
	if (NULL == pinst->pFIFOIdMaskTable) {
		printf( "%s:ERROR, allocate FIFO ID filter table elem\n", __func__);
		pinst->FIFOMaskTableSize = 0;
		return false;
	}
	pinst->FIFOMaskTableIndx = 0;

	return true;
}

int32_t ParseCanMessToString (pFLEXCAN_queue_element_t pCanMess, const uint8_t *pDestBuff, flowcontrol_t * flowcontrol) {
	uint8_t   tmp, tmp1, ind, *pmsg_str, curr_msg_len = 0;
	
	bool remote_frame = false;

	if (NULL == pCanMess || NULL == pDestBuff) {
		printf("%s:ERROR, wrong params\n", __func__);
		return -1;
	}

	pmsg_str = (uint8_t*)pDestBuff;

	//Detect remote or regular message
	tmp = (uint8_t) ((pCanMess->msg_buff.cs >> 20) & 0x1);
	tmp1 = (uint8_t)((pCanMess->msg_buff.cs >> 21) & 0x1);
	if (tmp) {
		//remoute frame
		*pmsg_str = 'r';
		if (tmp1) {
			*pmsg_str = 'R';
		}
		remote_frame = true;
	}
	else {
		//standard frame
		*pmsg_str = 't';
		if (tmp1) {
			*pmsg_str = 'T';
		}
		remote_frame = false;
	}
	pmsg_str++;
	curr_msg_len++;
	//set message ID;
	if (tmp1) {
		//Extended
		sprintf ( (char*)pmsg_str, "%08x", pCanMess->msg_buff.msgId);
		pmsg_str += 8;
		curr_msg_len += 8;
	}
	else {
		//Standard
		sprintf ( (char*)pmsg_str, "%03x", pCanMess->msg_buff.msgId);
		pmsg_str += 3;
		curr_msg_len += 3;
	}
	//Message length
	tmp = (uint8_t)((pCanMess->msg_buff.cs >> 16) & 0xF);
	*pmsg_str = tmp + '0';
	pmsg_str++;
	curr_msg_len ++;
	
	if (!remote_frame){
		for (ind = 0; ind < tmp; ind++) {
			tmp1 = (pCanMess->msg_buff.data[ind]>>4) & 0xF;
			if (tmp1 > 9 )
				*pmsg_str = tmp1 - 10 + 'A';
			else
				*pmsg_str = tmp1 + '0';

			pmsg_str++;
			curr_msg_len++;
			tmp1 = pCanMess->msg_buff.data[ind] & 0xF;
			if (tmp1 > 9 )
				*pmsg_str = tmp1 - 10 + 'A';
			else
				*pmsg_str = tmp1 + '0';

			pmsg_str++;
			curr_msg_len++;
		}
	}
	//Message time stamp
	pmsg_str += 3;
	for (ind = 0; ind < 4; ind++) {
		tmp1 = (uint8_t)((pCanMess->msg_buff.cs >> (ind<<2)) & 0xF);
		if (tmp1 > 9 )
			*pmsg_str = tmp1 - 10 + 'A';
		else
			*pmsg_str = tmp1 + '0';

		pmsg_str--;
	}
	pmsg_str += 5;
	curr_msg_len += 5;

	if ( 0x10 == pCanMess->msg_buff.data[0]) {
		//check for a match in flow control array
		for (ind = 0; ind < FLOW_CONTROL_ARR_SIZE; ind++){
			/* check to see if we have gone through all the valid flowcontrol entries */
			if (flowcontrol->msg_id[ind] == FLOW_CONTROL_INVALID_ID){
				break;
			}
			if (pCanMess->msg_buff.msgId == flowcontrol->msg_id[ind]){
				//printf("\nflow control found at index %d, msgid %d\n", ind, flowcontrol->msg_id[ind]);
				flowcontrol->match_position = ind;
				break;
			}
		}
	}
	else{
		flowcontrol->match_position = FLOW_CONTROL_INVALID_POS;
	}

	//Add CAN_OK_RESPONCE character
	*pmsg_str = CAN_OK_RESPONCE;

	return (int32_t)curr_msg_len;
}

bool CheckCommandIdSupp ( const uint8_t* buff, uint16_t* IdVal) {
	uint8_t* src = (uint8_t*)buff;

	if(NULL == buff || NULL == IdVal){
		printf("%s:ERROR, param\n", __func__ );
		return false;
	}

	if('u' != *src){
		return false;
	}
	src++;

	if (!parseAsciToShort(src, IdVal)) {
		return false;
	}

	return true;
}

int8_t convert_mask_to_ASCII(flexcan_mask_id_table_t * pFIFOIdMaskTable, char * pbuff){
	int8_t msg_size = 0;
	if(pFIFOIdMaskTable->isExtendedFrame && !pFIFOIdMaskTable->isRemoteFrame ){
		*pbuff = 'T';
	}
	else if(!pFIFOIdMaskTable->isExtendedFrame && !pFIFOIdMaskTable->isRemoteFrame ){
		*pbuff =  't';
	}
	else if (pFIFOIdMaskTable->isExtendedFrame && pFIFOIdMaskTable->isRemoteFrame ){
		*pbuff = 'R';
	}
	else if (!pFIFOIdMaskTable->isExtendedFrame && pFIFOIdMaskTable->isRemoteFrame ){
		*pbuff = 'r';
	}
	pbuff++;
	msg_size++;
	
	sprintf ( (char*)pbuff, "%08x", pFIFOIdMaskTable->MaskId);
	pbuff += 8;
	msg_size += 8;
	
	return msg_size;
}

int8_t convert_filter_ID_to_ASCII(flexcan_id_table_t * pFIFOIdTable, char * pbuff){
	int8_t msg_size = 0;
	if(pFIFOIdTable->isExtendedFrame && !pFIFOIdTable->isRemoteFrame ){
		*pbuff = 'T';
	}
	else if(!pFIFOIdTable->isExtendedFrame && !pFIFOIdTable->isRemoteFrame ){
		*pbuff =  't';
	}
	else if (pFIFOIdTable->isExtendedFrame && pFIFOIdTable->isRemoteFrame ){
		*pbuff = 'R';
	}
	else if (!pFIFOIdTable->isExtendedFrame && pFIFOIdTable->isRemoteFrame ){
		*pbuff = 'r';
	}
	pbuff++;
	msg_size++;

	sprintf ( (char*)pbuff, "%08x", pFIFOIdTable->idFilter);
	pbuff += 8;
	msg_size += 8;

	return msg_size;
}
				 
int8_t convert_flowcontrol_setting_to_ASCII(flowcontrol_t * pflowcontrol, uint8_t i, char * pbuff){
	int8_t msg_size = 0;

	if (pflowcontrol->bisExtended[i]){
		*pbuff = 'F';
		pbuff++;
		msg_size++;
		sprintf ( (char*)pbuff, "%08x", pflowcontrol->msg_id[i]);
		pbuff += 8;
		msg_size += 8;
	}
	else{
		*pbuff = 'f';
		pbuff++;
		msg_size++;
		sprintf ( (char*)pbuff, "%03x", pflowcontrol->msg_id[i]);
		pbuff += 3;
		msg_size += 3;
	}

	_mem_copy(pflowcontrol->p_response[i], (char*)pbuff, (pflowcontrol->resp_size[i] - 1));
	pbuff += pflowcontrol->resp_size[i] - 1;
	msg_size += pflowcontrol->resp_size[i] - 1;

	return msg_size;
}

int8_t get_msg_response(pflexcanInstance_t pinst, get_message_t * msg_req, char * resp, uint8_t resp_max_size){
	int8_t resp_size = 0;
	resp[0] = 'G';
	resp_size++;
	switch (msg_req->message_type){
	case GET_MSG_MASK:
		if ((msg_req->index >= 0) && (msg_req->index < pinst->FIFOMaskTableSize)){
			resp_size += convert_mask_to_ASCII(pinst->pFIFOIdMaskTable + msg_req->index, &resp[1]);
		}
		else{
			resp_size = 0;
		}
		break;
	case GET_MSG_FILTER_CODE:
		if ((msg_req->index >= 0) && (msg_req->index < (pinst->FIFOFilterTableSize/ sizeof(flexcan_id_table_t)))){
			resp_size += convert_filter_ID_to_ASCII(pinst->pFIFOIdFilterTable + msg_req->index, &resp[1]);
		}
		else{
			resp_size = 0;
		}
		break;
	case GET_MSG_AUTO_FLOW:
		if ((msg_req->index >= 0) && (msg_req->index < FLOW_CONTROL_ARR_SIZE)){
			resp_size += convert_flowcontrol_setting_to_ASCII(&(pinst->flowcontrol), msg_req->index, &resp[1]);
		}
		else{
			resp_size = 0;
		}
		break;
	default:
		return -1;
	}
	return resp_size;
}
