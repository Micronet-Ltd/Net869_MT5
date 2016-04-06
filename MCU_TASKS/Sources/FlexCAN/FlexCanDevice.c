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

#include "board.h"
#include "gpio_pins.h"
//#include "fsl_debug_console.h"
#include "Uart_debugTerminal.h"
#include "tasks_list.h"
#include "FlexCanDevice.h"
#include "mic_typedef.h"

/* Definition */

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
	{ 0, 0, 0, 0x39, 3}, /* 33.33 KHz Not finaly config*/
	{ 4, 7, 1, 0x3B, 3}, /* 50 KHz */
	{ 4, 7, 1, 0x1D, 3}, /* 100 KHz */
	{ 6, 7, 7, 15, 3 },  /* 125 kHz */
	{ 6, 7, 7,  7, 3 },  /* 250 kHz */
	{ 6, 7, 7,  3, 3 },  /* 500 kHz */
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

flexcanInstance_t g_flexcanDeviceInstance[BOARD_CAN_INSTANCE];
pflexcanInstance_t can_Device_0 = &g_flexcanDeviceInstance[BSP_CAN_DEVICE_0];
pflexcanInstance_t can_Device_1 = &g_flexcanDeviceInstance[BSP_CAN_DEVICE_1];

uint32_t g_flexacandeviceSeted = 0;

uint32_t g_flexacandevice_PacketCountRX = 0;
uint32_t g_flexacandevice_PacketCountRX1 = 0;

flexcan_device_bitrate_t    ConvertTimetoBitRate( flexcan_time_segment_t *ptimeSegmentTable,  flexcan_time_segment_t *ptimeSegment );
flexcan_device_status_t     FlexCanDevice_InitInstance( uint8_t instNum, pflexcandevice_initparams_t pinstance_Can );
flexcan_device_status_t		DecodeSendTxMessage ( const char* buff, uint32_t bufflen, pflexcandevice_TX_data_t pTxData );

void FLEXCAN_Tx_Task( uint32_t param ) {
	uint32_t result, can_instance;
	pflexcanInstance_t pinstance;
	_queue_id  msg_qid;
	APPLICATION_MESSAGE_PTR_T msg_ptr;
	flexcan_device_status_t ret;
	LWEVENT_STRUCT      event_ISR;
	flexcandevice_TX_data_t Tx_data;

	flexcandevice_initparams_t initCan;

	if ( BOARD_CAN_INSTANCE <= param ) {
		printf( "CAN_TX thread wrong param %u\n", param );
		return;
	}
	can_instance = param;
	pinstance = &g_flexcanDeviceInstance[can_instance];

	initCan.flexcanMode        = fdFlexCanNormalMode;
	initCan.instanceBitrate    = fdBitrate_125_kHz;
	initCan.is_rx_fifo_needed  = false;
	initCan.max_num_mb         = MAX_MB_NUMBER;
	initCan.num_id_filters     = kFlexCanRxFifoIDFilters_8;
	initCan.RX_queue_num       = RX_FLEXCAN_MSGQ_MESAGES;
	initCan.TX_queue_num       = TX_FLEXCAN_MSGQ_MESAGES;

	if ( BSP_CAN_DEVICE_0 == can_instance ) {
		msg_qid = _msgq_open(CAN1_TX_QUEUE, 0);
	}
	else {
		msg_qid = _msgq_open(CAN2_TX_QUEUE, 0);
	}

	if (MSGQ_NULL_QUEUE_ID == msg_qid)
    {
       printf("\nCould not create a message pool CAN %u _TX_QUEU\n", param);
       _task_block();
    }


	if (MQX_OK != _lwevent_create(&event_ISR, LWEVENT_AUTO_CLEAR)) // Not set auto clean bits
    {
        printf("Make event failed\n");
        return;;// ( kStatus_FLEXCAN_Fail );
    }


	do {
		msg_ptr = _msgq_receive(msg_qid, 1);
        if(NULL == msg_ptr) { _time_delay(1); continue; }

		char* pbuff = (char *)msg_ptr->data;
		int msg_size = msg_ptr->header.SIZE - APP_MESSAGE_NO_ARRAY_SIZE;
		char erroResp = '\r';
		char termination, baudrate;
		do {
			switch ( *pbuff ) {
			case 'S':
				if ( msg_size > 2 ) {
					pbuff++;
					msg_size--;
					baudrate = *pbuff;
					if ( (fdBitrate_125_kHz <= baudrate) && ( fdBitrate_MAX > baudrate ) ) {
						initCan.instanceBitrate = ( flexcan_device_bitrate_t )baudrate;
						printf("Set baudrate command %c \n", baudrate );
					}
					else {
						printf("Error get baudrate command %c \n", baudrate );
						erroResp = '\a';
						_mem_copy ( &erroResp, msg_ptr->data, 1 );
						msg_ptr->header.SIZE = APP_MESSAGE_NO_ARRAY_SIZE + 1;
						break;
					}
					pbuff++;
					msg_size--;
					if ( 0 == msg_size ) {
						_mem_copy ( &erroResp, msg_ptr->data, 1 );
						msg_ptr->header.SIZE = APP_MESSAGE_NO_ARRAY_SIZE + 1;
					}
					break;
				}
			case 'O':
				pbuff++;
				msg_size--;
				
				termination = *pbuff;
				
				if ( '\r' == termination)
				{
					printf ( "Open wrong format to enable termination\n" );
					erroResp = '\a';
					break;					
				}
				pbuff++;
				msg_size--;
				
				if ( BSP_CAN_DEVICE_0 == can_instance ) {
					ret = FlexCanDevice_Init(&initCan, NULL);
				}
				else {
					ret = FlexCanDevice_Init(NULL, &initCan);
				}
				printf("FlexCanDevice_Init( ) return %d\n", ret);

				pinstance->canState.pevent_ISR = &event_ISR;

				ret = FlexCanDevice_Start(pinstance);
				printf("FlexCanDevice_Start( ) return %d\n", ret);

				ret = FlexCanDevice_SetRxMaskType(pinstance, false);
				printf("FlexCanDevice_SetRxMaskType( ) return %d\n", ret);

				//ret = FlexCanDevice_SetRxMaskType(pinstance, true);
				//printf("FlexCanDevice_SetRxMaskType( ) return %d\n", ret);

				//ret = FlexCanDevice_SetRxMbGlobalMask ( pinstance, kFlexCanMsgIdStd , 0x700 );
				//printf("FlexCanDevice_SetRxMbGlobalMask( ) return %d\n", ret);

				pinstance->iScanInstanceStarted = true;
				
				if ( 0 == termination ){
				    ret = FlexCanDevice_SetTermination(pinstance, false);
				}
				else{
					ret = FlexCanDevice_SetTermination(pinstance, true);
				}
				printf("FlexCanDevice_SetTermination( ) return %d on set %d\n", ret, termination);

				_time_delay(20);

				for (int i = 0; i < 14; i++) {
					//ret = FlexCanDevice_SetRxIndividualMask ( pinstance, kFlexCanMsgIdStd, i, 0xF00 );
					//printf("FlexCanDevice_SetRxIndividualMask %d return %d\n", i, ret);
					ret = FlexCanDevice_setMailbox(pinstance, kFlexCanMsgIdStd, i, 0x1c, true);
					printf("FlexCanDevice_setMailbox %d return %d\n", i, ret);
				}
				
				//Enable CAN
				if ( BSP_CAN_DEVICE_0 == can_instance ){
					GPIO_DRV_SetPinOutput(CAN1_J1708_ENABLE);
				}
				else {
					GPIO_DRV_SetPinOutput(CAN2_ENABLE);
				}

				_time_delay(20);				
				break;
			case 'C':
				pbuff++;
				msg_size--;
				//Disable CAN
				pinstance->iScanInstanceStarted = false;
				if ( BSP_CAN_DEVICE_0 == can_instance ) {
					GPIO_DRV_ClearPinOutput(CAN1_J1708_ENABLE);
				}
				else {
					GPIO_DRV_ClearPinOutput(CAN2_ENABLE);
				}

				ret = FlexCanDevice_SetTermination(pinstance, false);
				printf("FlexCanDevice_SetTermination( ) return %d\n", ret);

				ret = FlexCanDevice_Stop(pinstance);
				printf("FlexCanDevice_Stop( ) return %d\n", ret);

				ret = FlexCanDevice_DeInit(pinstance);
				printf("FlexCanDevice_DeInit( ) return %d\n", ret);

				_time_delay(100);
				break;
			case 't':
				printf( "ERROR Command not recognized %c size %d", *pbuff, msg_size  );
				Tx_data.msgbuffType = kFlexCanMsgIdStd;
			case 'T':
				if ( 'T' == *pbuff ) {
					Tx_data.msgbuffType = kFlexCanMsgIdExt;
				}
				if ( ((4 > msg_size) && (kFlexCanMsgIdStd == Tx_data.msgbuffType)) || ((9 > msg_size) && (kFlexCanMsgIdExt == Tx_data.msgbuffType)) )
				{
					_msg_free(msg_ptr);
					msg_ptr = NULL;
					msg_size = 0;
					break;
				}
				pbuff++;
				msg_size--;
				if ( fcStatus_FLEXCAN_Success == DecodeSendTxMessage ( (const char*) pbuff, msg_size, &Tx_data ) )
				{
					if ( fcStatus_FLEXCAN_Success != FlexCanDevice_TxMessage ( pinstance, 14, &Tx_data ) )
						erroResp = '\a';
				}
				else
				{
					erroResp = '\a';
				}
				msg_size = 0;
				break;
			case 'F':
				pbuff++;
				msg_size--;
				break;
			case '\r':
				pbuff++;
				msg_size--;
				break;
			case '\a':
				pbuff++;
				msg_size--;
				break;
			default:
				printf( "ERROR Command not recognized %c size %d", *pbuff, msg_size  );
				_msg_free(msg_ptr);
				msg_ptr = NULL;
				msg_size = 0;
				//erroResp = '\a';
			}

			if ( (0 >= msg_size) || ('\r' != erroResp) ) {
				break;
			}
		}while ( 1 );
		
		if ( msg_ptr ) {			
			msg_ptr->header.SOURCE_QID = msg_qid;
			msg_ptr->header.TARGET_QID = _msgq_get_id(0, USB_QUEUE);
			_mem_copy ( &erroResp, msg_ptr->data, 1 );
			msg_ptr->header.SIZE = APP_MESSAGE_NO_ARRAY_SIZE + 1;
			if ( BSP_CAN_DEVICE_0 == can_instance ) {
				msg_ptr->portNum = MIC_CDC_USB_3;
			}
			else {
				msg_ptr->portNum = MIC_CDC_USB_4;
			}
			_msgq_send (msg_ptr);
		}
		else
        	_time_delay(1);

	} while ( 1 );

	//TODO
	if (MQX_OK != _lwevent_destroy(&(event_ISR)))
	{
		printf("_lwevent_destroy event failed\n");
		return;// ( kStatus_FLEXCAN_Fail );
	}
}

#define MIC_LED_TEST

void FLEXCAN_Rx_Task( uint32_t param ) {

	uint32_t result, can_instance;
	pflexcanInstance_t pinstance;
	_queue_id  msg_qid;
	APPLICATION_MESSAGE_PTR_T msg_ptr;
	char tmp;
	char msg_str[35];
	_mqx_uint err_task;

	if ( BOARD_CAN_INSTANCE <= param ) {
		return;
	}
	can_instance = param;
	
	printf("FLEXCAN_Rx_Task Task: Loop instance %u \n", can_instance);

	if ( BSP_CAN_DEVICE_0 == can_instance ) {
		msg_qid = _msgq_open(CAN1_RX_QUEUE, 0);
	}
	else {
		msg_qid = _msgq_open(CAN2_RX_QUEUE, 0);
	}

	if (MSGQ_NULL_QUEUE_ID == msg_qid)
    {
       printf("\nCould not create a message pool CAN %u _TX_QUEU\n", param);
       _task_block();
    }

	pinstance = &g_flexcanDeviceInstance[can_instance];

	do {
		uint32_t idx, i, j, pmsg_size, curr_msg_len;
		//flexcan_device_msgRX_t msg;
		char *pmsg_data;
		_mqx_uint mRet;
		if ((uint32_t)false == pinstance->iScanInstanceStarted )
		{
			_time_delay(2);
			continue;
		}
		result = FLEXCAN_DRV_GetReceiveStatusBlocking(pinstance->instance, &idx, 0);

		if ( !result ) {
			pmsg_data = NULL;
			pmsg_size = curr_msg_len = 0;
			if ( (msg_ptr = (APPLICATION_MESSAGE_PTR_T) _msg_alloc (g_out_message_pool)) == NULL )
			{
				printf("CAN RX task %u failed allocate msg\n", can_instance);
			}
			if ( msg_ptr ) {
				if ( BSP_CAN_DEVICE_0 == can_instance ) {
					msg_ptr->portNum = MIC_CDC_USB_3;
				}
				else {
					msg_ptr->portNum = MIC_CDC_USB_4;
				}
				msg_ptr->header.SOURCE_QID = msg_qid;
				msg_ptr->header.TARGET_QID = _msgq_get_id( 0, USB_QUEUE );;
				msg_ptr->header.SIZE = APP_MESSAGE_NO_ARRAY_SIZE;

				pmsg_data = ( char* )msg_ptr->data;
			}
			for ( i = 0; idx && (i < MAX_MB_NUMBER); i++ ) {
				j = idx & 0x00000001;
				idx >>= 1;
				if ( 1 == j ) {
#ifdef MIC_LED_TEST
					if ( 0 == pinstance->instance ) {
						GPIO_DRV_SetPinOutput(LED_RED);
					}
					if ( 1 == pinstance->instance ) {
						GPIO_DRV_SetPinOutput(LED_BLUE);
					}
#endif
					if ( pmsg_data ) {
						tmp = (char)(((pinstance->MB_msgbuff[i].cs) >> 16) & 0xF);
						//printf("\r\nDLC=%u, mb_idx=%u", tmp, pinstance->MB_config[i].iD);
						//printf("\r\nID: 0x%x", pinstance->MB_msgbuff[i].msgId);
						//printf("\r\nRX MB data: 0x");

						switch ( tmp ) {
						case 1:
							sprintf ( msg_str, "%x#%02x\r", pinstance->MB_msgbuff[i].msgId, 
									  pinstance->MB_msgbuff[i].data[0]  );
							break;
						case 2:
							sprintf ( msg_str, "%x#%02x%02x\r", pinstance->MB_msgbuff[i].msgId, 
									  pinstance->MB_msgbuff[i].data[0],
									  pinstance->MB_msgbuff[i].data[1] );
							break;
						case 3:
							sprintf ( msg_str, "%x#%02x%02x%02x\r", pinstance->MB_msgbuff[i].msgId, 
									  pinstance->MB_msgbuff[i].data[0],
									  pinstance->MB_msgbuff[i].data[1],
									  pinstance->MB_msgbuff[i].data[2] );
							break;
						case 4:
							sprintf ( msg_str, "%x#%02x%02x%02x%02x\r", pinstance->MB_msgbuff[i].msgId, 
									  pinstance->MB_msgbuff[i].data[0],
									  pinstance->MB_msgbuff[i].data[1],
									  pinstance->MB_msgbuff[i].data[2],
									  pinstance->MB_msgbuff[i].data[3] );
							break;
						case 5:
							sprintf ( msg_str, "%x#%02x%02x%02x%02x%02x\r", pinstance->MB_msgbuff[i].msgId, 
									  pinstance->MB_msgbuff[i].data[0],
									  pinstance->MB_msgbuff[i].data[1],
									  pinstance->MB_msgbuff[i].data[2],
									  pinstance->MB_msgbuff[i].data[3],
									  pinstance->MB_msgbuff[i].data[4] );
							break;
						case 6:
							sprintf ( msg_str, "%x#%02x%02x%02x%02x%02x%02x\r", pinstance->MB_msgbuff[i].msgId, 
									  pinstance->MB_msgbuff[i].data[0],
									  pinstance->MB_msgbuff[i].data[1],
									  pinstance->MB_msgbuff[i].data[2],
									  pinstance->MB_msgbuff[i].data[3],
									  pinstance->MB_msgbuff[i].data[4],
									  pinstance->MB_msgbuff[i].data[5] );
							break;
						case 7:
							sprintf ( msg_str, "%x#%02x%02x%02x%02x%02x%02x%02x\r", pinstance->MB_msgbuff[i].msgId, 
									  pinstance->MB_msgbuff[i].data[0],
									  pinstance->MB_msgbuff[i].data[1],
									  pinstance->MB_msgbuff[i].data[2],
									  pinstance->MB_msgbuff[i].data[3],
									  pinstance->MB_msgbuff[i].data[4],
									  pinstance->MB_msgbuff[i].data[5],
									  pinstance->MB_msgbuff[i].data[6] );
							break;
						case 8:
							sprintf ( msg_str, "%x#%02x%02x%02x%02x%02x%02x%02x%02x\r", pinstance->MB_msgbuff[i].msgId, 
									  pinstance->MB_msgbuff[i].data[0],
									  pinstance->MB_msgbuff[i].data[1],
									  pinstance->MB_msgbuff[i].data[2],
									  pinstance->MB_msgbuff[i].data[3],
									  pinstance->MB_msgbuff[i].data[4],
									  pinstance->MB_msgbuff[i].data[5],
									  pinstance->MB_msgbuff[i].data[6],
									  pinstance->MB_msgbuff[i].data[7] );
							break;
						default:
							sprintf ( msg_str, "%x#\r", pinstance->MB_msgbuff[i].msgId );

						}

//						for ( result = 0; result < tmp; result++ ) {
//							msg.data[result] = pinstance->MB_msgbuff[i].data[result];
//							//printf("%02x ", pinstance->MB_msgbuff[i].data[result]);
//						}
						curr_msg_len = _strnlen( msg_str, sizeof ( msg_str ) );
						printf("CAN%d msg %s size %d\n", can_instance, msg_str, curr_msg_len );
						_mem_copy ( ( const void*)msg_str, ( void* )pmsg_data, curr_msg_len );
						pmsg_size += curr_msg_len;
						pmsg_data += curr_msg_len;
						printf("pmsg_size %d\n", pmsg_size);
					}

					mRet = _mutex_lock(&(pinstance->mutex_MB_sync));

					if ( pinstance->MB_config[i].isEnable ) {
						flexcan_data_info_t rxInfo;

						rxInfo.msg_id_type = pinstance->MB_config[i].iD_type;
						rxInfo.data_length = kFlexCanMessageSize;

						mRet = FLEXCAN_DRV_ConfigRxMb(pinstance->instance, i, &rxInfo, pinstance->MB_config[i].iD_Mask);
						if ( mRet ) {
							//numErrors++;
							printf("\r\nFlexCAN RX MB configuration failed. result: 0x%lx\n", mRet);
						}
						//Enable Interrupt and start recieving
						mRet = FLEXCAN_DRV_RxMessageBuffer(pinstance->instance, pinstance->MB_config[i].iD, &(pinstance->MB_msgbuff[i]));
						if ( mRet ) {
							printf("\r\nFLEXCAN_DRV_RxMessageBuffer. result: 0x%lx\n", mRet);
						}
					}

					g_flexacandevice_PacketCountRX++;
					mRet = _mutex_unlock(&(pinstance->mutex_MB_sync));
				}
			}
		}

		if ( msg_ptr )
		{
			msg_ptr->header.SIZE += pmsg_size;
			_msgq_send (msg_ptr);
			if (MQX_OK != (err_task = _task_get_error()))
			{
				printf("FCD_RX_%u Task: ERROR: message send failed %x\n", can_instance, err_task);
				_task_set_error(MQX_OK);
			}
			msg_ptr = NULL;
		}
		else
			_time_delay(1);

	} while ( 1 );
}

pflexcanInstance_t FlexCanDevice_GetInstance( flexcandevice_module_t moduleID ) {
	if ( fdcandevice_CAN1_MAX > moduleID ) {
		return NULL;
	}

	return &g_flexcanDeviceInstance[moduleID];
}

flexcan_device_status_t FlexCanDevice_InitInstance(  uint8_t instNum, pflexcandevice_initparams_t pinstance_Can ) {
	flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;

	if ( !pinstance_Can || (BOARD_CAN_INSTANCE <= instNum) ) {
		return fcStatus_FLEXCAN_Error;
	}

	_mem_zero((void *)&g_flexcanDeviceInstance[instNum], sizeof(flexcanInstance_t));

	g_flexcanDeviceInstance[instNum].instance                         = (uint32_t)instNum;
	g_flexcanDeviceInstance[instNum].flexcanData.flexcanMode          = (flexcan_operation_modes_t)pinstance_Can->flexcanMode; //kFlexCanDisableMode;
	g_flexcanDeviceInstance[instNum].flexcanData.is_rx_fifo_needed    = pinstance_Can->is_rx_fifo_needed; //false;
	g_flexcanDeviceInstance[instNum].flexcanData.max_num_mb           = pinstance_Can->max_num_mb; //16;
	g_flexcanDeviceInstance[instNum].flexcanData.num_id_filters       = pinstance_Can->num_id_filters; //kFlexCanRxFifoIDFilters_8;
	g_flexcanDeviceInstance[instNum].instanceBitrate                  = pinstance_Can->instanceBitrate;
	g_flexcanDeviceInstance[instNum].initialize                       = true;

	if ( g_flexcanDeviceInstance[instNum].initialize ) {
		ret = (flexcan_device_status_t)_mutex_init(&(g_flexcanDeviceInstance[instNum].mutex_MB_sync), NULL);
		if ( MQX_EOK != ret ) {
			printf("Error init mutex for instance %u - %x\n", instNum, ret);
			g_flexcanDeviceInstance[instNum].initialize = false;
			ret = fcStatus_FLEXCAN_Error;
		}
	}

	return ret;
}

flexcan_device_status_t FlexCanDevice_Init( pflexcandevice_initparams_t pinstance_Can0, pflexcandevice_initparams_t pinstance_Can1 ) {
	flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;

	if ( NULL != pinstance_Can0 ) {
		ret = FlexCanDevice_InitInstance(0, pinstance_Can0);
		if ( 0 > ret ) {
			printf("Error Initialize instance 0\n");
		}
	}

	if ( NULL != pinstance_Can1 ) {
		ret = FlexCanDevice_InitInstance(1, pinstance_Can1);
		if ( 0 > ret ) {
			printf("Error Initialize instance 1\n");
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
		printf(" Error FlexCanDevice_Stop - %x\n", ret);
	}

	if ( MQX_EOK != _mutex_destroy(&(pInstance->mutex_MB_sync)) ) {
		printf(" Error _mutex_destroy\n");
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
		printf("Instance not initialized correct\n");
		return fcStatus_FLEXCAN_Error;
	}

	ret = (flexcan_device_status_t)FLEXCAN_DRV_Init(pInstance->instance, &(pInstance->canState), &(pInstance->flexcanData));
	if ( fcStatus_FLEXCAN_Success < ret ) {
		printf("\r\nFLEXCAN initilization failed. result: 0x%x \n", ret);
		return ret;
	}
	//pInstance->iScanInstanceStarted = (uint32_t)true;

	ret = FlexCanDevice_SetBitrate(pInstance, pInstance->instanceBitrate);

	return ret;
}

flexcan_device_status_t FlexCanDevice_Stop( pflexcanInstance_t pInstance ) {
	flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;

	if ( !pInstance ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	if ( !pInstance->iScanInstanceStarted ) {
		return ret;
	}

	//TODO
	//Clear all before stop HW

	FLEXCAN_DRV_Deinit(pInstance->instance);
	pInstance->iScanInstanceStarted = (uint32_t)false;
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
	printf( "FlexCan clock %u", canPeClk );

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
			printf("\r\nFLEXCAN bitrate table not available for PE clock: %d \n", (int)pInstance->canPeClk);
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

	if ( !pinstance->iScanInstanceStarted ) {
		return fcStatus_FLEXCAN_Error;
	}

	if ( MAX_MB_NUMBER <= id  ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	if ( (pinstance->MB_config[id].iD == id) && (pinstance->MB_config[id].iD_Mask == mask) &&
		 (pinstance->MB_config[id].iD_type == id_type) && (pinstance->MB_config[id].isEnable == enabled) ) {
		return ret;
	}

	//mRet = _mutex_lock(&(pinstance->mutex_MB_sync));

	bprev = pinstance->MB_config[id].isEnable;
	pinstance->MB_config[id].iD         = id;
	pinstance->MB_config[id].iD_Mask    = mask;
	pinstance->MB_config[id].iD_type    = id_type;
	pinstance->MB_config[id].isEnable   = enabled;

	//mRet = _mutex_unlock( &(pinstance->mutex_MB_sync) );

	if ( enabled ) {
		_mutex_lock(&(pinstance->mutex_MB_sync));
		ret = FlexCanDevice_SetRxIndividualMask(pinstance, id_type, id, mask);
		printf("\r\nFlexCAN SetRxIndividualMask ret %u MB %x\n", ret, id);

		if ( bprev != enabled ) {
			rxInfo.msg_id_type = pinstance->MB_config[id].iD_type;
			rxInfo.data_length = kFlexCanMessageSize;

			printf("\r\nFlexCAN MB receive config MB %x\n", id);

			/* Configure RX MB fields*/
			ret = (flexcan_device_status_t)FLEXCAN_DRV_ConfigRxMb(pinstance->instance, id, &rxInfo, pinstance->MB_config[id].iD_Mask);
			if ( ret ) {
				//numErrors++;
				printf("\r\nFlexCAN RX MB configuration failed. result: 0x%lx\n", ret);
			}
			//Enable Interrupt and start recieving
			ret = (flexcan_device_status_t)FLEXCAN_DRV_RxMessageBuffer(pinstance->instance, pinstance->MB_config[id].iD, &(pinstance->MB_msgbuff[id]));
			printf("\r\nFLEXCAN_DRV_RxMessageBuffer. result: 0x%lx \n", ret);
		}
		_mutex_unlock(&(pinstance->mutex_MB_sync));

		return ret;
	}

	_mutex_lock(&(pinstance->mutex_MB_sync));
	ret = FlexCanDevice_SetRxIndividualMask(pinstance, id_type, id, 0);
	_mutex_unlock(&(pinstance->mutex_MB_sync));
	return ret;
}

flexcan_device_status_t FlexCanDevice_TxMessage ( pflexcanInstance_t pinstance, uint32_t MbId, pflexcandevice_TX_data_t pTxData ) {
	flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;
	flexcan_data_info_t txInfo;

	if ( !pinstance || !pTxData ) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}

	if ( pinstance->iScanInstanceStarted ) {
		return fcStatus_FLEXCAN_Error;
	}

	txInfo.msg_id_type = pTxData->msgbuffType;
	txInfo.data_length = pTxData->msgSize;

	ret = (flexcan_device_status_t)FLEXCAN_DRV_ConfigTxMb(pinstance->instance, MbId, &txInfo, pTxData->msgID);
	if ( fcStatus_FLEXCAN_Success == ret ) {
		ret = (flexcan_device_status_t)FLEXCAN_DRV_SendBlocking(pinstance->instance, MbId, &txInfo, pTxData->msgID, pTxData->msgData, FLEXCAN_DEVICE_TX_TIMEOUT);
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

flexcan_device_status_t DecodeSendTxMessage ( const char* buff, uint32_t bufflen, pflexcandevice_TX_data_t pTxData ) {
	flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;
	uint32_t i, msg_limit = 3;
	char *pbuff =  (char*)buff;

	if ( NULL == buff || 0 == bufflen ||  NULL == pTxData) {
		return fcStatus_FLEXCAN_InvalidArgument;
	}
	
	if ( kFlexCanMsgIdExt ==  pTxData->msgbuffType )
		msg_limit = 8;

	for ( i = 0; '\r' != *pbuff && 1 < bufflen; i++ ) {
		if ( msg_limit > i ) {
			pTxData->msgID <<= 4;
			pTxData->msgID |= (char)*pbuff;
			pbuff++;
			continue;
		}
		if ( msg_limit == i ) {
			pTxData->msgSize = (char)*pbuff;
			pbuff++;
			continue;
		}
		pTxData->msgData[i-(msg_limit + 1)] = (char)*pbuff;
	}
	pbuff++;
	return ret;
}


