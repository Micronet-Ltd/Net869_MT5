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

#include "board.h"
#include "gpio_pins.h"
//#include "fsl_debug_console.h"
#include "Uart_debugTerminal.h"
#include "tasks_list.h"
#include "FlexCanDevice.h"

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

void FLEXCAN_Tx_Task( uint32_t param ) {
}

#define MIC_LED_TEST

void FLEXCAN_Rx_Task( uint32_t param ) {

	uint32_t result;
	pflexcanInstance_t pinstance;
	uint32_t tmp;

	if ( BOARD_CAN_INSTANCE <= param ) {
		return;
	}

	MIC_DEBUG_UART_PRINTF("FLEXCAN_Rx_Task Task: Loop instance %u \n", param);

	pinstance = &g_flexcanDeviceInstance[param];

	if (MQX_OK != _lwevent_create(&(pinstance->canState.event_ISR), LWEVENT_AUTO_CLEAR)) // Not set auto clean bits
    {
        //printf("Make event failed\n");
        return;;// ( kStatus_FLEXCAN_Fail );
    }

	do {
		uint32_t idx, i, j;
		flexcan_device_msgRX_t msg;
		_mqx_uint mRet;
		result = FLEXCAN_DRV_GetReceiveStatusBlocking(pinstance->instance, &idx, 0);

		if ( !result ) {
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
					msg.msgID                   = pinstance->MB_msgbuff[i].msgId;
					msg.mb_inst_id.canInstance  = pinstance->instance;
					msg.mb_inst_id.mb_id        = pinstance->MB_config[i].iD;

					tmp = ((pinstance->MB_msgbuff[i].cs) >> 16) & 0xF;
					//MIC_DEBUG_UART_PRINTF("\r\nDLC=%u, mb_idx=%u", tmp, pinstance->MB_config[i].iD);
					//MIC_DEBUG_UART_PRINTF("\r\nID: 0x%x", pinstance->MB_msgbuff[i].msgId);
					//MIC_DEBUG_UART_PRINTF("\r\nRX MB data: 0x");

					for ( result = 0; result < tmp; result++ ) {
						msg.data[result] = pinstance->MB_msgbuff[i].data[result];
						//MIC_DEBUG_UART_PRINTF("%02x ", pinstance->MB_msgbuff[i].data[result]);
					}

					mRet = _mutex_lock(&(pinstance->mutex_MB_sync));

					if ( pinstance->MB_config[i].isEnable ) {
						flexcan_data_info_t rxInfo;

						rxInfo.msg_id_type = pinstance->MB_config[i].iD_type;
						rxInfo.data_length = kFlexCanMessageSize;

						mRet = FLEXCAN_DRV_ConfigRxMb(pinstance->instance, i, &rxInfo, pinstance->MB_config[i].iD_Mask);
						if ( mRet ) {
							//numErrors++;
							MIC_DEBUG_UART_PRINTF("\r\nFlexCAN RX MB configuration failed. result: 0x%lx\n", mRet);
						}
						//Enable Interrupt and start recieving
						mRet = FLEXCAN_DRV_RxMessageBuffer(pinstance->instance, pinstance->MB_config[i].iD, &(pinstance->MB_msgbuff[i]));
						if ( mRet ) {
							MIC_DEBUG_UART_PRINTF("\r\nFLEXCAN_DRV_RxMessageBuffer. result: 0x%lx\n", mRet);
						}
					}

					g_flexacandevice_PacketCountRX++;
					mRet = _mutex_unlock(&(pinstance->mutex_MB_sync));

					// Temporary disable need for USB
					//_lwmsgq_send((void *)pinstance->pRX_queue, (_mqx_max_type_ptr)&msg, LWMSGQ_SEND_BLOCK_ON_FULL);
					_time_delay_ticks(1);
				}
			}
		}

	} while ( 1 );

	//TODO
	if (MQX_OK != _lwevent_destroy(&(pinstance->canState.event_ISR)))
	{
		//printf("_lwevent_destroy event failed\n");
		return;// ( kStatus_FLEXCAN_Fail );
	}
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
	g_flexcanDeviceInstance[instNum].pRX_queue                        = (uint32_t *)_mem_alloc(sizeof(LWMSGQ_STRUCT) / sizeof(uint32_t) +
																							   pinstance_Can->RX_queue_num * sizeof(flexcan_device_msgRX_t));

	if ( NULL !=  g_flexcanDeviceInstance[instNum].pRX_queue ) {
		ret = (flexcan_device_status_t)_lwmsgq_init((void *)g_flexcanDeviceInstance[instNum].pRX_queue, pinstance_Can->RX_queue_num, sizeof(flexcan_device_msgRX_t));
		if ( MQX_OK != ret ) {
			MIC_DEBUG_UART_PRINTF("Error _lwmsgq_init - %x\n", ret);
			_mem_free(g_flexcanDeviceInstance[instNum].pRX_queue);
			g_flexcanDeviceInstance[instNum].pRX_queue = NULL;
			g_flexcanDeviceInstance[instNum].initialize = false;
			ret = fcStatus_FLEXCAN_Error;
		}
	} else {
		g_flexcanDeviceInstance[instNum].initialize = false;
		ret = fcStatus_FLEXCAN_Error;
	}

	if ( g_flexcanDeviceInstance[instNum].initialize ) {
		ret = (flexcan_device_status_t)_mutex_init(&(g_flexcanDeviceInstance[instNum].mutex_MB_sync), NULL);
		if ( MQX_EOK != ret ) {
			MIC_DEBUG_UART_PRINTF("Error init mutex for instance %u - %x\n", instNum, ret);

			if ( MQX_OK != _lwmsgq_deinit(&(g_flexcanDeviceInstance[instNum].pRX_queue)) ) {
				MIC_DEBUG_UART_PRINTF("Error _lwmsgq_deinit instance %u\n", instNum);
			}
			_mem_free(g_flexcanDeviceInstance[instNum].pRX_queue);
			g_flexcanDeviceInstance[instNum].pRX_queue = NULL;
			g_flexcanDeviceInstance[instNum].initialize = false;
			ret = fcStatus_FLEXCAN_Error;
		}
	}

	return ret;
}

flexcan_device_status_t FlexCanDevice_Init( pflexcandevice_initparams_t pinstance_Can0, pflexcandevice_initparams_t pinstance_Can1 ) {
	flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;

	ret = FlexCanDevice_InitInstance(0, pinstance_Can0);
	if ( 0 > ret ) {
		MIC_DEBUG_UART_PRINTF("Error Initialize instance 0\n");
	}

	ret = FlexCanDevice_InitInstance(1, pinstance_Can1);
	if ( 0 > ret ) {
		MIC_DEBUG_UART_PRINTF("Error Initialize instance 1\n");
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
		MIC_DEBUG_UART_PRINTF(" Error FlexCanDevice_Stop - %x\n", ret);
	}

	if ( MQX_EOK != _mutex_destroy(&(pInstance->mutex_MB_sync)) ) {
		MIC_DEBUG_UART_PRINTF(" Error _mutex_destroy\n");
	}

	if ( MQX_OK != _lwmsgq_deinit(&(pInstance->pRX_queue)) ) {
		MIC_DEBUG_UART_PRINTF("Error _lwmsgq_deinit instance 1\n");
	}
	if ( NULL != pInstance->pRX_queue ) {
		_mem_free(pInstance->pRX_queue);
		pInstance->pRX_queue = NULL;
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
		MIC_DEBUG_UART_PRINTF("Instance not initialized correct\n");
		return fcStatus_FLEXCAN_Error;
	}

	ret = (flexcan_device_status_t)FLEXCAN_DRV_Init(pInstance->instance, &(pInstance->canState), &(pInstance->flexcanData));
	if ( fcStatus_FLEXCAN_Success < ret ) {
		MIC_DEBUG_UART_PRINTF("\r\nFLEXCAN initilization failed. result: 0x%x \n", ret);
		return ret;
	}
	pInstance->iScanInstanceStarted = (uint32_t)true;

	ret = FlexCanDevice_SetBitrate(pInstance, pInstance->instanceBitrate);

	//Start RX TX Tasks
	pInstance->RX_idTask = _task_create(0, CAN_TASK_RX_0, pInstance->instance);
	if ( pInstance->RX_idTask == MQX_NULL_TASK_ID ) {
		MIC_DEBUG_UART_PRINTF("FlexCanDevice_Start Could not create CAN_TASK_RX for inst %u \n", pInstance->instance);
	}

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
			// MIC_DEBUG_UART_PRINTF("\r\nFLEXCAN bitrate table not available for PE clock: %d", canPeClk);
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

	ret = FLEXCAN_DRV_GetBitrate(pInstance->instance, &instanceTimeSegment);

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
			MIC_DEBUG_UART_PRINTF("\r\nFLEXCAN bitrate table not available for PE clock: %d \n", (int)pInstance->canPeClk);
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
	return -1;
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
		MIC_DEBUG_UART_PRINTF("\r\nFlexCAN SetRxIndividualMask ret %u MB %x\n", ret, id);

		if ( bprev != enabled ) {
			rxInfo.msg_id_type = pinstance->MB_config[id].iD_type;
			rxInfo.data_length = kFlexCanMessageSize;

			MIC_DEBUG_UART_PRINTF("\r\nFlexCAN MB receive config MB %x\n", id);

			/* Configure RX MB fields*/
			ret = FLEXCAN_DRV_ConfigRxMb(pinstance->instance, id, &rxInfo, pinstance->MB_config[id].iD_Mask);
			if ( ret ) {
				//numErrors++;
				MIC_DEBUG_UART_PRINTF("\r\nFlexCAN RX MB configuration failed. result: 0x%lx\n", ret);
			}
			//Enable Interrupt and start recieving
			ret = FLEXCAN_DRV_RxMessageBuffer(pinstance->instance, pinstance->MB_config[id].iD, &(pinstance->MB_msgbuff[id]));
			MIC_DEBUG_UART_PRINTF("\r\nFLEXCAN_DRV_RxMessageBuffer. result: 0x%lx \n", ret);
		}
		_mutex_unlock(&(pinstance->mutex_MB_sync));

		return ret;
	}

	_mutex_lock(&(pinstance->mutex_MB_sync));
	ret = FlexCanDevice_SetRxIndividualMask(pinstance, id_type, id, 0);
	_mutex_unlock(&(pinstance->mutex_MB_sync));
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


