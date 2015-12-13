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

#include "board.h"
#include "gpio_pins.h"
#include "fsl_debug_console.h"
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
pflexcanInstance_t can_device_1 = &g_flexcanDeviceInstance[BSP_CAN_DEVICE_1];

uint32_t g_flexacandeviceSeted = 0;

flexcan_device_bitrate_t ConvertTimetoBitRate( flexcan_time_segment_t *ptimeSegmentTable,  flexcan_time_segment_t *ptimeSegment );

void FLEXCAN_Tx_Task( uint32_t param) {
}

void FLEXCAN_Rx_Task( uint32_t param ) {

    uint32_t result;
    pflexcanInstance_t pinstance;
    uint32_t i, tmp;

    if ( BOARD_CAN_INSTANCE <= param ) {
        return;
    }

    pinstance = &g_flexcanDeviceInstance[param];

    do {
        flexcan_msgbuff_t rx_mb;
        flexcan_device_msgRX_t msg;
        for ( i = 0; i < MAX_MB_NUMBER; i++ ) {
            if ( pinstance->MB_config[i].isEnable ) {
                result = FLEXCAN_DRV_RxMessageBuffer(pinstance->instance, pinstance->MB_config[i].iD, &rx_mb);
                while ( FLEXCAN_DRV_GetReceiveStatus(pinstance->instance) != kStatus_FLEXCAN_Success ) { _time_delay_ticks(1); }
                if ( !result ) {
                    msg.msgID                   = rx_mb.msgId;
                    msg.mb_inst_id.canInstance  = pinstance->instance;
                    msg.mb_inst_id.mb_id        = pinstance->MB_config[i].iD;

                    tmp = ((rx_mb.cs) >> 16) & 0xF;
                    //PRINTF("\r\nDLC=%d, mb_idx=%d", tmp, pinstance->MB_config[i].iD);
                    ////PRINTF("\r\nID: 0x%x", rx_mb.msgId);
                    //PRINTF("\r\nRX MB data: 0x");

                    for ( result = 0; result < tmp; result++ ) {
                        msg.data[result] = rx_mb.data[result];
                        //PRINTF("%02x ", rx_mb.data[result]);
                    }

                    _lwmsgq_send((void *)pinstance->RX_queue, (_mqx_max_type_ptr)&msg, LWMSGQ_SEND_BLOCK_ON_FULL);
                    _time_delay_ticks(1);
                }
            }
            _time_delay_ticks(1);
        }
    } while ( 1 );
}

pflexcanInstance_t FlexCanDevice_GetInstance( flexcandevice_module_t moduleID ) {
    if ( fdcandevice_CAN1_MAX > moduleID ) {
        return NULL;
    }

    return &g_flexcanDeviceInstance[moduleID];
}

flexcan_device_status_t FlexCanDevice_Init( pflexcandevice_initparams_t pinstance_Can0, pflexcandevice_initparams_t pinstance_Can1 ) {
    flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;

    if ( !pinstance_Can0 || !pinstance_Can1 ) {
        return fcStatus_FLEXCAN_Error;
    }

    memset((void *)g_flexcanDeviceInstance, 0, sizeof(g_flexcanDeviceInstance));

    if ( NULL != pinstance_Can0 ) {
        g_flexcanDeviceInstance[0].instance                         = (uint32_t)fdcandevice_CAN0;
        g_flexcanDeviceInstance[0].flexcanData.flexcanMode          = (flexcan_operation_modes_t)pinstance_Can0->flexcanMode; //kFlexCanDisableMode;
        g_flexcanDeviceInstance[0].flexcanData.is_rx_fifo_needed    = pinstance_Can0->is_rx_fifo_needed; //false;
        g_flexcanDeviceInstance[0].flexcanData.max_num_mb           = pinstance_Can0->max_num_mb; //16;
        g_flexcanDeviceInstance[0].flexcanData.num_id_filters       = pinstance_Can0->num_id_filters; //kFlexCanRxFifoIDFilters_8;
        g_flexcanDeviceInstance[0].instanceBitrate                  = pinstance_Can0->instanceBitrate;
        g_flexcanDeviceInstance[0].initialize                       = (uint32_t)true;
        g_flexcanDeviceInstance[0].RX_queue                         = (uint32_t*)_mem_alloc(sizeof(LWMSGQ_STRUCT)/sizeof(uint32_t) + 
                                                                                            pinstance_Can0->RX_queue_num * sizeof( flexcan_device_msgRX_t ));
        if ( NULL !=  g_flexcanDeviceInstance[0].RX_queue) {
            ret = (flexcan_device_status_t)_lwmsgq_init((void *)g_flexcanDeviceInstance[0].RX_queue, pinstance_Can0->RX_queue_num, sizeof( flexcan_device_msgRX_t ));
        }
        else
            g_flexcanDeviceInstance[0].initialize = (uint32_t)false;
    }

    if ( NULL != pinstance_Can1 ) {
        g_flexcanDeviceInstance[1].instance                         = (uint32_t)fdcandevice_CAN0;
        g_flexcanDeviceInstance[1].flexcanData.flexcanMode          = (flexcan_operation_modes_t)pinstance_Can0->flexcanMode; //kFlexCanDisableMode;
        g_flexcanDeviceInstance[1].flexcanData.is_rx_fifo_needed    = pinstance_Can0->is_rx_fifo_needed; //false;
        g_flexcanDeviceInstance[1].flexcanData.max_num_mb           = pinstance_Can0->max_num_mb; //16;
        g_flexcanDeviceInstance[1].flexcanData.num_id_filters       = pinstance_Can0->num_id_filters; //kFlexCanRxFifoIDFilters_8;
        g_flexcanDeviceInstance[1].instanceBitrate                  = pinstance_Can0->instanceBitrate;
        g_flexcanDeviceInstance[1].initialize                       = (uint32_t)true;
        g_flexcanDeviceInstance[1].RX_queue                         = (uint32_t*)_mem_alloc(sizeof(LWMSGQ_STRUCT)/sizeof(uint32_t) + 
                                                                                            pinstance_Can1->RX_queue_num * sizeof( flexcan_device_msgRX_t ));
        if ( NULL !=  g_flexcanDeviceInstance[1].RX_queue) {
            ret = (flexcan_device_status_t)_lwmsgq_init((void *)g_flexcanDeviceInstance[1].RX_queue, pinstance_Can1->RX_queue_num, sizeof( flexcan_device_msgRX_t ));
        }
        else
            g_flexcanDeviceInstance[1].initialize = (uint32_t)false;
    }

    return ret;
}

flexcan_device_status_t FlexCanDevice_Start ( pflexcanInstance_t pInstance ) {
    flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;

    if ( !pInstance ) {
        return fcStatus_FLEXCAN_InvalidArgument;
    }

    ret = (flexcan_device_status_t)FLEXCAN_DRV_Init(pInstance->instance, &(pInstance->canState), &(pInstance->flexcanData));
    if ( fcStatus_FLEXCAN_Success < ret ) {
        // PRINTF("\r\nFLEXCAN initilization failed. result: 0x%lx", ret);
        return ret;
    }
    pInstance->iScanInstanceStarted = (uint32_t)true;

    //Set devault bitrate 125
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
            // PRINTF("\r\nFLEXCAN bitrate table not available for PE clock: %d", canPeClk);
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
            PRINTF("\r\nFLEXCAN bitrate table not available for PE clock: %d", (int)pInstance->canPeClk);
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

flexcan_device_status_t FlexCanDevice_setMailbox( pflexcanInstance_t pinstance, flexcan_msgbuff_id_type_t id_type, uint32_t id, uint32_t mask, bool enabled ){
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

    pinstance->MB_config[id].iD         = id;
    pinstance->MB_config[id].iD_Mask    = mask;
    pinstance->MB_config[id].iD_type    = id_type;
    pinstance->MB_config[id].isEnable   = enabled;

    if ( enabled ) {
        ret = FlexCanDevice_SetRxIndividualMask ( pinstance, id_type, id, mask );
        return ret;
    }

    ret = FlexCanDevice_SetRxIndividualMask ( pinstance, id_type, id, 0 );
    return ret;
}

flexcan_device_status_t FlexCanDevice_SetTermination( pflexcanInstance_t pinstance, bool isSet ) {
    flexcan_device_status_t ret = fcStatus_FLEXCAN_Success;

    if ( !pinstance ) {
        return fcStatus_FLEXCAN_InvalidArgument;
    }

    //TODO
    //Add here configuration of termination set up
    if ( BSP_CAN_DEVICE_0 == pinstance->instance ) {
        if ( isSet ) {
            GPIO_DRV_ClearPinOutput ( CAN1_TERM_ENABLE );
            return ret;
        }
        GPIO_DRV_SetPinOutput ( CAN1_TERM_ENABLE );
        return ret;
    }

    if ( BSP_CAN_DEVICE_1 == pinstance->instance ) {
        if ( isSet ) {
            GPIO_DRV_ClearPinOutput ( CAN2_TERM_ENABLE );
            return ret;
        }
        GPIO_DRV_SetPinOutput ( CAN2_TERM_ENABLE );
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


