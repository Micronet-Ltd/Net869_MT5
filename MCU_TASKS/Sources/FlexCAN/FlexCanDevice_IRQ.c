/*
 * FlexCanDevice_IRQ.c
 *
 *  Created on: Nov 17, 2015
 *      Author: ruslans
 */

#include "fsl_flexcan_driver.h"

extern FLEXCAN_Debug_t g_Flexdebug;

#if FSL_FEATURE_SOC_FLEXCAN_COUNT

/*******************************************************************************
 * Code
 ******************************************************************************/
#if (CAN_INSTANCE_COUNT > 0U)
/* Implementation of CAN0 handler named in startup code. */
void CAN0_ORed_Message_buffer_IRQHandler(void)
{
    g_Flexdebug.DataInterrCount_0++;
    FLEXCAN_DRV_IRQHandler(g_flexcanBase[0], g_flexcanStatePtr[0]);
}

/* Implementation of CAN0 handler named in startup code. */
void CAN0_Bus_Off_IRQHandler(void)
{
    g_Flexdebug.BusOffCount_0++;
    FLEXCAN_DRV_IRQHandler(g_flexcanBase[0], g_flexcanStatePtr[0]);
}

/* Implementation of CAN0 handler named in startup code. */
void CAN0_Error_IRQHandler(void)
{
    g_Flexdebug.ErrorCount_0++;
    FLEXCAN_DRV_IRQHandler(g_flexcanBase[0], g_flexcanStatePtr[0]);
}

/* Implementation of CAN0 handler named in startup code. */
void CAN0_Wake_Up_IRQHandler(void)
{
    g_Flexdebug.WakeUpCount_0++;
    FLEXCAN_DRV_IRQHandler(g_flexcanBase[0], g_flexcanStatePtr[0]);
}
#endif

#if (CAN_INSTANCE_COUNT > 1U)
/* Implementation of CAN1 handler named in startup code. */
void CAN1_ORed_Message_buffer_IRQHandler(void)
{
    g_Flexdebug.DataInterrCount_1++;
    FLEXCAN_DRV_IRQHandler(g_flexcanBase[1], g_flexcanStatePtr[1]);
}

/* Implementation of CAN1 handler named in startup code. */
void CAN1_Bus_Off_IRQHandler(void)
{
    g_Flexdebug.BusOffCount_1++;
    FLEXCAN_DRV_IRQHandler(g_flexcanBase[1], g_flexcanStatePtr[1]);
}

/* Implementation of CAN1 handler named in startup code. */
void CAN1_Error_IRQHandler(void)
{
    g_Flexdebug.ErrorCount_1++;
    FLEXCAN_DRV_IRQHandler(g_flexcanBase[1], g_flexcanStatePtr[1]);
}

/* Implementation of CAN1 handler named in startup code. */
void CAN1_Wake_Up_IRQHandler(void)
{
    g_Flexdebug.WakeUpCountCount_1++;
    FLEXCAN_DRV_IRQHandler(g_flexcanBase[1], g_flexcanStatePtr[1]);
}
#endif
#endif


/*******************************************************************************
 * EOF
 ******************************************************************************/


