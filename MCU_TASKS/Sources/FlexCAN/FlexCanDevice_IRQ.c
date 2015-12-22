/*
 * FlexCanDevice_IRQ.c
 *
 *  Created on: Nov 17, 2015
 *      Author: ruslans
 */

 #include "fsl_flexcan_driver.h"
#if FSL_FEATURE_SOC_FLEXCAN_COUNT

/*******************************************************************************
 * Code
 ******************************************************************************/
#if (CAN_INSTANCE_COUNT > 0U)
/* Implementation of CAN0 handler named in startup code. */
void CAN0_ORed_Message_buffer_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(0);
}

/* Implementation of CAN0 handler named in startup code. */
void CAN0_Bus_Off_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(0);
}

/* Implementation of CAN0 handler named in startup code. */
void CAN0_Error_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(0);
}

/* Implementation of CAN0 handler named in startup code. */
void CAN0_Wake_Up_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(0);
}
#endif

#if (CAN_INSTANCE_COUNT > 1U)
/* Implementation of CAN1 handler named in startup code. */
void CAN1_ORed_Message_buffer_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(1);
}

/* Implementation of CAN1 handler named in startup code. */
void CAN1_Bus_Off_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(1);
}

/* Implementation of CAN1 handler named in startup code. */
void CAN1_Error_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(1);
}

/* Implementation of CAN1 handler named in startup code. */
void CAN1_Wake_Up_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(1);
}
#endif
#endif


/*******************************************************************************
 * EOF
 ******************************************************************************/


