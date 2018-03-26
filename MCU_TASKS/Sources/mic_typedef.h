/*
 * mic_typedefs.h
 *
 *  Created on: Nov 17, 2015
 *      Author: ruslans
 */

 #ifndef _MIC_TYPEDEFS_H__
 #define _MIC_TYPEDEFS_H__

 /******************************************************************************
 * Includes - None
 *****************************************************************************/
#include "message.h"
/******************************************************************************
 * Constants - None
 *****************************************************************************/

/*****************************************************************************
  * Types
  ****************************************************************************/
typedef struct cdc_mic_queue_element_struct {
	QUEUE_ELEMENT_STRUCT    HEADER;
	uint32_t                packetNum;
	uint8_t                 send_size;
	uint8_t                data_buff[];
}cdc_mic_queue_element_t, *pcdc_mic_queue_element_t;

typedef enum {
	//USB_NVIC_IRQ_Priority = (BSP_SYSTIMER_ISR_PRIOR + 2),
	CAN_NVIC_IRQ_Priority = 0x03U,
	ADC_NVIC_IRQ_Priority = 0x04U,
	ACC_NVIC_IRQ_Priority = 0x0CU,
	I2C_NVIC_IRQ_Priority = 0x0CU,
	PORT_NVIC_IRQ_Priority = 0x0CU,
	UART_NVIC_IRQ_Priority = 0x0CU,
}NVIC_Priority_table_enum;

enum OTG_ID_CFG{
	//USB_NVIC_IRQ_Priority = (BSP_SYSTIMER_ISR_PRIOR + 2),
	OTG_ID_CFG_FORCE_NONE = 0,
	OTG_ID_CFG_FORCE_MCU_A8 = 1,
	OTG_ID_CFG_FORCE_BYPASS = 2,
	OTG_ID_CFG_FORCE_DISABLE = 3,
};

/******************************************************************************
 * Macro's
 *****************************************************************************/

#define MIC_CDC_USB_1			(0) // ttyACM0 control serial port
#define MIC_CDC_USB_2			(1) // ttyACM1 Accelerometer
#define MIC_CDC_USB_3			(2) // ttyACM2 CAN0
#define MIC_CDC_USB_4			(3) // ttyACM3 CAN1
#define MIC_CDC_USB_5			(4) // ttyACM4 J1708

#define DEBUG_LOG				0
/****************************************************************************
 * Global Variables
 ****************************************************************************/

//extern _pool_id         g_out_message_pool;	// msg pool for send information USB out
//extern _pool_id         g_in_message_pool;	// msg pool for recieve USB information
extern const uint16_t   g_CanCDCPacketsize;

/*****************************************************************************
 * Global Functions
 *****************************************************************************/
extern bool Virtual_Com_MemAlloc( );
extern pcdc_mic_queue_element_t GetUSBWriteBuffer(uint8_t cdcport);
extern bool SetUSBWriteBuffer(pcdc_mic_queue_element_t pcdcBuff, uint8_t cdcport);
extern uint32_t GetUSBFreeBufferCount (uint8_t cdcport);

#endif //~_MIC_TYPEDEFS_H__
