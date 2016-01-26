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

/******************************************************************************
 * Constants - None
 *****************************************************************************/

/******************************************************************************
 * Macro's
 *****************************************************************************/

#define MIC_CDC_USB_1			(1) // ttyACM0 control serial port
#define MIC_CDC_USB_2			(2) // ttyACM1 Accelerometer
#define MIC_CDC_USB_3			(3) // ttyACM2 CAN0
#define MIC_CDC_USB_4			(4) // ttyACM3 CAN1
#define MIC_CDC_USB_5			(5) // ttyACM4 J1708

/****************************************************************************
 * Global Variables
 ****************************************************************************/

extern _pool_id   g_out_message_pool;	// msg pool for send information USB out 
extern _pool_id   g_in_message_pool;	// msg pool for recieve USB information 

#endif //~_MIC_TYPEDEFS_H__

