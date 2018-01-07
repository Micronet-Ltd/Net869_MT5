/**HEADER********************************************************************
 *
 * Copyright (c) 2008, 2013 - 2015 Freescale Semiconductor;
 * All Rights Reserved
 *
 * Copyright (c) 1989-2008 ARC International;
 * All Rights Reserved
 *
 ***************************************************************************
 *
 * THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************
 *
 * $FileName: virtual_com.c$
 * $Version :
 * $Date    :
 *
 * Comments:
 *
 * @brief  The file emulates a USB PORT as RS232 PORT.
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device_stack_interface.h"
#include "usb_class_composite.h"
#include "usb_composite.h"
#include "usb_descriptor.h"

////
#include "fsl_device_registers.h"
#include "fsl_clock_manager.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_port_hal.h"

#include <stdio.h>
#include <stdlib.h>
#include "board.h"
/////

/*Micronet Includes */
#include "tasks_list.h"

#include "frame.h"

#include "mic_typedef.h"

#include "virtual_com.h"
#include "main_tasks.h"

/*****************************************************************************
 * Constant and Macro's - None
 *****************************************************************************/
//#define USB_MSGQ_MAX_POOL_SIZE      20

#define USB_CDC_0_OUT_BUFFERS_COUNT 5 // Command interface
#define USB_CDC_1_OUT_BUFFERS_COUNT 5 // Accelerometer interface
#define USB_CDC_2_OUT_BUFFERS_COUNT 20 // CAN0 interface
#define USB_CDC_3_OUT_BUFFERS_COUNT 20 // CAN1 interface
#define USB_CDC_4_OUT_BUFFERS_COUNT 10 // J1708 interface

#define MIC_USB_CONTROL_CHANNEL_BUFF_SIZE       (DATA_BUFF_SIZE+2)
#define MIC_USB_ACCELOROMETER_CHANNEL_BUFF_SIZE (MIC_USB_FRAME_BUFFER_SIZE)
#define MIC_USB_CAN_CHANNEL_BUFF_SIZE           (DATA_BUFF_SIZE<<2)
#define MIC_USB_J1708_CHANNEL_BUFF_SIZE         (DATA_BUFF_SIZE>>1) //????? Need to check the MAX packet size

#define MIC_USB_CDC0_BUFF_ALLOC_SIZE            (sizeof(cdc_mic_queue_element_t)+MIC_USB_CONTROL_CHANNEL_BUFF_SIZE)
#define MIC_USB_CDC1_BUFF_ALLOC_SIZE            (sizeof(cdc_mic_queue_element_t)+MIC_USB_ACCELOROMETER_CHANNEL_BUFF_SIZE)
#define MIC_USB_CDC2_BUFF_ALLOC_SIZE            (sizeof(cdc_mic_queue_element_t)+MIC_USB_CAN_CHANNEL_BUFF_SIZE)
#define MIC_USB_CDC3_BUFF_ALLOC_SIZE            (MIC_USB_CDC2_BUFF_ALLOC_SIZE)
#define MIC_USB_CDC4_BUFF_ALLOC_SIZE            (sizeof(cdc_mic_queue_element_t)+MIC_USB_J1708_CHANNEL_BUFF_SIZE)

const uint16_t   g_CanCDCPacketsize = MIC_USB_CAN_CHANNEL_BUFF_SIZE;

//#define MIC_USB_DEBUG
/*****************************************************************************
 * Global Functions Prototypes
 *****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/
extern usb_desc_request_notify_struct_t desc_callback;
app_composite_device_struct_t g_app_composite_device;
extern uint8_t USB_Desc_Set_Speed(uint32_t handle, uint16_t speed);

extern _pool_id         g_in_message_pool;	// msg pool for recieve USB information

/*****************************************************************************
 * Local Types - None
 *****************************************************************************/

/*****************************************************************************
 * Local Functions Prototypes
 *****************************************************************************/
void USB_App_Device_Callback(uint8_t event_type, void* val, void* arg);
uint8_t USB_App_Class_Callback(uint8_t event, uint16_t value, uint8_t ** data, uint32_t* size, void* arg);

static void USB_Recive_Data ( cdc_struct_t *handle );

static void CDC0_resv ( cdc_struct_t *handle );
static void CDC1_resv ( cdc_struct_t *handle );
static void CDC2_resv ( cdc_struct_t *handle );
static void CDC3_resv ( cdc_struct_t *handle );
static void CDC4_resv ( cdc_struct_t *handle );

static bool CDC_Queue_init (uint8_t cdcNum, uint8_t* memAdr, uint32_t ElemCount, uint32_t queueElemSize);

static bool CDC_SendData (cdc_handle_t handle, cdc_struct_t *phandle );


/*****************************************************************************
 * Local Variables
 *****************************************************************************/
static uint16_t g_cdc_device_speed;
static uint16_t g_bulk_out_max_packet_size;
static uint16_t g_bulk_in_max_packet_size;

/*****************************************************************************
 * Global Functions
 *****************************************************************************/
bool Virtual_Com_MemAlloc( )
{
	//This function should be call once on MCU power up
	_mem_zero((void*)&g_app_composite_device, sizeof(app_composite_device_struct_t));

	g_app_composite_device.DataBuffSize = (MIC_USB_CDC0_BUFF_ALLOC_SIZE * USB_CDC_0_OUT_BUFFERS_COUNT)
#if MIC_USB_CDC_INF_COUNT > 1
										+ (MIC_USB_CDC1_BUFF_ALLOC_SIZE * USB_CDC_1_OUT_BUFFERS_COUNT)
#endif
#if MIC_USB_CDC_INF_COUNT > 2
										+ (MIC_USB_CDC2_BUFF_ALLOC_SIZE * USB_CDC_2_OUT_BUFFERS_COUNT)
#endif
#if MIC_USB_CDC_INF_COUNT > 3
										+ (MIC_USB_CDC3_BUFF_ALLOC_SIZE * USB_CDC_3_OUT_BUFFERS_COUNT)
#endif
#if MIC_USB_CDC_INF_COUNT > 4
										+ (MIC_USB_CDC4_BUFF_ALLOC_SIZE * USB_CDC_4_OUT_BUFFERS_COUNT);
#endif
	;

	g_app_composite_device.pDataBuff = (uint8_t*)_mem_alloc(g_app_composite_device.DataBuffSize);
	if (NULL == g_app_composite_device.pDataBuff) {
		printf("ERROR allocate USB out buffers s=%d\n", g_app_composite_device.DataBuffSize);
		return false;
	}
	printf("USB Out buffers allocated s=%d\n", g_app_composite_device.DataBuffSize);

	if ( MQX_OK != _lwsem_create(&(g_app_composite_device.SendReadySem), 0) )
	{
		printf("ERROR USB sema\n");
		return false;
	}
	return true;
}

pcdc_mic_queue_element_t GetUSBWriteBuffer(uint8_t cdcport) {
	pcdc_mic_queue_element_t pqueueElem;

	if (COMPOSITE_CFG_MAX <= cdcport) {
		printf("%s:ERROR the USB port %d wrong\n", __func__, cdcport);
		return NULL;
	}

	pqueueElem = (pcdc_mic_queue_element_t)_queue_dequeue(&(g_app_composite_device.cdc_vcom[cdcport].qs_OutFreeMsg));
	if (NULL == pqueueElem) {
		if (true == g_app_composite_device.cdc_vcom[cdcport].start_app && true == g_app_composite_device.cdc_vcom[cdcport].start_transactions) {
			//printf("CDC%d no Free buff drop ready\n", cdcport);
		}

		pqueueElem = (pcdc_mic_queue_element_t)_queue_dequeue(&(g_app_composite_device.cdc_vcom[cdcport].qs_OutInProcMsg));
		if (NULL == pqueueElem) {
			//printf("CDC%d ERROR get used buffer\n", cdcport);
		}
	}

	return pqueueElem;
}

bool SetUSBWriteBuffer(pcdc_mic_queue_element_t pcdcBuff, uint8_t cdcport) {

	if ((NULL == pcdcBuff) || (COMPOSITE_CFG_MAX <= cdcport)) {
		printf("%s:ERROR parameters wrong\n", __func__);
		return false;
	}

	if (0 != pcdcBuff->send_size) {
		if ( !_queue_enqueue((QUEUE_STRUCT_PTR)&(g_app_composite_device.cdc_vcom[cdcport].qs_OutInProcMsg), (QUEUE_ELEMENT_STRUCT_PTR)pcdcBuff ) ) {
			printf("%s:ERROR add queue elem cdc%d, addr %x\n", __func__, cdcport, (uint32_t)pcdcBuff);
			return false;
		}

		if (MQX_OK != _lwsem_post(&(g_app_composite_device.SendReadySem))) {
			printf("%s:ERROR set sem for cdc%d\n", __func__, cdcport);
			return false;
		}
	}
	else {
		if ( !_queue_enqueue((QUEUE_STRUCT_PTR)&(g_app_composite_device.cdc_vcom[cdcport].qs_OutFreeMsg), (QUEUE_ELEMENT_STRUCT_PTR)pcdcBuff ) ) {
			printf("%s:ERROR add to free queue elem cdc%d, addr %x\n", __func__, cdcport, (uint32_t)pcdcBuff);
			return false;
		}
	}


	return true;
}

uint32_t GetUSBFreeBufferCount (uint8_t cdcport) {

	if (COMPOSITE_CFG_MAX <= cdcport) {
		printf("%s:ERROR the USB port %d wrong\n", __func__, cdcport);
		return NULL;
	}

	return (uint32_t)(_queue_get_size(&(g_app_composite_device.cdc_vcom[cdcport].qs_OutFreeMsg)));
}

/*****************************************************************************
 * Local Functions
 *****************************************************************************/

/**************************************************************************//*!
 *
 * @name  USB_Get_Line_Coding
 *
 * @brief The function returns the Line Coding/Configuration
 *
 * @param handle:        handle
 * @param interface:     interface number
 * @param coding_data:   output line coding data
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Get_Line_Coding(uint32_t handle,
	uint8_t interface,
	uint8_t * *coding_data)
{
	cdc_struct_t *phandle = (cdc_struct_t *) handle;

	//UNUSED_ARGUMENT(handle)

	/* if interface valid */
	if (interface < USB_MAX_SUPPORTED_INTERFACES)
	{
		/* get line coding data*/
		*coding_data = phandle->line_coding;
		return USB_OK;
	}

	return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Set_Line_Coding
 *
 * @brief The function sets the Line Coding/Configuration
 *
 * @param handle: handle
 * @param interface:     interface number
 * @param coding_data:   output line coding data
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Set_Line_Coding(uint32_t handle,
	uint8_t interface,
	uint8_t * *coding_data)
{
	uint8_t count;
	cdc_struct_t *phandle = (cdc_struct_t *) handle;

	//UNUSED_ARGUMENT(handle)

	/* if interface valid */
	if (interface < USB_MAX_SUPPORTED_INTERFACES)
	{
		/* set line coding data*/
		for (count = 0; count < LINE_CODING_SIZE; count++)
		{
			phandle->line_coding[count] = *((*coding_data + USB_SETUP_PKT_SIZE) + count);
		}
		return USB_OK;
	}

	return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Get_Abstract_State
 *
 * @brief The function gets the current setting for communication feature
 *                                                  (ABSTRACT_STATE)
 * @param handle:        handle
 * @param interface:     interface number
 * @param feature_data:   output comm feature data
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Get_Abstract_State(uint32_t handle,
	uint8_t interface,
	uint8_t * *feature_data)
{
	cdc_struct_t *phandle = (cdc_struct_t *) handle;

	//UNUSED_ARGUMENT(handle)

	/* if interface valid */
	if (interface < USB_MAX_SUPPORTED_INTERFACES)
	{
		/* get line coding data*/
		*feature_data = phandle->abstract_state;
		return USB_OK;
	}

	return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Get_Country_Setting
 *
 * @brief The function gets the current setting for communication feature
 *                                                  (COUNTRY_CODE)
 * @param handle:        handle
 * @param interface:     interface number
 * @param feature_data:   output comm feature data
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Get_Country_Setting(uint32_t handle,
	uint8_t interface,
	uint8_t * *feature_data)
{
	cdc_struct_t *phandle = (cdc_struct_t *) handle;

	//UNUSED_ARGUMENT(handle)

	/* if interface valid */
	if (interface < USB_MAX_SUPPORTED_INTERFACES)
	{
		/* get line coding data*/
		*feature_data = phandle->country_code;
		return USB_OK;
	}

	return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Set_Abstract_State
 *
 * @brief The function gets the current setting for communication feature
 *                                                  (ABSTRACT_STATE)
 * @param handle:        handle
 * @param interface:     interface number
 * @param feature_data:   output comm feature data
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Set_Abstract_State(uint32_t handle,
	uint8_t interface,
	uint8_t * *feature_data)
{
	uint8_t count;
	cdc_struct_t *phandle = (cdc_struct_t *) handle;

	//UNUSED_ARGUMENT(handle)

	/* if interface valid */
	if (interface < USB_MAX_SUPPORTED_INTERFACES)
	{
		/* set Abstract State Feature*/
		for (count = 0; count < COMM_FEATURE_DATA_SIZE; count++)
		{
			phandle->abstract_state[count] = *(*feature_data + count);
		}
		return USB_OK;
	}

	return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Set_Country_Setting
 *
 * @brief The function gets the current setting for communication feature
 *                                                  (COUNTRY_CODE)
 * @param handle: handle
 * @param interface:     interface number
 * @param feature_data:   output comm feature data
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Set_Country_Setting(uint32_t handle,
	uint8_t interface,
	uint8_t * *feature_data)
{
	uint8_t count;
	cdc_struct_t *phandle = (cdc_struct_t *) handle;

	//UNUSED_ARGUMENT (handle)

	/* if interface valid */
	if (interface < USB_MAX_SUPPORTED_INTERFACES)
	{
		for (count = 0; count < COMM_FEATURE_DATA_SIZE; count++)
		{
			phandle->country_code[count] = *(*feature_data + count);
		}
		return USB_OK;
	}

	return USBERR_INVALID_REQ_TYPE;
}

/*****************************************************************************
 *
 *     @name          cdc_vcom_preinit
 *
 *     @brief       This function pre-initializes the App.
 *
 *     @param       None
 *
 *     @return      None
 **
 *****************************************************************************/
void cdc_vcom_preinit(cdc_struct_t* param)
{
	if (NULL == param)
	{
		return;
	}
	param->recv_size = 0;
}
/*****************************************************************************
 *
 *    @name         APP_init
 *
 *    @brief         This function do initialization for APP.
 *
 *    @param         None
 *
 *    @return       true on success or false
 **
 *****************************************************************************/
bool APP_init(void)
{
	uint8_t i, j;
	uint8_t* buff_ptr;
	uint8_t l_line_coding[LINE_CODING_SIZE] = {
											/*e.g. 0x00,0x10,0x0E,0x00 : 0x000E1000 is 921600 bits per second */
											(LINE_CODE_DTERATE_IFACE >> 0) & 0x000000FF,
											(LINE_CODE_DTERATE_IFACE >> 8) & 0x000000FF,
											(LINE_CODE_DTERATE_IFACE >> 16) & 0x000000FF,
											(LINE_CODE_DTERATE_IFACE >> 24) & 0x000000FF,
											LINE_CODE_CHARFORMAT_IFACE,
											LINE_CODE_PARITYTYPE_IFACE,
											LINE_CODE_DATABITS_IFACE };

	uint8_t l_abstract_state[COMM_FEATURE_DATA_SIZE] = {
														(STATUS_ABSTRACT_STATE_IFACE >> 0) & 0x00FF,
														(STATUS_ABSTRACT_STATE_IFACE >> 8) & 0x00FF };

	uint8_t l_country_code[COMM_FEATURE_DATA_SIZE] = {
														(COUNTRY_SETTING_IFACE >> 0) & 0x00FF,
														(COUNTRY_SETTING_IFACE >> 8) & 0x00FF };

	USB_prepare_descroptors (  );
	USB_init_memory_Desc (  );
	class_config_struct_t* cdc_vcom_config_callback_handle;

	buff_ptr = g_app_composite_device.pDataBuff;
	for (i = 0; i < MIC_USB_CDC_INF_COUNT; i++)
	{
		cdc_vcom_config_callback_handle = &g_app_composite_device.composite_device_config_list[i];
		cdc_vcom_config_callback_handle->composite_application_callback.callback = USB_App_Device_Callback;
		cdc_vcom_config_callback_handle->composite_application_callback.arg = &g_app_composite_device.cdc_vcom[i];
		cdc_vcom_config_callback_handle->class_specific_callback.callback = (usb_class_specific_handler_func)USB_App_Class_Callback;
		cdc_vcom_config_callback_handle->class_specific_callback.arg = &g_app_composite_device.cdc_vcom[i];
		cdc_vcom_config_callback_handle->board_init_callback.callback = usb_device_board_init;
		cdc_vcom_config_callback_handle->board_init_callback.arg = CONTROLLER_ID;
		cdc_vcom_config_callback_handle->desc_callback_ptr = &desc_callback;
		cdc_vcom_config_callback_handle->type = USB_CLASS_COMMUNICATION;

		_mem_copy( (void*)( l_line_coding ), (void*)( g_app_composite_device.cdc_vcom[i].line_coding ), sizeof ( l_line_coding ) );
		_mem_copy( (void*)( l_abstract_state ), (void*)( g_app_composite_device.cdc_vcom[i].abstract_state ), sizeof ( l_abstract_state ) );
		_mem_copy( (void*)( l_country_code ), (void*)( g_app_composite_device.cdc_vcom[i].country_code ), sizeof ( l_country_code ) );

		g_app_composite_device.cdc_vcom[i].send_ready   = FALSE;
		g_app_composite_device.cdc_vcom[i].portNum      = i;
		g_app_composite_device.cdc_vcom[i].recv_size	= 0;
		g_app_composite_device.cdc_vcom[i].sendPackets  = 0;
		g_app_composite_device.cdc_vcom[i].SendPacketsCompl = 0;

		_queue_init(&(g_app_composite_device.cdc_vcom[i].qs_OutFreeMsg), 0);
		_queue_init(&(g_app_composite_device.cdc_vcom[i].qs_OutInProcMsg), 0);

		switch (i) {
		case 0:
			if (!CDC_Queue_init (i, buff_ptr, USB_CDC_0_OUT_BUFFERS_COUNT, MIC_USB_CDC0_BUFF_ALLOC_SIZE )) {
				printf("ERROR start CDC%d\n", i);
				return false;
			}
			buff_ptr += MIC_USB_CDC0_BUFF_ALLOC_SIZE * USB_CDC_0_OUT_BUFFERS_COUNT;
			break;
		case 1:
#if MIC_USB_CDC_INF_COUNT > 1
			if (!CDC_Queue_init (i, buff_ptr, USB_CDC_1_OUT_BUFFERS_COUNT, MIC_USB_CDC1_BUFF_ALLOC_SIZE )) {
				printf("ERROR start CDC%d\n", i);
				return false;
			}
			buff_ptr += MIC_USB_CDC1_BUFF_ALLOC_SIZE * USB_CDC_1_OUT_BUFFERS_COUNT;
#endif
			break;
		case 2:
#if MIC_USB_CDC_INF_COUNT > 2
			if (!CDC_Queue_init (i, buff_ptr, USB_CDC_2_OUT_BUFFERS_COUNT, MIC_USB_CDC2_BUFF_ALLOC_SIZE )) {
				printf("ERROR start CDC%d\n", i);
				return false;
			}
			buff_ptr += MIC_USB_CDC2_BUFF_ALLOC_SIZE * USB_CDC_2_OUT_BUFFERS_COUNT;
#endif
			break;
		case 3:
#if MIC_USB_CDC_INF_COUNT > 3
			if (!CDC_Queue_init (i, buff_ptr, USB_CDC_3_OUT_BUFFERS_COUNT, MIC_USB_CDC3_BUFF_ALLOC_SIZE )) {
				printf("ERROR start CDC%d\n", i);
				return false;
			}
			buff_ptr += MIC_USB_CDC3_BUFF_ALLOC_SIZE * USB_CDC_3_OUT_BUFFERS_COUNT;
#endif
			break;
		case 4:
#if MIC_USB_CDC_INF_COUNT > 4
			if (!CDC_Queue_init (i, buff_ptr, USB_CDC_4_OUT_BUFFERS_COUNT, MIC_USB_CDC4_BUFF_ALLOC_SIZE )) {
				printf("ERROR start CDC%d\n", i);
				return false;
			}
			buff_ptr += MIC_USB_CDC4_BUFF_ALLOC_SIZE * USB_CDC_4_OUT_BUFFERS_COUNT;
#endif
			break;
		}
	}

	g_cdc_device_speed = USB_SPEED_FULL;
	g_bulk_out_max_packet_size = FS_DIC_BULK_OUT_ENDP_PACKET_SIZE;
	g_bulk_in_max_packet_size = FS_DIC_BULK_IN_ENDP_PACKET_SIZE;

	g_app_composite_device.composite_device_config_callback.count = MIC_USB_CDC_INF_COUNT;
	g_app_composite_device.composite_device_config_callback.class_app_callback = g_app_composite_device.composite_device_config_list;

	/* Initialize the USB interface */
	USB_Composite_Init(CONTROLLER_ID, &g_app_composite_device.composite_device_config_callback, &g_app_composite_device.composite_device);

	g_app_composite_device.cdc_vcom[0].in_endpoint  = DIC1_BULK_IN_ENDPOINT;
	g_app_composite_device.cdc_vcom[0].out_endpoint = DIC1_BULK_OUT_ENDPOINT;
	g_app_composite_device.cdc_vcom[0].cdc_handle   = (cdc_handle_t)g_app_composite_device.composite_device_config_list[0].class_handle;
#if MIC_USB_CDC_INF_COUNT > 1
	g_app_composite_device.cdc_vcom[1].in_endpoint  = DIC2_BULK_IN_ENDPOINT;
	g_app_composite_device.cdc_vcom[1].out_endpoint = DIC2_BULK_OUT_ENDPOINT;
	g_app_composite_device.cdc_vcom[1].cdc_handle   = (cdc_handle_t)g_app_composite_device.composite_device_config_list[1].class_handle;
#endif
#if MIC_USB_CDC_INF_COUNT > 2
	g_app_composite_device.cdc_vcom[2].in_endpoint  = DIC3_BULK_IN_ENDPOINT;
	g_app_composite_device.cdc_vcom[2].out_endpoint = DIC3_BULK_OUT_ENDPOINT;
	g_app_composite_device.cdc_vcom[2].cdc_handle   = (cdc_handle_t)g_app_composite_device.composite_device_config_list[2].class_handle;
#endif
#if MIC_USB_CDC_INF_COUNT > 3
	g_app_composite_device.cdc_vcom[3].in_endpoint  = DIC4_BULK_IN_ENDPOINT;
	g_app_composite_device.cdc_vcom[3].out_endpoint = DIC4_BULK_OUT_ENDPOINT;
	g_app_composite_device.cdc_vcom[3].cdc_handle   = (cdc_handle_t)g_app_composite_device.composite_device_config_list[3].class_handle;
#endif
#if MIC_USB_CDC_INF_COUNT > 4
	g_app_composite_device.cdc_vcom[4].in_endpoint  = DIC5_BULK_IN_ENDPOINT;
	g_app_composite_device.cdc_vcom[4].out_endpoint = DIC5_BULK_OUT_ENDPOINT;
	g_app_composite_device.cdc_vcom[4].cdc_handle   = (cdc_handle_t)g_app_composite_device.composite_device_config_list[4].class_handle;
#endif

	return true;
}



/******************************************************************************
 *
 *    @name        USB_App_Device_Callback
 *
 *    @brief       This function handles the callback
 *
 *    @param       handle : handle to Identify the controller
 *    @param       event_type : value of the event
 *    @param       val : gives the configuration value
 *
 *    @return      None
 *
 *****************************************************************************/
void USB_App_Device_Callback(uint8_t event_type, void* val, void* arg)
{
	cdc_struct_t *phandle = (cdc_struct_t *) arg;
	uint32_t handle = *((uint32_t *) arg);

	if (event_type == USB_DEV_EVENT_BUS_RESET)
	{
		phandle->start_app = FALSE;
		if (USB_OK == USB_Class_CDC_Get_Speed(handle, &g_cdc_device_speed))
		{
			USB_Desc_Set_Speed(handle, g_cdc_device_speed);
			if (USB_SPEED_HIGH == g_cdc_device_speed)
			{
				g_bulk_out_max_packet_size = HS_DIC_BULK_OUT_ENDP_PACKET_SIZE;
				g_bulk_in_max_packet_size = HS_DIC_BULK_IN_ENDP_PACKET_SIZE;
			}
			else
			{
				g_bulk_out_max_packet_size = FS_DIC_BULK_OUT_ENDP_PACKET_SIZE;
				g_bulk_in_max_packet_size = FS_DIC_BULK_IN_ENDP_PACKET_SIZE;
			}
		}
	}
	else if (event_type == USB_DEV_EVENT_CONFIG_CHANGED)
	{
		/* Schedule buffer for receive */
		USB_Class_CDC_Recv_Data(handle, phandle->out_endpoint , phandle->curr_recv_buf, g_bulk_out_max_packet_size);
		phandle->start_app = TRUE;
	}
	else if (event_type == USB_DEV_EVENT_ERROR)
	{
		/* add user code for error handling */
		printf("ERROR USB get USB_DEV_EVENT_ERROR\n\n");
	}
	return;
}

/******************************************************************************
 *
 *    @name        USB_App_Class_Callback
 *
 *    @brief       This function handles the callback for Get/Set report req
 *
 *    @param       request  :  request type
 *    @param       value    :  give report type and id
 *    @param       data     :  pointer to the data
 *    @param       size     :  size of the transfer
 *
 *    @return      status
 *                  USB_OK  :  if successful
 *                  else return error
 *
 *****************************************************************************/
extern int g_otg_ctl_port_active;
uint8_t USB_App_Class_Callback
(
	uint8_t event,
	uint16_t value,
	uint8_t ** data,
	uint32_t* size,
	void* arg
)
{
	cdc_handle_t handle = *((cdc_handle_t *) arg);
	uint8_t error = USB_OK;
	cdc_struct_t *phandle = (cdc_struct_t *) arg;

	switch(event)
	{
	case GET_LINE_CODING:
		error = USB_Get_Line_Coding((uint32_t)phandle, value, data);
		break;
	case GET_ABSTRACT_STATE:
		error = USB_Get_Abstract_State((uint32_t)phandle, value, data);
		break;
	case GET_COUNTRY_SETTING:
		error = USB_Get_Country_Setting((uint32_t)phandle, value, data);
		break;
	case SET_LINE_CODING:
		error = USB_Set_Line_Coding((uint32_t)phandle, value, data);
		break;
	case SET_ABSTRACT_STATE:
		error = USB_Set_Abstract_State((uint32_t)phandle, value, data);
		break;
	case SET_COUNTRY_SETTING:
		error = USB_Set_Country_Setting((uint32_t)phandle, value, data);
		break;
	case USB_APP_CDC_DTE_ACTIVATED:
		if (phandle->start_app == TRUE)
		{
			printf("Port%d activated\n", phandle->portNum);
			if (0 == phandle->portNum)
				g_otg_ctl_port_active = 1;
			phandle->start_transactions = TRUE;
			phandle->send_ready = TRUE;
		}
		break;
	case USB_APP_CDC_DTE_DEACTIVATED:
		if (phandle->start_app == TRUE)
		{
			printf("Port%d deactivated\n", phandle->portNum);
			if (0 == phandle->portNum)
				g_otg_ctl_port_active = 0;
			phandle->start_transactions = FALSE;
			phandle->send_ready = FALSE;
		}
		break;
	case USB_DEV_EVENT_DATA_RECEIVED:
		if ((phandle->start_app == TRUE) && (phandle->start_transactions == TRUE)) {
//			uint64_t current_time;
			phandle->recv_size = *size;

			if ((0 != phandle->recv_size) && (0xFFFFFFFF != phandle->recv_size))
			{
				USB_Recive_Data ( phandle );
				/* Schedule buffer for next receive event */
				USB_Class_CDC_Recv_Data(handle, phandle->out_endpoint, phandle->curr_recv_buf, g_bulk_out_max_packet_size);

//				current_time = ms_from_start();
//				printf("%s: %llu ms\n", __func__, current_time);
			}
		}
		break;
	case USB_DEV_EVENT_SEND_COMPLETE:
		if ((size != NULL) && (*size != 0) && (!(*size % g_bulk_in_max_packet_size)))
		{
			//printf("%s: EvComp2 %x\n\n", __func__, (uint32_t)(phandle->pSendElem) );
			/* If the last packet is the size of endpoint, then send also zero-ended packet,
			 ** meaning that we want to inform the host that we do not have any additional
			 ** data, so it can flush the output.
			 */
			USB_Class_CDC_Send_Data(handle, phandle->in_endpoint, NULL, 0);
		}
		else if ((phandle->start_app == TRUE) && (phandle->start_transactions == TRUE))
		{
			if ((*data != NULL) || ((*data == NULL) && (*size == 0))) {

				if (NULL != phandle->pSendElem) {
					if (!_queue_enqueue(&(phandle->qs_OutFreeMsg), (QUEUE_ELEMENT_STRUCT_PTR)(phandle->pSendElem))) {
							printf("%s: Error add to free queue port%d mem %x\n", phandle->portNum, (uint32_t)(phandle->pSendElem));
						}
						phandle->pSendElem = NULL;
					}
				phandle->send_ready = TRUE;
				phandle->SendPacketsCompl++;
			}
			else {
				if (NULL != phandle->pSendElem)
					printf ("stop1\n");
			}
		}
		else {
			printf("%s: EvComp5 %x\n\n", __func__, (uint32_t)(phandle->pSendElem) );
			if (NULL != phandle->pSendElem) {
				if (!_queue_enqueue(&(phandle->qs_OutFreeMsg), (QUEUE_ELEMENT_STRUCT_PTR)(phandle->pSendElem))) {
						printf("%s: Error add to free queue port%d mem %x\n", phandle->portNum, (uint32_t)(phandle->pSendElem));
				}
				phandle->pSendElem = NULL;
			}
		}
		break;
	case USB_APP_CDC_SERIAL_STATE_NOTIF:
		{
			/* User: add your own code for serial_state notify event */
		}
		break;
	default:
		error = USBERR_INVALID_REQ_TYPE;
		break;
	}

	return error;
}

#ifdef MIC_USB_DEBUG
_queue_id g_usb_test_qid = 0;
#endif


void Usb_task(uint32_t arg)
{
	TIME_STRUCT         stTime = {0};
	MQX_TICK_STRUCT     stTick = {0};
	_mqx_uint           wres, i;

	if (false == APP_init() ) {
		printf("Error inialize USB task blocked\n");
		_task_block();
	}

	do {
		task_sleep_if_OS_suspended();
		stTime.SECONDS = 0;
		stTime.MILLISECONDS = 100;
		if (!_time_to_ticks(&stTime, &stTick)) {
			printf("%s: Error conv time2tiks\n", __func__);
		}

		wres = _lwsem_wait_for(&(g_app_composite_device.SendReadySem), &stTick );
		if (MQX_CANNOT_CALL_FUNCTION_FROM_ISR == wres || MQX_INVALID_LWSEM == wres) {
			printf("%s: Error %d sem\n", wres);
			_time_delay(100);
			continue;
		}

		for (i = 0; i < COMPOSITE_CFG_MAX; i++) {
			if ( ( TRUE == g_app_composite_device.cdc_vcom[i].start_app ) &&
				 ( TRUE == g_app_composite_device.cdc_vcom[i].start_transactions ) &&
				 g_app_composite_device.cdc_vcom[i].send_ready ) {

				if (!CDC_SendData (g_app_composite_device.cdc_vcom[i].cdc_handle, ((g_app_composite_device.cdc_vcom) + i)) ) {
					printf("%s:WARN cdc_%d\n", __func__, g_app_composite_device.cdc_vcom[i].portNum);
				}
			}
		}

		_time_delay(1);

	} while (0 == g_flag_Exit);
}

#if 0
void requeue_control_msg(APPLICATION_MESSAGE_T *ctl_tx_msg_old)
{
	APPLICATION_MESSAGE_T *ctl_tx_msg;
	_mqx_uint err_task;
	if ((ctl_tx_msg = (APPLICATION_MESSAGE_PTR_T) _msg_alloc (g_out_message_pool)) == NULL)
	{
		if (MQX_OK != (err_task = _task_get_error()))
		{
			_task_set_error(MQX_OK);
		}
		printf("send_control_msg: ERROR: message allocation failed %x\n", err_task);
	}

	if(ctl_tx_msg)
	{
		memcpy(ctl_tx_msg->data,(uint8_t *) ctl_tx_msg_old->data, ctl_tx_msg_old->header.SIZE);

		ctl_tx_msg->header.SOURCE_QID = _msgq_get_id(0, CONTROL_TX_QUEUE);
		ctl_tx_msg->header.TARGET_QID = _msgq_get_id(0, USB_QUEUE);
		ctl_tx_msg->header.SIZE = ctl_tx_msg_old->header.SIZE;
		ctl_tx_msg->portNum = MIC_CDC_USB_1;
		_msgq_send (ctl_tx_msg);

		if (MQX_OK != (err_task = _task_get_error()))
		{
			printf("requeue_control_msg: ERROR: message send failed %x\n", err_task);
			_task_set_error(MQX_OK);
		}
	}
}

#endif //if 0

void USB_Recive_Data ( cdc_struct_t *handle )
{
	if (NULL == handle)
	{
		return;
	}

	if ( handle->cdc_handle == g_app_composite_device.cdc_vcom[0].cdc_handle )
	{
		CDC0_resv ( handle );
	}
#if COMPOSITE_CFG_MAX > 1
	else if ( handle->cdc_handle == g_app_composite_device.cdc_vcom[1].cdc_handle )
	{
		CDC1_resv ( handle );
	}
#endif
#if COMPOSITE_CFG_MAX > 2
	else if ( handle->cdc_handle == g_app_composite_device.cdc_vcom[2].cdc_handle )
	{
		CDC2_resv ( handle ); // CAN0
	}
#endif
#if COMPOSITE_CFG_MAX > 3
	else if ( handle->cdc_handle == g_app_composite_device.cdc_vcom[3].cdc_handle )
	{
		CDC3_resv ( handle ); //CAN1
	}
#endif
#if COMPOSITE_CFG_MAX > 4
	else if ( handle->cdc_handle == g_app_composite_device.cdc_vcom[4].cdc_handle )
	{
		CDC4_resv ( handle ); //J1708
	}
#endif

}

/* Control/command messages are sent through this CDC EP */
void CDC0_resv ( cdc_struct_t *handle )
{
	//Add Source here
	APPLICATION_MESSAGE_T *msg;

#ifdef MIC_USB_DEBUG
	if ( (msg = (APPLICATION_MESSAGE_PTR_T) _msg_alloc (g_in_message_pool)) == NULL )
	{
		printf("CDC0_resv USB Task D: ERROR: message allocation failed\n");
		return;
	}
	_mem_copy ( handle->curr_recv_buf, msg->data, handle->recv_size );
	msg->header.SOURCE_QID = _msgq_get_id( 0, USB_TEST_QUEUE );
	msg->header.TARGET_QID = _msgq_get_id( 0, USB_QUEUE );
	msg->header.SIZE = handle->recv_size;
	msg->portNum = MIC_CDC_USB_1;
	_msgq_send (msg);
	handle->recv_size = 0;
#else

	/* check to make sure we have a start of a frame */
	//TODO: might need to move this check in the frame decode code so we can
	// send a frame reset response!!
	if ( handle->curr_recv_buf[0] != 0x7e )
	{
		//printf("CDC0_resv USB Task: first char not 0x7e it's %x\n", handle->curr_recv_buf[0]);
		handle->recv_size = 0;
		return;
	}

	if ( (msg = (APPLICATION_MESSAGE_PTR_T) _msg_alloc (g_in_message_pool)) == NULL )
	{
	  	_mqx_uint c, c2;
	  	c = _msgq_get_count(_msgq_get_id( 0, USB_QUEUE ));
	  	c2 = _msgq_get_count(_msgq_get_id( 0, CONTROL_RX_QUEUE ));
		printf("CDC0_resv USB Task: ERROR[%u]: message allocation failed %u[%u]\n", _task_get_error(), c, c2);
		handle->recv_size = 0;
//		msg = (APPLICATION_MESSAGE_PTR_T)_mem_alloc(sizeof(APPLICATION_MESSAGE_T);
//		if(0 == msg)
//		{
//			printf("CDC0_resv USB Task: ERROR[%u]: _mem_alloc failed\n", _task_get_error());
//			return;		  
//		}
//		msg->header.SOURCE_QID = _msgq_get_id( 0, USB_QUEUE );
//		msg->header.TARGET_QID = _msgq_get_id( 0, CONTROL_RX_QUEUE );
//		msg->header.SIZE = 0;
//		msg->portNum = MIC_CDC_USB_1;
//		_msgq_send_urgent(msg);
		return;
	}

	memcpy(msg->data, handle->curr_recv_buf, handle->recv_size);

	msg->header.SOURCE_QID = _msgq_get_id( 0, USB_QUEUE );
	msg->header.TARGET_QID = _msgq_get_id( 0, CONTROL_RX_QUEUE );
	msg->header.SIZE = handle->recv_size;
	msg->portNum = MIC_CDC_USB_1;
#if (DEBUG_LOG == 1)
	printf("%s: %u[%u]\n", __func__, msg->header.SOURCE_QID, msg->header.TARGET_QID);
#endif
	_msgq_send (msg);
	handle->recv_size = 0;

#endif
}

void CDC1_resv ( cdc_struct_t *handle )
{
#if COMPOSITE_CFG_MAX > 1
	//Add source here
	//APPLICATION_MESSAGE_T *msg;

#ifdef MIC_USB_DEBUG
	if ( (msg = (APPLICATION_MESSAGE_PTR_T) _msg_alloc (g_in_message_pool)) == NULL )
	{
		printf("CDC1_resv USB Task: ERROR: message allocation failed\n");
		return;
	}
	_mem_copy ( handle->curr_recv_buf, msg->data, handle->recv_size );
	msg->header.SOURCE_QID = _msgq_get_id( 0, USB_TEST_QUEUE );
	msg->header.TARGET_QID = _msgq_get_id( 0, USB_QUEUE );
	msg->header.SIZE = handle->recv_size;
	msg->portNum = MIC_CDC_USB_2;
	_msgq_send (msg);
	handle->recv_size = 0;
#endif

#endif
}

//CAN 0
void CDC2_resv ( cdc_struct_t *handle )
{
#if COMPOSITE_CFG_MAX > 2
	//Add source here
	APPLICATION_MESSAGE_T *msg;

	if ( (msg = (APPLICATION_MESSAGE_PTR_T) _msg_alloc (g_in_message_pool)) == NULL )
	{
		printf("CDC2_resv USB Task: ERROR: message allocation failed\n");
		return;
	}

#ifdef MIC_USB_DEBUG

	_mem_copy ( handle->curr_recv_buf, msg->data, handle->recv_size );
	msg->header.SOURCE_QID = _msgq_get_id( 0, USB_TEST_QUEUE );
	msg->header.TARGET_QID = _msgq_get_id( 0, USB_QUEUE );
	msg->header.SIZE = handle->recv_size;
	msg->portNum = MIC_CDC_USB_3;
	_msgq_send (msg);
	handle->recv_size = 0;
#else
	_mem_copy ( handle->curr_recv_buf, msg->data, handle->recv_size );
	msg->header.SOURCE_QID = _msgq_get_id( 0, USB_QUEUE );
	msg->header.TARGET_QID = _msgq_get_id( 0, CAN1_TX_QUEUE );
	msg->header.SIZE = handle->recv_size + APP_MESSAGE_NO_ARRAY_SIZE;
	msg->portNum = MIC_CDC_USB_3;
	_msgq_send (msg);
	handle->recv_size = 0;
#endif

#endif
}

void CDC3_resv ( cdc_struct_t *handle )
{
#if COMPOSITE_CFG_MAX > 3
	//Add source here
	APPLICATION_MESSAGE_T *msg;

	if ( (msg = (APPLICATION_MESSAGE_PTR_T) _msg_alloc (g_in_message_pool)) == NULL )
	{
		printf("CDC3_resv USB Task: ERROR: message allocation failed\n");
		return;
	}
#ifdef MIC_USB_DEBUG
	_mem_copy ( handle->curr_recv_buf, msg->data, handle->recv_size );
	msg->header.SOURCE_QID = _msgq_get_id( 0, USB_TEST_QUEUE );
	msg->header.TARGET_QID = _msgq_get_id( 0, USB_QUEUE );
	msg->header.SIZE = handle->recv_size;
	msg->portNum = MIC_CDC_USB_4;
	_msgq_send (msg);
	handle->recv_size = 0;
#else
	_mem_copy ( handle->curr_recv_buf, msg->data, handle->recv_size );
	msg->header.SOURCE_QID = _msgq_get_id( 0, USB_QUEUE );
	msg->header.TARGET_QID = _msgq_get_id( 0, CAN2_TX_QUEUE );
	msg->header.SIZE = handle->recv_size + APP_MESSAGE_NO_ARRAY_SIZE;
	msg->portNum = MIC_CDC_USB_4;
	_msgq_send (msg);
	handle->recv_size = 0;
#endif

#endif
}

void CDC4_resv ( cdc_struct_t *handle )
{
#if COMPOSITE_CFG_MAX > 4
	APPLICATION_MESSAGE_T *msg;
	frame_t frame_buf = { 0 };

#ifdef MIC_USB_DEBUG
	if ( (msg = (APPLICATION_MESSAGE_PTR_T) _msg_alloc (g_in_message_pool)) == NULL )
	{
		printf("CDC4_resv USB Task: ERROR: message allocation failed\n");
		return;
	}
	_mem_copy ( handle->curr_recv_buf, msg->data, handle->recv_size );
	msg->header.SOURCE_QID = _msgq_get_id( 0, USB_TEST_QUEUE );
	msg->header.TARGET_QID = _msgq_get_id( 0, USB_QUEUE );
	msg->header.SIZE = handle->recv_size;
	msg->portNum = MIC_CDC_USB_5;
	_msgq_send (msg);
	handle->recv_size = 0;
#else
	if ( 3 > handle->recv_size )
	{
		printf("CDC4_resv USB Task: rec data size %d < 3 \n", handle->recv_size);
		return;
	}

	if ( (msg = (APPLICATION_MESSAGE_PTR_T) _msg_alloc (g_in_message_pool)) == NULL )
	{
		printf("CDC4_resv USB Task: ERROR: message allocation failed\n");
		return;
	}

	frame_setbuffer ( &frame_buf, msg->data, handle->recv_size );
	frame_process_buffer ( &frame_buf, handle->curr_recv_buf, handle->recv_size );
	msg->header.SIZE = frame_buf.data_len;

	msg->header.SOURCE_QID = _msgq_get_id( 0, USB_QUEUE );
	msg->header.TARGET_QID = _msgq_get_id( 0, J1708_TX_QUEUE );
	msg->portNum = MIC_CDC_USB_5;
	_msgq_send (msg);
#endif

#endif
}

static bool CDC_Queue_init (uint8_t cdcNum, uint8_t* memAdr, uint32_t ElemCount, uint32_t queueElemSize) {
	uint16_t j;
	uint8_t* buffPtr = memAdr;

	if (NULL == buffPtr) {
		printf("Erro param %s CDC %d\n", __func__, cdcNum);
		return false;
	}

	g_app_composite_device.cdc_vcom[cdcNum].pOutBuffStart = memAdr;
	g_app_composite_device.cdc_vcom[cdcNum].queue_elem_num = ElemCount;
	for (j = 0; j < ElemCount; j++) {
		((pcdc_mic_queue_element_t)buffPtr)->packetNum = 0;
		((pcdc_mic_queue_element_t)buffPtr)->send_size = 0;
		//((pcdc_mic_queue_element_t)buffPtr)->data_buff = (uint8_t*)(buffPtr + sizeof(pcdc_mic_queue_element_t));
		if (!_queue_enqueue (&(g_app_composite_device.cdc_vcom[cdcNum].qs_OutFreeMsg), (QUEUE_ELEMENT_STRUCT_PTR)buffPtr)) {
			printf("ERROR add element to CDC %d queue\n", cdcNum);
			return false;
		}
		buffPtr += queueElemSize;
	}
	return true;
}

bool CDC_SendData (cdc_handle_t handle, cdc_struct_t *phandle ) {

	if (NULL == phandle) {
		printf("%s: Error param\n", __func__);
		return false;
	}

	if (NULL != phandle->pSendElem) {
		if (!_queue_enqueue(&(phandle->qs_OutFreeMsg), (QUEUE_ELEMENT_STRUCT_PTR)(phandle->pSendElem))) {
			printf("%s: Error add to free queue port%d mem %x\n", phandle->portNum, (uint32_t)(phandle->pSendElem));
		}
		phandle->pSendElem = NULL;
	}

	phandle->pSendElem = (pcdc_mic_queue_element_t)_queue_dequeue(&(phandle->qs_OutInProcMsg));
	if (NULL != phandle->pSendElem) {
		// this is the real fix for a control channel that stops being able to transmit messages
		phandle->send_ready = false;
		if ( USB_OK != USB_Class_CDC_Send_Data(handle, phandle->in_endpoint, phandle->pSendElem->data_buff, phandle->pSendElem->send_size ) ) {
			printf("%s: Error send USB port_%d\n", __func__, phandle->portNum );
			if (!_queue_enqueue(&(phandle->qs_OutFreeMsg), (QUEUE_ELEMENT_STRUCT_PTR)phandle->pSendElem)) {
				printf("%s: Error add to free queue port%d mem %x\n", phandle->portNum, (uint32_t)(phandle->pSendElem));
			}
			phandle->pSendElem = NULL;
			phandle->send_ready = true;
			//printf ("%s: Set sendR T P_%d\n", __func__, phandle->portNum);
			return false;
		}


		phandle->sendPackets++;
		if (phandle->sendPackets <= phandle->SendPacketsCompl) {
			// this is the race condition.  The packet can be sent prior to getting to this point in the code
			// which is why we need to set the send_ready flag to false prior to attempting to send the packet
			// instead of after attempting to send the message.
			printf ("%s: packet already sent T P_%d: sendPackets: %d, compl: %d\n", __func__, phandle->portNum, phandle->sendPackets, phandle->SendPacketsCompl);
		}

		//if (cdcNum != 1) printf ("%s: Set sendR F P_%d for cdcNum: %d, sendPackets: %d, compl: %d, %d (%d)\n", __func__, phandle->portNum, cdcNum, phandle->sendPackets, phandle->SendPacketsCompl,
		//                         pAccHandle->sendPackets, pAccHandle->SendPacketsCompl);
	}

	if (false == phandle->send_ready && NULL == phandle->pSendElem ) {
		//phandle->pSendElem = NULL;
		printf ("%s: Set sendR T P_%d\n", __func__, phandle->portNum);
	}

	return true;
}


/* EOF */
