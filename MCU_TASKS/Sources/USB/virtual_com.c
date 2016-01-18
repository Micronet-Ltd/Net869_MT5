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
#include "virtual_com.h"

/////
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


/*****************************************************************************
 * Constant and Macro's - None
 *****************************************************************************/

/*****************************************************************************
 * Global Functions Prototypes
 *****************************************************************************/
void TestApp_Init(void);

/****************************************************************************
 * Global Variables
 ****************************************************************************/
extern usb_desc_request_notify_struct_t desc_callback;
app_composite_device_struct_t g_app_composite_device;
extern uint8_t USB_Desc_Set_Speed(uint32_t handle, uint16_t speed);
//cdc_handle_t g_app_handle;
cdc_handle_t *g_cdc_vcom_ptr[MIC_USB_CDC_INF_COUNT];

/*****************************************************************************
 * Local Types - None
 *****************************************************************************/

/*****************************************************************************
 * Local Functions Prototypes
 *****************************************************************************/
void USB_App_Device_Callback(uint8_t event_type, void* val, void* arg);
uint8_t USB_App_Class_Callback(uint8_t event, uint16_t value, uint8_t ** data, uint32_t* size, void* arg);
//void Virtual_Com_App(void);
/*****************************************************************************
 * Local Variables 
 *****************************************************************************/
uint8_t g_line_coding[LINE_CODING_SIZE] =
{
    /*e.g. 0x00,0x10,0x0E,0x00 : 0x000E1000 is 921600 bits per second */
    (LINE_CODE_DTERATE_IFACE >> 0) & 0x000000FF,
    (LINE_CODE_DTERATE_IFACE >> 8) & 0x000000FF,
    (LINE_CODE_DTERATE_IFACE >> 16) & 0x000000FF,
    (LINE_CODE_DTERATE_IFACE >> 24) & 0x000000FF,
    LINE_CODE_CHARFORMAT_IFACE,
    LINE_CODE_PARITYTYPE_IFACE,
    LINE_CODE_DATABITS_IFACE
};

uint8_t g_abstract_state[COMM_FEATURE_DATA_SIZE] =
{
    (STATUS_ABSTRACT_STATE_IFACE >> 0) & 0x00FF,
    (STATUS_ABSTRACT_STATE_IFACE >> 8) & 0x00FF
};

uint8_t g_country_code[COMM_FEATURE_DATA_SIZE] =
{
    (COUNTRY_SETTING_IFACE >> 0) & 0x00FF,
    (COUNTRY_SETTING_IFACE >> 8) & 0x00FF
};
//static bool start_app = FALSE;
//static bool start_transactions = FALSE;

//static uint8_t g_curr_recv_buf[DATA_BUFF_SIZE];
//static uint8_t g_curr_send_buf[DATA_BUFF_SIZE];

//static uint32_t g_recv_size;
//static uint32_t g_send_size;

static uint16_t g_cdc_device_speed;
static uint16_t g_bulk_out_max_packet_size;
static uint16_t g_bulk_in_max_packet_size;
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
    //UNUSED_ARGUMENT(handle)
    /* if interface valid */
    if (interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        /* get line coding data*/
        *coding_data = g_line_coding;
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

    //UNUSED_ARGUMENT(handle)

    /* if interface valid */
    if (interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        /* set line coding data*/
        for (count = 0; count < LINE_CODING_SIZE; count++)
        {
            g_line_coding[count] = *((*coding_data + USB_SETUP_PKT_SIZE) + count);
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
    //UNUSED_ARGUMENT(handle)
    /* if interface valid */
    if (interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        /* get line coding data*/
        *feature_data = g_abstract_state;
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
    //UNUSED_ARGUMENT(handle)
    /* if interface valid */
    if (interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        /* get line coding data*/
        *feature_data = g_country_code;
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
    //UNUSED_ARGUMENT(handle)
    /* if interface valid */
    if (interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        /* set Abstract State Feature*/
        for (count = 0; count < COMM_FEATURE_DATA_SIZE; count++)
        {
            g_abstract_state[count] = *(*feature_data + count);
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
    //UNUSED_ARGUMENT (handle)

    /* if interface valid */
    if (interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        for (count = 0; count < COMM_FEATURE_DATA_SIZE; count++)
        {
            g_country_code[count] = *(*feature_data + count);
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
    param->g_recv_size = 0;
    param->g_send_size = 0;
}
/*****************************************************************************
 *  
 *    @name         APP_init
 * 
 *    @brief         This function do initialization for APP.
 * 
 *    @param         None
 * 
 *    @return       None
 **                  
 *****************************************************************************/
void APP_init(void)
{
    uint8_t i;
    composite_device_struct_t *l_compositeDevice = NULL;

    USB_prepare_descroptors (  );
    USB_init_memory_Desc (  );
	class_config_struct_t* cdc_vcom_config_callback_handle;

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

        g_cdc_vcom_ptr[i] = &(g_app_composite_device.cdc_vcom[i].cdc_handle);
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
    g_app_composite_device.cdc_vcom[4].out_endpoint = DIC1_BULK_OUT_ENDPOINT;
    g_app_composite_device.cdc_vcom[4].cdc_handle   = (cdc_handle_t)g_app_composite_device.composite_device_config_list[4].class_handle;
#endif

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
    cdc_struct_t *handle1 = (cdc_struct_t *) arg;
    uint32_t handle = *((uint32_t *) arg);

    if (event_type == USB_DEV_EVENT_BUS_RESET)
    {
        handle1->start_app = FALSE;
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
        USB_Class_CDC_Recv_Data(handle, handle1->out_endpoint , handle1->g_curr_recv_buf, g_bulk_out_max_packet_size);
        handle1->start_app = TRUE;
    }
    else if (event_type == USB_DEV_EVENT_ERROR)
    {
        /* add user code for error handling */
    }
    return;
}

int g_send_ready = 1;
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
    cdc_struct_t *handle1 = (cdc_struct_t *) arg;

    switch(event)
    {
    case GET_LINE_CODING:
        error = USB_Get_Line_Coding(handle, value, data);
        break;
    case GET_ABSTRACT_STATE:
        error = USB_Get_Abstract_State(handle, value, data);
        break;
    case GET_COUNTRY_SETTING:
        error = USB_Get_Country_Setting(handle, value, data);
        break;
    case SET_LINE_CODING:
        error = USB_Set_Line_Coding(handle, value, data);
        break;
    case SET_ABSTRACT_STATE:
        error = USB_Set_Abstract_State(handle, value, data);
        break;
    case SET_COUNTRY_SETTING:
        error = USB_Set_Country_Setting(handle, value, data);
        break;
    case USB_APP_CDC_DTE_ACTIVATED:
        if (handle1->start_app == TRUE)
        {
        	handle1->start_transactions = TRUE;
            //GPIO_DRV_SetPinOutput (LED_BLUE);
        }
        break;
    case USB_APP_CDC_DTE_DEACTIVATED:
        if (handle1->start_app == TRUE)
        {
        	handle1->start_transactions = FALSE;
            //GPIO_DRV_ClearPinOutput (LED_BLUE);
        }
        break;
    case USB_DEV_EVENT_DATA_RECEIVED:
        {
        if ((handle1->start_app == TRUE) && (handle1->start_transactions == TRUE))
        {
        	handle1->g_recv_size = *size;

            if (!handle1->g_recv_size)
            {
                /* Schedule buffer for next receive event */
                USB_Class_CDC_Recv_Data(handle, handle1->out_endpoint, handle1->g_curr_recv_buf, g_bulk_out_max_packet_size);
            }
        }
    }
        break;
    case USB_DEV_EVENT_SEND_COMPLETE:
        {
        if ((size != NULL) && (*size != 0) && (!(*size % g_bulk_in_max_packet_size)))
        {
            /* If the last packet is the size of endpoint, then send also zero-ended packet,
             ** meaning that we want to inform the host that we do not have any additional
             ** data, so it can flush the output.
             */
        	USB_Class_CDC_Send_Data(handle, handle1->in_endpoint, NULL, 0);
        }
        else if ((handle1->start_app == TRUE) && (handle1->start_transactions == TRUE))
        {
            if ((*data != NULL) || ((*data == NULL) && (*size == 0)))
            {
            	g_send_ready = 1;
                /* User: add your own code for send complete event */
                /* Schedule buffer for next receive event */
                //USB_Class_CDC_Recv_Data(handle, DIC_BULK_OUT_ENDPOINT, g_curr_recv_buf, g_bulk_out_max_packet_size);
            }
        }
    }
        break;
    case USB_APP_CDC_SERIAL_STATE_NOTIF:
        {
			/* User: add your own code for serial_state notify event */
		}
        break;
    default:
        {
			error = USBERR_INVALID_REQ_TYPE;
			break;
		}

    }

    return error;
}


void Usb_task(uint32_t arg)
{

	// TODO: SErial abstraction layer
	// This is for accelerometer only testing
#define ACC_MSG_SIZE (8 + 6 * 10)
	uint8_t frame_encoded[ACC_MSG_SIZE * 2 + 2];
	uint8_t payload[ACC_MSG_SIZE];
	APPLICATION_MESSAGE_PTR_T msg_ptr;
	uint8_t error;

	const _queue_id usb_qid = _msgq_open ((_queue_number)USB_QUEUE, 0);




    APP_init();

    while (1)
    {
		/* call the periodic task function */
		USB_CDC_Periodic_Task();

//		msg_ptr = _msgq_receive(usb_qid, 1);
//		if(NULL == msg_ptr) { _time_delay(1); continue; }
//
//		// FIXME: bad predicate
//		if( !((start_app == TRUE) && (start_transactions == TRUE) && g_send_ready))
//		{
//			_msg_free(msg_ptr);
//			_time_delay(1);
//		}
//		/*check whether enumeration is complete or not */
//		else if ((start_app == TRUE) && (start_transactions == TRUE) && g_send_ready)
//		{
//			uint32_t frame_len;
//			uint64_t ts = (msg_ptr->timestamp.SECONDS * 1000) + msg_ptr->timestamp.MILLISECONDS;
//
//			// FIXME: endian assumption
//			memcpy(payload, &ts, 8);
//			// FIXME: no single point definition of actual payload size here
//			memcpy(payload + 8, msg_ptr->data, sizeof(payload) - 8);
//			frame_len = frame_encode(payload, frame_encoded, sizeof(payload));
//
//			g_send_ready = 0;
//			error = USB_Class_CDC_Send_Data(g_app_handle, DIC_BULK_IN_ENDPOINT, frame_encoded, frame_len);
//
//			if(error != USB_OK)
//			{
//				//GPIO_DRV_SetPinOutput(LED_RED);
//			}
//			/*
//			else { GPIO_DRV_ClearPinOutput(LED_RED); }
//			static x = 0;
//			if(x) { GPIO_DRV_ClearPinOutput (LED_GREEN); x = 0; }
//			else { GPIO_DRV_SetPinOutput(LED_GREEN); x = 1; }
//			*/
//			_msg_free(msg_ptr);
//
//		}
//		else
		{
			_time_delay(1);
		}
    }
}

/* EOF */

