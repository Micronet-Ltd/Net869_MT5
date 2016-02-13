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

#include "mic_typedef.h"

/*****************************************************************************
 * Constant and Macro's - None
 *****************************************************************************/
#define USB_MSGQ_MAX_POOL_SIZE      20

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

/*****************************************************************************
 * Local Types - None
 *****************************************************************************/

/*****************************************************************************
 * Local Functions Prototypes
 *****************************************************************************/
void USB_App_Device_Callback(uint8_t event_type, void* val, void* arg);
uint8_t USB_App_Class_Callback(uint8_t event, uint16_t value, uint8_t ** data, uint32_t* size, void* arg);

static void CDC0_send ( APPLICATION_MESSAGE_PTR_T msg_ptr );
static void CDC1_send ( APPLICATION_MESSAGE_PTR_T msg_ptr );
static void CDC2_send ( APPLICATION_MESSAGE_PTR_T msg_ptr );
static void CDC3_send ( APPLICATION_MESSAGE_PTR_T msg_ptr );
static void CDC4_send ( APPLICATION_MESSAGE_PTR_T msg_ptr );

static void USB_Recive_Data ( cdc_struct_t *handle );

static void CDC0_resv ( cdc_struct_t *handle );
static void CDC1_resv ( cdc_struct_t *handle );
static void CDC2_resv ( cdc_struct_t *handle );
static void CDC3_resv ( cdc_struct_t *handle );
static void CDC4_resv ( cdc_struct_t *handle );


/*****************************************************************************
 * Local Variables 
 *****************************************************************************/
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
    param->send_size = 0;
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

        g_app_composite_device.cdc_vcom[i].send_ready = FALSE;
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
        	phandle->start_transactions = TRUE;
            phandle->send_ready = TRUE;
            //GPIO_DRV_SetPinOutput (LED_BLUE);
        }
        break;
    case USB_APP_CDC_DTE_DEACTIVATED:
        if (phandle->start_app == TRUE)
        {
        	phandle->start_transactions = FALSE;
            phandle->send_ready = FALSE;
            //GPIO_DRV_ClearPinOutput (LED_BLUE);
        }
        break;
    case USB_DEV_EVENT_DATA_RECEIVED:
		if ((phandle->start_app == TRUE) && (phandle->start_transactions == TRUE))
        {
        	phandle->recv_size = *size;

            if (!phandle->recv_size)
            {
                /* Schedule buffer for next receive event */
                USB_Class_CDC_Recv_Data(handle, phandle->out_endpoint, phandle->curr_recv_buf, g_bulk_out_max_packet_size);
            }

            if ((0 != phandle->recv_size) && (0xFFFFFFFF != phandle->recv_size))
            {
				USB_Recive_Data ( phandle );
                /* Schedule buffer for next receive event */
                USB_Class_CDC_Recv_Data(handle, phandle->out_endpoint, phandle->curr_recv_buf, g_bulk_out_max_packet_size);
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
        	USB_Class_CDC_Send_Data(handle, phandle->in_endpoint, NULL, 0);
        }
        else if ((phandle->start_app == TRUE) && (phandle->start_transactions == TRUE))
        {
            if ((*data != NULL) || ((*data == NULL) && (*size == 0)))
            {
            	phandle->send_ready = TRUE;
                /* User: add your own code for send complete event */
                /* Schedule buffer for next receive event */
            	//USB_Class_CDC_Recv_Data(handle, phandle->out_endpoint, phandle->curr_recv_buf, g_bulk_out_max_packet_size);
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

#define ACC_MSG_SIZE (8 + 6 * 10)

#ifdef MIC_USB_DEBUG
_queue_id g_usb_test_qid = 0;
#endif


void Usb_task(uint32_t arg)
{
    APPLICATION_MESSAGE_PTR_T msg_ptr;

    const _queue_id usb_qid = _msgq_open ((_queue_number)USB_QUEUE, 0);
    if (MSGQ_NULL_QUEUE_ID == usb_qid)
    {
       printf("\nCould not create a message pool USB_QUEUE\n");
       _task_block();
    }

#ifdef MIC_USB_DEBUG
    g_usb_test_qid = _msgq_open ((_queue_number)USB_TEST_QUEUE, 0);
    if (MSGQ_NULL_QUEUE_ID == g_usb_test_qid)
    {
       printf("\nCould not create a message pool USB_QUEUE\n");
       _task_block();
    }
#endif

    APP_init();

    while (1)
    {
       /* call the periodic task function */
       USB_CDC_Periodic_Task();

        msg_ptr = _msgq_receive(usb_qid, 1);
        if(NULL == msg_ptr) { _time_delay(1); continue; }

        switch (msg_ptr->portNum)
        {
        case MIC_CDC_USB_1: //Control function
            CDC0_send ( msg_ptr );
            break;
        case MIC_CDC_USB_2: //Acc function
            CDC1_send ( msg_ptr );
            break;
        case MIC_CDC_USB_3: //CAN0 function
            CDC2_send ( msg_ptr );
            break;
        case MIC_CDC_USB_4: //CAN1 function
            CDC3_send ( msg_ptr );
            break;
        case MIC_CDC_USB_5: //J1708 function
            CDC4_send ( msg_ptr );
            break;
        default:
            _msg_free(msg_ptr);
            _time_delay(1);
            break;
        }
    }
}

void CDC0_send ( APPLICATION_MESSAGE_PTR_T msg_ptr )
{
    uint8_t error;

    if( ( TRUE != g_app_composite_device.cdc_vcom[0].start_app ) ||
            ( TRUE != g_app_composite_device.cdc_vcom[0].start_transactions ) ||
            !g_app_composite_device.cdc_vcom[0].send_ready )
    {
        _msg_free(msg_ptr);
        _time_delay(1);
        return;
    }

    /*check whether enumeration is complete or not */
    else if ( ( TRUE == g_app_composite_device.cdc_vcom[0].start_app ) &&
            ( TRUE == g_app_composite_device.cdc_vcom[0].start_transactions ) &&
            g_app_composite_device.cdc_vcom[0].send_ready)
    {
#ifdef MIC_USB_DEBUG
        _mem_copy( msg_ptr->data, g_app_composite_device.cdc_vcom[0].curr_send_buf, msg_ptr->header.SIZE );
        g_app_composite_device.cdc_vcom[0].send_size = msg_ptr->header.SIZE;
#else
        g_app_composite_device.cdc_vcom[0].send_size = frame_encode(msg_ptr->data, g_app_composite_device.cdc_vcom[0].curr_send_buf, msg_ptr->header.SIZE);
#endif

        g_app_composite_device.cdc_vcom[0].send_ready = FALSE;
        error = USB_Class_CDC_Send_Data(g_app_composite_device.cdc_vcom[0].cdc_handle,
                                        g_app_composite_device.cdc_vcom[0].in_endpoint,
                                        g_app_composite_device.cdc_vcom[0].curr_send_buf, 
                                        g_app_composite_device.cdc_vcom[0].send_size);

        if(error != USB_OK)
        {
            //GPIO_DRV_SetPinOutput(LED_RED);
        }

        _msg_free(msg_ptr);
    }
    else
    {
        _msg_free(msg_ptr);
        _time_delay(1);
    }
}

void CDC1_send ( APPLICATION_MESSAGE_PTR_T msg_ptr )
{
    uint8_t payload[ACC_MSG_SIZE];
    uint8_t error;
    uint8_t * pld_size = &g_app_composite_device.cdc_vcom[1].send_size;
    uint8_t * buf = &g_app_composite_device.cdc_vcom[1].curr_send_buf;

#if COMPOSITE_CFG_MAX > 1
    // FIXME: bad predicate
    if( ( TRUE != g_app_composite_device.cdc_vcom[1].start_app ) ||
            ( TRUE != g_app_composite_device.cdc_vcom[1].start_transactions ) ||
            !g_app_composite_device.cdc_vcom[1].send_ready )
    {
        _msg_free(msg_ptr);
        _time_delay(1);
        return;
    }

    /*check whether enumeration is complete or not */
    else if ( ( TRUE == g_app_composite_device.cdc_vcom[1].start_app ) &&
            ( TRUE == g_app_composite_device.cdc_vcom[1].start_transactions ) &&
            g_app_composite_device.cdc_vcom[1].send_ready)
    {
#ifdef MIC_USB_DEBUG
        _mem_copy( msg_ptr->data, g_app_composite_device.cdc_vcom[1].curr_send_buf, msg_ptr->header.SIZE );
        g_app_composite_device.cdc_vcom[1].send_size = msg_ptr->header.SIZE;
#else
        // FIXME: endian assumption
        _mem_copy (&(msg_ptr->timestamp), payload, sizeof(msg_ptr->timestamp ));
        // FIXME: no single point definition of actual payload size here
        _mem_copy( msg_ptr->data, payload + sizeof(msg_ptr->timestamp), msg_ptr->header.SIZE );
        g_app_composite_device.cdc_vcom[1].send_size = \
        		frame_encode(payload, g_app_composite_device.cdc_vcom[1].curr_send_buf, (msg_ptr->header.SIZE + sizeof(msg_ptr->timestamp )));

#endif
        g_app_composite_device.cdc_vcom[1].send_ready = FALSE;
        error = USB_Class_CDC_Send_Data(g_app_composite_device.cdc_vcom[1].cdc_handle,
                                        g_app_composite_device.cdc_vcom[1].in_endpoint,
                                        g_app_composite_device.cdc_vcom[1].curr_send_buf, 
                                        g_app_composite_device.cdc_vcom[1].send_size);

        //printf("CDC1_send beg:%x,%x,%x,%x - end:%x,%x,%x,%x \t pld = %d, enc = %d\n", buf[0], buf[1], buf[2], buf[3], \
        //		buf[*pld_size -4], buf[*pld_size -3], buf[*pld_size-2], buf[*pld_size -1], (msg_ptr->header.SIZE + sizeof(msg_ptr->timestamp )), *pld_size );

        if(error != USB_OK)
        {
            //GPIO_DRV_SetPinOutput(LED_RED);
        }
        /*
        else { GPIO_DRV_ClearPinOutput(LED_RED); }
        static x = 0;
        if(x) { GPIO_DRV_ClearPinOutput (LED_GREEN); x = 0; }
        else { GPIO_DRV_SetPinOutput(LED_GREEN); x = 1; }
        */
        _msg_free(msg_ptr);
        return;
    }
    else
    {
    	_msg_free(msg_ptr);
    	_time_delay(1);
    }
#else
    _msg_free(msg_ptr);
    _time_delay(10);
#endif
    

}

void CDC2_send ( APPLICATION_MESSAGE_PTR_T msg_ptr )
{
#if COMPOSITE_CFG_MAX > 2
    uint8_t error;

    if( ( TRUE != g_app_composite_device.cdc_vcom[2].start_app ) ||
            ( TRUE != g_app_composite_device.cdc_vcom[2].start_transactions ) ||
            !g_app_composite_device.cdc_vcom[2].send_ready )
    {
        _msg_free(msg_ptr);
        _time_delay(1);
        return;
    }

    /*check whether enumeration is complete or not */
    else if ( ( TRUE == g_app_composite_device.cdc_vcom[2].start_app ) &&
            ( TRUE == g_app_composite_device.cdc_vcom[2].start_transactions ) &&
            g_app_composite_device.cdc_vcom[2].send_ready)
    {
#ifdef MIC_USB_DEBUG
        _mem_copy( msg_ptr->data, g_app_composite_device.cdc_vcom[2].curr_send_buf, msg_ptr->header.SIZE );
        g_app_composite_device.cdc_vcom[2].send_size = msg_ptr->header.SIZE;
#else
        g_app_composite_device.cdc_vcom[2].send_size = frame_encode(msg_ptr->data, g_app_composite_device.cdc_vcom[2].curr_send_buf, msg_ptr->header.SIZE);
#endif

        g_app_composite_device.cdc_vcom[2].send_ready = FALSE;
        error = USB_Class_CDC_Send_Data(g_app_composite_device.cdc_vcom[2].cdc_handle,
                                        g_app_composite_device.cdc_vcom[2].in_endpoint,
                                        g_app_composite_device.cdc_vcom[2].curr_send_buf, 
                                        g_app_composite_device.cdc_vcom[2].send_size);

        if(error != USB_OK)
        {
            //GPIO_DRV_SetPinOutput(LED_RED);
        }

        _msg_free(msg_ptr);
    }
    else
    {
        _msg_free(msg_ptr);
        _time_delay(1);
    }
#else
    _msg_free(msg_ptr);
    _time_delay(1);
#endif
    
}

void CDC3_send ( APPLICATION_MESSAGE_PTR_T msg_ptr )
{
#if COMPOSITE_CFG_MAX > 3
    uint8_t error;

    if( ( TRUE != g_app_composite_device.cdc_vcom[3].start_app ) ||
            ( TRUE != g_app_composite_device.cdc_vcom[3].start_transactions ) ||
            !g_app_composite_device.cdc_vcom[3].send_ready )
    {
        _msg_free(msg_ptr);
        _time_delay(1);
        return;
    }

    /*check whether enumeration is complete or not */
    else if ( ( TRUE == g_app_composite_device.cdc_vcom[3].start_app ) &&
            ( TRUE == g_app_composite_device.cdc_vcom[3].start_transactions ) &&
            g_app_composite_device.cdc_vcom[3].send_ready)
    {
#ifdef MIC_USB_DEBUG
        _mem_copy( msg_ptr->data, g_app_composite_device.cdc_vcom[3].curr_send_buf, msg_ptr->header.SIZE );
        g_app_composite_device.cdc_vcom[3].send_size = msg_ptr->header.SIZE;
#else
        g_app_composite_device.cdc_vcom[3].send_size = frame_encode(msg_ptr->data, g_app_composite_device.cdc_vcom[3].curr_send_buf, msg_ptr->header.SIZE);
#endif

        g_app_composite_device.cdc_vcom[3].send_ready = FALSE;
        error = USB_Class_CDC_Send_Data(g_app_composite_device.cdc_vcom[3].cdc_handle,
                                        g_app_composite_device.cdc_vcom[3].in_endpoint,
                                        g_app_composite_device.cdc_vcom[3].curr_send_buf, 
                                        g_app_composite_device.cdc_vcom[3].send_size);

        if(error != USB_OK)
        {
            //GPIO_DRV_SetPinOutput(LED_RED);
        }

        _msg_free(msg_ptr);
    }
    else
    {
        _msg_free(msg_ptr);
        _time_delay(1);
    }
#else
     _msg_free(msg_ptr);
     _time_delay(1);
#endif
}

void CDC4_send ( APPLICATION_MESSAGE_PTR_T msg_ptr )
{
#if COMPOSITE_CFG_MAX > 4
	uint8_t error;

    if( ( TRUE != g_app_composite_device.cdc_vcom[4].start_app ) ||
    		( TRUE != g_app_composite_device.cdc_vcom[4].start_transactions ) ||
			!g_app_composite_device.cdc_vcom[4].send_ready )
    {
        _msg_free(msg_ptr);
        _time_delay(1);
        return;
    }

    /*check whether enumeration is complete or not */
    else if ( ( TRUE == g_app_composite_device.cdc_vcom[4].start_app ) &&
    		( TRUE == g_app_composite_device.cdc_vcom[4].start_transactions ) &&
			g_app_composite_device.cdc_vcom[4].send_ready)
    {
#ifdef MIC_USB_DEBUG
        _mem_copy( msg_ptr->data, g_app_composite_device.cdc_vcom[4].curr_send_buf, msg_ptr->header.SIZE );
        g_app_composite_device.cdc_vcom[4].send_size = msg_ptr->header.SIZE;
#else
        g_app_composite_device.cdc_vcom[4].send_size = frame_encode(msg_ptr->data, g_app_composite_device.cdc_vcom[4].curr_send_buf, msg_ptr->header.SIZE);
#endif

        g_app_composite_device.cdc_vcom[4].send_ready = FALSE;
        error = USB_Class_CDC_Send_Data(g_app_composite_device.cdc_vcom[4].cdc_handle,
                                        g_app_composite_device.cdc_vcom[4].in_endpoint,
                                        g_app_composite_device.cdc_vcom[4].curr_send_buf, 
                                        g_app_composite_device.cdc_vcom[4].send_size);

        if(error != USB_OK)
        {
            //GPIO_DRV_SetPinOutput(LED_RED);
        }

        _msg_free(msg_ptr);
    }
    else
    {
        _msg_free(msg_ptr);
        _time_delay(1);
    }
#else
    _msg_free(msg_ptr);
    _time_delay(1);
#endif
    
}

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
        CDC2_resv ( handle );
    }
#endif
#if COMPOSITE_CFG_MAX > 3
    else if ( handle->cdc_handle == g_app_composite_device.cdc_vcom[3].cdc_handle )
    {
        CDC3_resv ( handle );
    }
#endif
#if COMPOSITE_CFG_MAX > 4
    else if ( handle->cdc_handle == g_app_composite_device.cdc_vcom[4].cdc_handle )
    {
        CDC4_resv ( handle );
    }
#endif
   
}

void CDC0_resv ( cdc_struct_t *handle )
{
    //Add Source here
    APPLICATION_MESSAGE_T *msg;

#ifdef MIC_USB_DEBUG
    if ( (msg = (APPLICATION_MESSAGE_PTR_T) _msg_alloc (g_in_message_pool)) == NULL )
    {
        printf("CDC0_resv USB Task: ERROR: message allocation failed\n");
        return;
    }
    _mem_copy ( handle->curr_recv_buf, msg->data, handle->recv_size );
    msg->header.SOURCE_QID = _msgq_get_id( 0, USB_TEST_QUEUE );
    msg->header.TARGET_QID = _msgq_get_id( 0, USB_QUEUE );
    msg->header.SIZE = handle->recv_size;
    msg->portNum = MIC_CDC_USB_1;
    _msgq_send (msg);
    handle->recv_size = 0;
#endif
}

void CDC1_resv ( cdc_struct_t *handle )
{
#if COMPOSITE_CFG_MAX > 1
    //Add source here
    APPLICATION_MESSAGE_T *msg;

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

void CDC2_resv ( cdc_struct_t *handle )
{
#if COMPOSITE_CFG_MAX > 2
    //Add source here
    APPLICATION_MESSAGE_T *msg;

#ifdef MIC_USB_DEBUG
    if ( (msg = (APPLICATION_MESSAGE_PTR_T) _msg_alloc (g_in_message_pool)) == NULL )
    {
        printf("CDC2_resv USB Task: ERROR: message allocation failed\n");
        return;
    }
    _mem_copy ( handle->curr_recv_buf, msg->data, handle->recv_size );
    msg->header.SOURCE_QID = _msgq_get_id( 0, USB_TEST_QUEUE );
    msg->header.TARGET_QID = _msgq_get_id( 0, USB_QUEUE );
    msg->header.SIZE = handle->recv_size;
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

#ifdef MIC_USB_DEBUG
    if ( (msg = (APPLICATION_MESSAGE_PTR_T) _msg_alloc (g_in_message_pool)) == NULL )
    {
        printf("CDC3_resv USB Task: ERROR: message allocation failed\n");
        return;
    }
    _mem_copy ( handle->curr_recv_buf, msg->data, handle->recv_size );
    msg->header.SOURCE_QID = _msgq_get_id( 0, USB_TEST_QUEUE );
    msg->header.TARGET_QID = _msgq_get_id( 0, USB_QUEUE );
    msg->header.SIZE = handle->recv_size;
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

    frame_setbuffer ( &frame_buf, handle->curr_recv_buf, handle->recv_size );
    frame_process_buffer ( &frame_buf, msg->data, handle->recv_size );

    msg->header.SOURCE_QID = _msgq_get_id( 0, USB_QUEUE );
    msg->header.TARGET_QID = _msgq_get_id( 0, J1708_TX_QUEUE );
    msg->header.SIZE = handle->recv_size;
    _msgq_send (msg);
#endif

#endif
}



/* EOF */

