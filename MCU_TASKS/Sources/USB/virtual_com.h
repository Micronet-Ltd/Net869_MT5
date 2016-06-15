/**HEADER********************************************************************
* 
* Copyright (c) 2008, 2015 Freescale Semiconductor;
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
* $FileName: virtual_com.h$
* $Version : 
* $Date    : 
*
* Comments:
*
* @brief The file contains Macro's and functions needed by the virtual com 
*        application
*
*****************************************************************************/

#ifndef _VIRTUAL_COM_H
#define _VIRTUAL_COM_H  1

#include "usb_descriptor.h"

/******************************************************************************
 * Constants - None
 *****************************************************************************/

/******************************************************************************
 * Macro's
 *****************************************************************************/
        
#define COMPOSITE_CFG_MAX                       (MIC_USB_CDC_INF_COUNT)
        
#if HIGH_SPEED
#define CONTROLLER_ID         	                (USB_CONTROLLER_EHCI_0)
#define DATA_BUFF_SIZE        	                (HS_DIC_BULK_OUT_ENDP_PACKET_SIZE)
#else
#define CONTROLLER_ID         	                (USB_CONTROLLER_KHCI_0)
#define DATA_BUFF_SIZE        	                (FS_DIC_BULK_OUT_ENDP_PACKET_SIZE)
#endif

#define MIC_USB_FRAME_BUFFER_SIZE               (0x84)

/* Implementation Specific Macros */
#define LINE_CODING_SIZE              	        (0x07)
#define COMM_FEATURE_DATA_SIZE        	        (0x02)

#define LINE_CODE_DTERATE_IFACE      	        (115200) /*e.g 9600 is 0x00002580 */
#define LINE_CODE_CHARFORMAT_IFACE   	        (0x00)   /* 1 stop bit */
#define LINE_CODE_PARITYTYPE_IFACE   	        (0x00)  /* No Parity */
#define LINE_CODE_DATABITS_IFACE     	        (0x08)  /* Data Bits Format */

#define STATUS_ABSTRACT_STATE_IFACE  	        (0x0000) /* Disable Multiplexing ENDP in
                                                          this interface will continue
                                                          to accept/offer data*/
#define COUNTRY_SETTING_IFACE        	        (0x0000) /* Country Code in the format as 
                                                            defined in [ISO3166]- 
                                                            - PLEASE CHECK THESE VALUES*/  
												  
/***************************************************************************** 
  * Types
  ****************************************************************************/

/* cdc_struct_t is as follow. */
/* It contain variables for one cdc class. */
typedef struct _cdc_variable_struct
{
    cdc_handle_t                cdc_handle;
    uint8_t                     curr_recv_buf[DATA_BUFF_SIZE];
    //uint8_t                     curr_send_buf[MIC_USB_FRAME_BUFFER_SIZE];
    uint32_t                    recv_size;
    //uint8_t                     send_size;
    bool                        start_app;
    bool                        start_transactions;
    bool                        send_ready;
    uint8_t                     out_endpoint;
    uint8_t                     in_endpoint;
    uint8_t                     line_coding[LINE_CODING_SIZE];
    uint8_t                     abstract_state[COMM_FEATURE_DATA_SIZE];
    uint8_t                     country_code[COMM_FEATURE_DATA_SIZE];
    uint8_t                     portNum;
    QUEUE_STRUCT                qs_OutFreeMsg;
    QUEUE_STRUCT                qs_OutInProcMsg;
    pcdc_mic_queue_element_t    pSendElem;
    uint8_t                     queue_elem_num;
    uint8_t*                    pOutBuffStart;
    uint32_t                    sendPackets;
    uint32_t                    SendPacketsCompl;
}cdc_struct_t;

/* cdc_struct_t represents cdc class */
typedef struct app_composite_device_struct
{
    composite_handle_t          composite_device;
    cdc_struct_t                cdc_vcom[COMPOSITE_CFG_MAX];
    composite_config_struct_t   composite_device_config_callback;
    class_config_struct_t       composite_device_config_list[COMPOSITE_CFG_MAX];
    uint8_t                     *pDataBuff;
    uint32_t                    DataBuffSize;
    LWSEM_STRUCT                SendReadySem;
}app_composite_device_struct_t;


/*****************************************************************************
 * Global variables
 *****************************************************************************/

extern app_composite_device_struct_t g_app_composite_device;

/*****************************************************************************
 * Global Functions
 *****************************************************************************/
//extern bool Virtual_Com_MemAloc(void);
//extern pcdc_mic_queue_element_t GetUSBWriteBuffer(uint8_t cdcport);
//extern bool SetUSBWriteBuffer(pcdc_mic_queue_element_t pcdcBuff, uint8_t cdcport);

#endif 


/* EOF */

