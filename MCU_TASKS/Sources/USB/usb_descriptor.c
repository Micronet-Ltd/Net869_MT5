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
 * $FileName: usb_descriptor.c$
 * $Version : 
 * $Date    : 
 *
 * Comments:
 *
 * @brief The file contains USB descriptors 
 *
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <mqx.h>

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device_stack_interface.h"

#include "usb_class_composite.h"
#include "usb_composite.h"

#include "virtual_com.h"
#include "usb_descriptor.h"


//#define MIC_DESCR_MEM

#ifdef MIC_DESCR_MEM
usb_class_struct_t *g_MIC_usb_dec_class;
usb_all_languages_t *g_MIC_g_languages;

uint8_t *g_MIC_g_device_descriptor;
uint8_t *g_MIC_g_config_descriptor;
#if HIGH_SPEED
uint8_t *g_MIC_g_device_qualifier_descriptor;
uint8_t *g_MIC_g_other_speed_config_descriptor;
#endif

uint8_t **g_MIC_g_std_descriptors;
uint8_t *g_MIC_g_alternate_interface;
usb_interfaces_struct_t *g_MIC_usb_configuration;
#endif

/*****************************************************************************
 * Constant and Macro's
 *****************************************************************************/
/* structure containing details of all the endpoints used by this device */
usb_ep_struct_t cic1_ep[CIC_ENDP_COUNT] = {
#if CIC_NOTIF_ELEM_SUPPORT
	{
		CIC1_NOTIF_ENDPOINT,
		USB_INTERRUPT_PIPE,
		USB_SEND,
		CIC_NOTIF_ENDP_PACKET_SIZE
	}
#endif
};

usb_ep_struct_t dic1_ep[DIC_ENDP_COUNT] = {
#if DATA_CLASS_SUPPORT
	{
		DIC1_BULK_IN_ENDPOINT,
		USB_BULK_PIPE,
		USB_SEND,
		DIC_BULK_IN_ENDP_PACKET_SIZE
	},
	{
		DIC1_BULK_OUT_ENDPOINT,
		USB_BULK_PIPE,
		USB_RECV,
		DIC_BULK_OUT_ENDP_PACKET_SIZE
	}
#endif
};

#if MIC_USB_CDC_INF_COUNT > 1
usb_ep_struct_t cic2_ep[CIC_ENDP_COUNT] = {
#if CIC_NOTIF_ELEM_SUPPORT
	{
		CIC2_NOTIF_ENDPOINT,
		USB_INTERRUPT_PIPE,
		USB_SEND,
		CIC_NOTIF_ENDP_PACKET_SIZE
	}
#endif
};

usb_ep_struct_t dic2_ep[DIC_ENDP_COUNT] = {
#if DATA_CLASS_SUPPORT
	{
		DIC2_BULK_IN_ENDPOINT,
		USB_BULK_PIPE,
		USB_SEND,
		DIC_BULK_IN_ENDP_PACKET_SIZE
	},
	{
		DIC2_BULK_OUT_ENDPOINT,
		USB_BULK_PIPE,
		USB_RECV,
		DIC_BULK_OUT_ENDP_PACKET_SIZE
	}
#endif
};
#endif //MIC_USB_CDC_INF_COUNT > 1

#if MIC_USB_CDC_INF_COUNT > 2
usb_ep_struct_t cic3_ep[CIC_ENDP_COUNT] = {
#if CIC_NOTIF_ELEM_SUPPORT
	{
		CIC3_NOTIF_ENDPOINT,
		USB_INTERRUPT_PIPE,
		USB_SEND,
		CIC_NOTIF_ENDP_PACKET_SIZE
	}
#endif
};

usb_ep_struct_t dic3_ep[DIC_ENDP_COUNT] = {
#if DATA_CLASS_SUPPORT
	{
		DIC3_BULK_IN_ENDPOINT,
		USB_BULK_PIPE,
		USB_SEND,
		DIC_BULK_IN_ENDP_PACKET_SIZE
	},
	{
		DIC3_BULK_OUT_ENDPOINT,
		USB_BULK_PIPE,
		USB_RECV,
		DIC_BULK_OUT_ENDP_PACKET_SIZE
	}
#endif
};
#endif // MIC_USB_CDC_INF_COUNT > 2

#if MIC_USB_CDC_INF_COUNT > 3
usb_ep_struct_t cic4_ep[CIC_ENDP_COUNT] = {
#if CIC_NOTIF_ELEM_SUPPORT
	{
		CIC4_NOTIF_ENDPOINT,
		USB_INTERRUPT_PIPE,
		USB_SEND,
		CIC_NOTIF_ENDP_PACKET_SIZE
	}
#endif
};

usb_ep_struct_t dic4_ep[DIC_ENDP_COUNT] = {
#if DATA_CLASS_SUPPORT
	{
		DIC4_BULK_IN_ENDPOINT,
		USB_BULK_PIPE,
		USB_SEND,
		DIC_BULK_IN_ENDP_PACKET_SIZE
	},
	{
		DIC4_BULK_OUT_ENDPOINT,
		USB_BULK_PIPE,
		USB_RECV,
		DIC_BULK_OUT_ENDP_PACKET_SIZE
	}
#endif
};
#endif // MIC_USB_CDC_INF_COUNT > 3

#if MIC_USB_CDC_INF_COUNT > 4
usb_ep_struct_t cic5_ep[CIC_ENDP_COUNT] = {
#if CIC_NOTIF_ELEM_SUPPORT
	{
		CIC5_NOTIF_ENDPOINT,
		USB_INTERRUPT_PIPE,
		USB_SEND,
		CIC_NOTIF_ENDP_PACKET_SIZE
	}
#endif
};

usb_ep_struct_t dic5_ep[DIC_ENDP_COUNT] = {
#if DATA_CLASS_SUPPORT
	{
		DIC5_BULK_IN_ENDPOINT,
		USB_BULK_PIPE,
		USB_SEND,
		DIC_BULK_IN_ENDP_PACKET_SIZE
	},
	{
		DIC5_BULK_OUT_ENDPOINT,
		USB_BULK_PIPE,
		USB_RECV,
		DIC_BULK_OUT_ENDP_PACKET_SIZE
	}
#endif
};
#endif // MIC_USB_CDC_INF_COUNT > 4

/* Interfaces */
#define USB_CDC_IF_MAX      (2)
#define USB_CDC_CFG_MAX     (MIC_USB_CDC_INF_COUNT)
#define USB_CDC_CLASS_MAX   (MIC_USB_CDC_INF_COUNT)

static usb_if_struct_t usb1_if[USB_CDC_IF_MAX] = {
	USB_DESC_INTERFACE(0, CIC_ENDP_COUNT, cic1_ep),
	USB_DESC_INTERFACE(1, DIC_ENDP_COUNT, dic1_ep),
};

#if MIC_USB_CDC_INF_COUNT > 1
static usb_if_struct_t usb2_if[USB_CDC_IF_MAX] = {
	USB_DESC_INTERFACE(2, CIC_ENDP_COUNT, cic2_ep),
	USB_DESC_INTERFACE(3, DIC_ENDP_COUNT, dic2_ep),
};
#endif

#if MIC_USB_CDC_INF_COUNT > 2
static usb_if_struct_t usb3_if[USB_CDC_IF_MAX] = {
	USB_DESC_INTERFACE(4, CIC_ENDP_COUNT, cic3_ep),
	USB_DESC_INTERFACE(5, DIC_ENDP_COUNT, dic3_ep),
};
#endif

#if MIC_USB_CDC_INF_COUNT > 3
static usb_if_struct_t usb4_if[USB_CDC_IF_MAX] = {
	USB_DESC_INTERFACE(6, CIC_ENDP_COUNT, cic4_ep),
	USB_DESC_INTERFACE(7, DIC_ENDP_COUNT, dic4_ep),
};
#endif

#if MIC_USB_CDC_INF_COUNT > 4
static usb_if_struct_t usb5_if[USB_CDC_IF_MAX] = {
	USB_DESC_INTERFACE(8, CIC_ENDP_COUNT, cic5_ep),
	USB_DESC_INTERFACE(9, DIC_ENDP_COUNT, dic5_ep),
};
#endif

/* Configuration */

static usb_interfaces_struct_t usb_CDC_configuration[USB_CDC_CFG_MAX] = {
	USB_DESC_CONFIGURATION(USB_CDC_IF_MAX, usb1_if),

#if MIC_USB_CDC_INF_COUNT > 1
	USB_DESC_CONFIGURATION(USB_CDC_IF_MAX, usb2_if),
#endif
#if MIC_USB_CDC_INF_COUNT > 2
	USB_DESC_CONFIGURATION(USB_CDC_IF_MAX, usb3_if),
#endif
#if MIC_USB_CDC_INF_COUNT > 3
	USB_DESC_CONFIGURATION(USB_CDC_IF_MAX, usb4_if),
#endif
#if MIC_USB_CDC_INF_COUNT > 4
	USB_DESC_CONFIGURATION(USB_CDC_IF_MAX, usb5_if),
#endif
};

static usb_class_struct_t usb_dec_class[USB_CDC_CLASS_MAX] =
{
	{
		USB_CLASS_COMMUNICATION,
		USB_DESC_CONFIGURATION(USB_CDC_IF_MAX, usb1_if),
	},
#if MIC_USB_CDC_INF_COUNT > 1
	{
		USB_CLASS_COMMUNICATION,
		USB_DESC_CONFIGURATION(USB_CDC_IF_MAX, usb2_if),
	},
#endif
#if MIC_USB_CDC_INF_COUNT > 2
	{
		USB_CLASS_COMMUNICATION,
		USB_DESC_CONFIGURATION(USB_CDC_IF_MAX, usb3_if),
	},
#endif
#if MIC_USB_CDC_INF_COUNT > 3
	{
		USB_CLASS_COMMUNICATION,
		USB_DESC_CONFIGURATION(USB_CDC_IF_MAX, usb4_if),
	},
#endif
#if MIC_USB_CDC_INF_COUNT > 4
	{
		USB_CLASS_COMMUNICATION,
		USB_DESC_CONFIGURATION(USB_CDC_IF_MAX, usb5_if),
	},
#endif
};

static usb_composite_info_struct_t g_usb_composite_info = {
	MIC_USB_CDC_INF_COUNT, // Count of devices in composite device
	usb_dec_class
};

uint8_t g_device_descriptor[DEVICE_DESCRIPTOR_SIZE] =
{
	/* "Device Descriptor Size */
	DEVICE_DESCRIPTOR_SIZE,
	/* "Device" Type of descriptor */
	USB_DEVICE_DESCRIPTOR,
	/*  BCD USB version  */
	USB_uint_16_low(BCD_USB_VERSION), USB_uint_16_high(BCD_USB_VERSION),
	/* Device Class is indicated in the interface descriptors */
	DEVICE_DESC_DEVICE_CLASS,
	/*  Device Subclass is indicated in the interface descriptors  */
	DEVICE_DESC_DEVICE_SUBCLASS,
	/*  Device Protocol  */
	DEVICE_DESC_DEVICE_PROTOCOL,
	/* Max Packet size */
	CONTROL_MAX_PACKET_SIZE,
	/* Vendor ID */
	0xa2, 0x15,
	/* Product ID */
	(0x00 + MIC_USB_CDC_INF_COUNT), 0x03,
	/* BCD Device version */
	0x00, 0x02,
	/* Manufacturer string index */
	0x01,
	/* Product string index */
	0x02,
	/*  Serial number string index */
	0x00,
	/*  Number of configurations */
	DEVICE_DESC_NUM_CONFIG_SUPPORTED
};

uint8_t g_config_descriptor[CONFIG_DESC_SIZE] =
{
	CONFIG_ONLY_DESC_SIZE,              /*  Configuration Descriptor Size - always 9 bytes*/
	USB_CONFIG_DESCRIPTOR,              /* "Configuration" type of descriptor */
	USB_uint_16_low(CONFIG_DESC_SIZE),
	USB_uint_16_high(CONFIG_DESC_SIZE), /*  Total length of the Configuration descriptor */
	/*  NumInterfaces */
	CONFIG_DESC_NUM_INTERFACES_SUPPORTED,
	0x01,                               /*  Configuration Value */
	0x00,                               /*  Configuration Description String Index*/
	/*  Attributes.support RemoteWakeup and self power */
	(USB_DESC_CFG_ATTRIBUTES_D7_POS) | (USBCFG_DEV_SELF_POWER << USB_DESC_CFG_ATTRIBUTES_SELF_POWERED_SHIFT) | (USBCFG_DEV_REMOTE_WAKEUP << USB_DESC_CFG_ATTRIBUTES_REMOTE_WAKEUP_SHIFT),
	/*  Current draw from bus */
	CONFIG_DESC_CURRENT_DRAWN,

	/* Interface Association Descriptor */
	IAD_CONFIG_DESC_SIZE,                   /* Size of this descriptor */
	USB_IFACE_ASSOCIATION_DESCRIPTOR,       /* INTERFACE ASSOCIATION Descriptor */
	0x00,                                   /* Interface number of the CDC Control interface that is associated with this function */
	0x02,                                   /* Number of contiguous CDC interfaces that are associated with this function */
	CDC_CLASS,                              /* CDC_CC */
	DEVICE_DESC_DEVICE_SUBCLASS,
	DEVICE_DESC_DEVICE_PROTOCOL,
	0x00,                                   /* Index of string */

	/* CIC INTERFACE DESCRIPTOR */
	IFACE_ONLY_DESC_SIZE,
	USB_IFACE_DESCRIPTOR,
	0x00,                                   /* bInterfaceNumber */
	0x00,                                   /* bAlternateSetting */
	CIC_ENDP_COUNT,                         /* management and notification(optional)element present */
	CDC_CLASS,                              /* Communication Interface Class */
	CIC_SUBCLASS_CODE,
	CIC_PROTOCOL_CODE,
	0x00, /* Interface Description String Index*/

	/* CDC Class-Specific descriptor */
	CDC_HEADER_FUNC_DESC_SIZE,              /* size of Functional Desc in bytes */
	USB_CS_INTERFACE,                       /* descriptor type*/
	HEADER_FUNC_DESC,
	0x10, 0x01,                             /* USB Class Definitions for CDC spec release number in BCD */

	CDC_CALL_MANAG_DESC_SIZE,               /* Size of this descriptor */
	USB_CS_INTERFACE,                       /* descriptor type*/
	CALL_MANAGEMENT_FUNC_DESC,
	CM_D0_D1,                                   /*D0(if set): device handles call management itself
											D1(if set): process commands multiplexed over the data interface*/
	0x01,                                   /* Indicates multiplexed commands are handled via data interface */

	CDC_ABSTRACT_DESC_SIZE,                 /* Size of this descriptor */
	USB_CS_INTERFACE,                       /* descriptor type*/
	ABSTRACT_CONTROL_FUNC_DESC,
	0x06,                                   /* Device supports request send break, device supports request
											 combination o set_line_coding, set_control_line_state, 
											 get_line_coding and the notification serial state */

	CDC_UNION_FUNC_DESC_SIZE,               /* size of Functional Desc in bytes */
	USB_CS_INTERFACE,                       /* descriptor type*/
	UNION_FUNC_DESC,
	0x00,                                   /* Interface Number of Control */
	0x01                                    /* Interface Number of Subordinate (Data Class) Interface */

#if CIC_NOTIF_ELEM_SUPPORT                  /*Endpoint descriptor */
	, /* Comma Added if NOTIF ELEM IS TO BE ADDED */
	ENDP_ONLY_DESC_SIZE,
	USB_ENDPOINT_DESCRIPTOR,
	CIC1_NOTIF_ENDPOINT | (USB_SEND << 7),
	USB_INTERRUPT_PIPE,
	USB_uint_16_low(CIC_NOTIF_ENDP_PACKET_SIZE),
	USB_uint_16_high(CIC_NOTIF_ENDP_PACKET_SIZE),
	CIC_NOTIF_ENDP_INTERVAL
#endif

#if DATA_CLASS_SUPPORT
	, /* Comma Added if DATA_CLASS_DESC IS TO BE ADDED */
	IFACE_ONLY_DESC_SIZE,
	USB_IFACE_DESCRIPTOR,
	(uint8_t)(0x00 + DATA_CLASS_SUPPORT),     /* bInterfaceNumber */
	0x00,                                   /* bAlternateSetting */
	DIC_ENDP_COUNT,                         /* notification element included */
	0x0A,                                   /* DATA Interface Class */
	0x00,                                   /* Data Interface SubClass Code */
	DIC_PROTOCOL_CODE,
	0x00, /* Interface Description String Index*/

	/*Endpoint descriptor */
	ENDP_ONLY_DESC_SIZE,
	USB_ENDPOINT_DESCRIPTOR,
	DIC1_BULK_IN_ENDPOINT | (USB_SEND << 7),
	USB_BULK_PIPE,
	USB_uint_16_low(DIC_BULK_IN_ENDP_PACKET_SIZE),
	USB_uint_16_high(DIC_BULK_IN_ENDP_PACKET_SIZE),
	0x00,                                   /* This value is ignored for Bulk ENDPOINT */

	/*Endpoint descriptor */
	ENDP_ONLY_DESC_SIZE,
	USB_ENDPOINT_DESCRIPTOR,
	DIC1_BULK_OUT_ENDPOINT | (USB_RECV << 7),
	USB_BULK_PIPE,
	USB_uint_16_low(DIC_BULK_OUT_ENDP_PACKET_SIZE),
	USB_uint_16_high(DIC_BULK_OUT_ENDP_PACKET_SIZE),
	0x00 /* This value is ignored for Bulk ENDPOINT */
#endif

#if MIC_USB_CDC_INF_COUNT > 1
	, /* Interface Association Descriptor */
	IAD_CONFIG_DESC_SIZE,                   /* Size of this descriptor */
	USB_IFACE_ASSOCIATION_DESCRIPTOR,       /* INTERFACE ASSOCIATION Descriptor */
	0x02,                                   /* Interface number of the CDC Control
											interface that is associated with this function */
	0x02,                                   /* Number of contiguous CDC interfaces
											that are associated with this function */
	CDC_CLASS,                              /* CDC_CC */
	DEVICE_DESC_DEVICE_SUBCLASS,
	DEVICE_DESC_DEVICE_PROTOCOL,
	0x00,                                   /* Index of string */

	/* CIC INTERFACE DESCRIPTOR */
	IFACE_ONLY_DESC_SIZE,
	USB_IFACE_DESCRIPTOR,
	0x02,                                   /* bInterfaceNumber */
	0x00,                                   /* bAlternateSetting */
	CIC_ENDP_COUNT,                         /* management and notification(optional)element present */
	CDC_CLASS,                              /* Communication Interface Class */
	CIC_SUBCLASS_CODE,
	CIC_PROTOCOL_CODE,
	0x00, 									/* Interface Description String Index*/

	/* CDC Class-Specific descriptor */
	CDC_HEADER_FUNC_DESC_SIZE,              /* size of Functional Desc in bytes */
	USB_CS_INTERFACE,                       /* descriptor type*/
	HEADER_FUNC_DESC,
	0x10, 0x01,                             /* USB Class Definitions for CDC spec release number in BCD */

	CDC_CALL_MANAG_DESC_SIZE,               /* Size of this descriptor */
	USB_CS_INTERFACE,                       /* descriptor type*/
	CALL_MANAGEMENT_FUNC_DESC,
	CM_D0_D1,                                   /*D0(if set): device handles call management itself
											D1(if set): process commands multiplexed over the data interface*/
	0x01,                                   /* Indicates multiplexed commands are handled via data interface */

	CDC_ABSTRACT_DESC_SIZE,                 /* Size of this descriptor */
	USB_CS_INTERFACE,                       /* descriptor type*/
	ABSTRACT_CONTROL_FUNC_DESC,
	0x06,                                   /* Device supports request send break, device supports request
											 combination o set_line_coding, set_control_line_state, 
											 get_line_coding and the notification serial state */

	CDC_UNION_FUNC_DESC_SIZE,               /* size of Functional Desc in bytes */
	USB_CS_INTERFACE,                       /* descriptor type*/
	UNION_FUNC_DESC,
	0x02,                                   /* Interface Number of Control */
	0x03                                    /* Interface Number of Subordinate (Data Class) Interface */

#if CIC_NOTIF_ELEM_SUPPORT                  /*Endpoint descriptor */
	, /* Comma Added if NOTIF ELEM IS TO BE ADDED */
	ENDP_ONLY_DESC_SIZE,
	USB_ENDPOINT_DESCRIPTOR,
	CIC2_NOTIF_ENDPOINT | (USB_SEND << 7),
	USB_INTERRUPT_PIPE,
	USB_uint_16_low(CIC_NOTIF_ENDP_PACKET_SIZE),
	USB_uint_16_high(CIC_NOTIF_ENDP_PACKET_SIZE),
	CIC_NOTIF_ENDP_INTERVAL
#endif

#if DATA_CLASS_SUPPORT
	, /* Comma Added if DATA_CLASS_DESC IS TO BE ADDED */
	IFACE_ONLY_DESC_SIZE,
	USB_IFACE_DESCRIPTOR,
	(uint8_t)(0x02 + DATA_CLASS_SUPPORT),     	/* bInterfaceNumber */
	0x00,                                   	/* bAlternateSetting */
	DIC_ENDP_COUNT,                         	/* notification element included */
	0x0A,                                   	/* DATA Interface Class */
	0x00,                                   	/* Data Interface SubClass Code */
	DIC_PROTOCOL_CODE,
	0x00, /* Interface Description String Index*/

	/*Endpoint descriptor */
	ENDP_ONLY_DESC_SIZE,
	USB_ENDPOINT_DESCRIPTOR,
	DIC2_BULK_IN_ENDPOINT | (USB_SEND << 7),
	USB_BULK_PIPE,
	USB_uint_16_low(DIC_BULK_IN_ENDP_PACKET_SIZE),
	USB_uint_16_high(DIC_BULK_IN_ENDP_PACKET_SIZE),
	0x00, /* This value is ignored for Bulk ENDPOINT */

	/*Endpoint descriptor */
	ENDP_ONLY_DESC_SIZE,
	USB_ENDPOINT_DESCRIPTOR,
	DIC2_BULK_OUT_ENDPOINT | (USB_RECV << 7),
	USB_BULK_PIPE,
	USB_uint_16_low(DIC_BULK_OUT_ENDP_PACKET_SIZE),
	USB_uint_16_high(DIC_BULK_OUT_ENDP_PACKET_SIZE),
	0x00 /* This value is ignored for Bulk ENDPOINT */
#endif
#endif //MIC_USB_CDC_INF_COUNT > 1

// CAN 1 chanell
#if MIC_USB_CDC_INF_COUNT > 2
	, /* Interface Association Descriptor */
	IAD_CONFIG_DESC_SIZE,                   /* Size of this descriptor */
	USB_IFACE_ASSOCIATION_DESCRIPTOR,       /* INTERFACE ASSOCIATION Descriptor */
	0x04,                                   /* Interface number of the CDC Control
											interface that is associated with this function */
	0x02,                                   /* Number of contiguous CDC interfaces
											that are associated with this function */
	CDC_CLASS,                              /* CDC_CC */
	DEVICE_DESC_DEVICE_SUBCLASS_CAN,
	DEVICE_DESC_DEVICE_PROTOCOL,
	0x00,                                   /* Index of string */

	/* CIC INTERFACE DESCRIPTOR */
	IFACE_ONLY_DESC_SIZE,
	USB_IFACE_DESCRIPTOR,
	0x04,                                   /* bInterfaceNumber */
	0x00,                                   /* bAlternateSetting */
	CIC_ENDP_COUNT,                         /* management and notification(optional)element present */
	CDC_CLASS,                              /* Communication Interface Class */
	CIC_SUBCLASS_CODE,
	AT_250_PROTOCOL,
	0x00, /* Interface Description String Index*/

	/* CDC Class-Specific descriptor */
	CDC_HEADER_FUNC_DESC_SIZE,              /* size of Functional Desc in bytes */
	USB_CS_INTERFACE,                       /* descriptor type*/
	HEADER_FUNC_DESC,
	0x10, 0x01,                             /* USB Class Definitions for CDC spec release number in BCD */

	//Call Managment Functional Descriptor
	CDC_CALL_MANAG_DESC_SIZE,               /* Size of this descriptor */
	USB_CS_INTERFACE,                       /* descriptor type*/
	CALL_MANAGEMENT_FUNC_DESC,
	CM_DO_D1_CAN,                           /*D0(if set): device handles call management itself
											D1(if set): process commands multiplexed over the data interface*/
	0x01,                                   /* Indicates multiplexed commands are handled via data interface */

	//ACM Functional descriptor
	CDC_ABSTRACT_DESC_SIZE,                 /* Size of this descriptor */
	USB_CS_INTERFACE,                       /* descriptor type*/
	ABSTRACT_CONTROL_FUNC_DESC,
	0x02,                                   /* Device supports request send break, device supports request
											 combination o set_line_coding, set_control_line_state, 
											 get_line_coding and the notification serial state */

	//Union Functional Descriptor
	CDC_UNION_FUNC_DESC_SIZE,               /* size of Functional Desc in bytes */
	USB_CS_INTERFACE,                       /* descriptor type*/
	UNION_FUNC_DESC,
	0x04,                                   /* Interface Number of Control */
	0x05                                    /* Interface Number of Subordinate (Data Class) Interface */

#if CIC_NOTIF_ELEM_SUPPORT                  /*Endpoint descriptor */
	, /* Comma Added if NOTIF ELEM IS TO BE ADDED */
	ENDP_ONLY_DESC_SIZE,
	USB_ENDPOINT_DESCRIPTOR,
	CIC3_NOTIF_ENDPOINT | (USB_SEND << 7),
	USB_INTERRUPT_PIPE,
	USB_uint_16_low(CIC_NOTIF_ENDP_PACKET_SIZE),
	USB_uint_16_high(CIC_NOTIF_ENDP_PACKET_SIZE),
	CIC_NOTIF_ENDP_INTERVAL
#endif

#if DATA_CLASS_SUPPORT
	, /* Comma Added if DATA_CLASS_DESC IS TO BE ADDED */
	IFACE_ONLY_DESC_SIZE,
	USB_IFACE_DESCRIPTOR,
	(uint8_t)(0x04 + DATA_CLASS_SUPPORT),     /* bInterfaceNumber */
	0x00,                                   /* bAlternateSetting */
	DIC_ENDP_COUNT,                         /* notification element included */
	0x0A,                                   /* DATA Interface Class */
	0x00,                                   /* Data Interface SubClass Code */
	DIC_PROTOCOL_CODE,
	0x00, /* Interface Description String Index*/

	/*Endpoint descriptor */
	ENDP_ONLY_DESC_SIZE,
	USB_ENDPOINT_DESCRIPTOR,
	DIC3_BULK_IN_ENDPOINT | (USB_SEND << 7),
	USB_BULK_PIPE,
	USB_uint_16_low(DIC_BULK_IN_ENDP_PACKET_SIZE),
	USB_uint_16_high(DIC_BULK_IN_ENDP_PACKET_SIZE),
	0x00, /* This value is ignored for Bulk ENDPOINT */

	/*Endpoint descriptor */
	ENDP_ONLY_DESC_SIZE,
	USB_ENDPOINT_DESCRIPTOR,
	DIC3_BULK_OUT_ENDPOINT | (USB_RECV << 7),
	USB_BULK_PIPE,
	USB_uint_16_low(DIC_BULK_OUT_ENDP_PACKET_SIZE),
	USB_uint_16_high(DIC_BULK_OUT_ENDP_PACKET_SIZE),
	0x00 /* This value is ignored for Bulk ENDPOINT */
#endif
#endif //MIC_USB_CDC_INF_COUNT > 2

// CAN 2 chanell
#if MIC_USB_CDC_INF_COUNT > 3
	, /* Interface Association Descriptor */
	IAD_CONFIG_DESC_SIZE,                   /* Size of this descriptor */
	USB_IFACE_ASSOCIATION_DESCRIPTOR,       /* INTERFACE ASSOCIATION Descriptor */
	0x06,                                   /* Interface number of the CDC Control
											interface that is associated with this function */
	0x02,                                   /* Number of contiguous CDC interfaces
											that are associated with this function */
	CDC_CLASS,                              /* CDC_CC */
	DEVICE_DESC_DEVICE_SUBCLASS,
	DEVICE_DESC_DEVICE_PROTOCOL,
	0x00,                                   /* Index of string */

	/* CIC INTERFACE DESCRIPTOR */
	IFACE_ONLY_DESC_SIZE,
	USB_IFACE_DESCRIPTOR,
	0x06,                                   /* bInterfaceNumber */
	0x00,                                   /* bAlternateSetting */
	CIC_ENDP_COUNT,                         /* management and notification(optional)element present */
	CDC_CLASS,                              /* Communication Interface Class */
	CIC_SUBCLASS_CODE,
	CIC_PROTOCOL_CODE,
	0x00, /* Interface Description String Index*/

	/* CDC Class-Specific descriptor */
	CDC_HEADER_FUNC_DESC_SIZE,              /* size of Functional Desc in bytes */
	USB_CS_INTERFACE,                       /* descriptor type*/
	HEADER_FUNC_DESC,
	0x10, 0x01,                             /* USB Class Definitions for CDC spec release number in BCD */

	CDC_CALL_MANAG_DESC_SIZE,               /* Size of this descriptor */
	USB_CS_INTERFACE,                       /* descriptor type*/
	CALL_MANAGEMENT_FUNC_DESC,
	CM_D0_D1,                                   /*D0(if set): device handles call management itself
											D1(if set): process commands multiplexed over the data interface*/
	0x01,                                   /* Indicates multiplexed commands are handled via data interface */

	CDC_ABSTRACT_DESC_SIZE,                 /* Size of this descriptor */
	USB_CS_INTERFACE,                       /* descriptor type*/
	ABSTRACT_CONTROL_FUNC_DESC,
	0x06,                                   /* Device supports request send break, device supports request
											 combination o set_line_coding, set_control_line_state, 
											 get_line_coding and the notification serial state */

	CDC_UNION_FUNC_DESC_SIZE,               /* size of Functional Desc in bytes */
	USB_CS_INTERFACE,                       /* descriptor type*/
	UNION_FUNC_DESC,
	0x06,                                   /* Interface Number of Control */
	0x07                                    /* Interface Number of Subordinate (Data Class) Interface */

#if CIC_NOTIF_ELEM_SUPPORT                  /*Endpoint descriptor */
	, /* Comma Added if NOTIF ELEM IS TO BE ADDED */
	ENDP_ONLY_DESC_SIZE,
	USB_ENDPOINT_DESCRIPTOR,
	CIC4_NOTIF_ENDPOINT | (USB_SEND << 7),
	USB_INTERRUPT_PIPE,
	USB_uint_16_low(CIC_NOTIF_ENDP_PACKET_SIZE),
	USB_uint_16_high(CIC_NOTIF_ENDP_PACKET_SIZE),
	CIC_NOTIF_ENDP_INTERVAL
#endif

#if DATA_CLASS_SUPPORT
	, /* Comma Added if DATA_CLASS_DESC IS TO BE ADDED */
	IFACE_ONLY_DESC_SIZE,
	USB_IFACE_DESCRIPTOR,
	(uint8_t)(0x06 + DATA_CLASS_SUPPORT),     /* bInterfaceNumber */
	0x00,                                   /* bAlternateSetting */
	DIC_ENDP_COUNT,                         /* notification element included */
	0x0A,                                   /* DATA Interface Class */
	0x00,                                   /* Data Interface SubClass Code */
	DIC_PROTOCOL_CODE,
	0x00, /* Interface Description String Index*/

	/*Endpoint descriptor */
	ENDP_ONLY_DESC_SIZE,
	USB_ENDPOINT_DESCRIPTOR,
	DIC4_BULK_IN_ENDPOINT | (USB_SEND << 7),
	USB_BULK_PIPE,
	USB_uint_16_low(DIC_BULK_IN_ENDP_PACKET_SIZE),
	USB_uint_16_high(DIC_BULK_IN_ENDP_PACKET_SIZE),
	0x00, /* This value is ignored for Bulk ENDPOINT */

	/*Endpoint descriptor */
	ENDP_ONLY_DESC_SIZE,
	USB_ENDPOINT_DESCRIPTOR,
	DIC4_BULK_OUT_ENDPOINT | (USB_RECV << 7),
	USB_BULK_PIPE,
	USB_uint_16_low(DIC_BULK_OUT_ENDP_PACKET_SIZE),
	USB_uint_16_high(DIC_BULK_OUT_ENDP_PACKET_SIZE),
	0x00 /* This value is ignored for Bulk ENDPOINT */
#endif
#endif //MIC_USB_CDC_INF_COUNT > 3

// J1708 chanell
#if MIC_USB_CDC_INF_COUNT > 4
	, /* Interface Association Descriptor */
	IAD_CONFIG_DESC_SIZE,                   /* Size of this descriptor */
	USB_IFACE_ASSOCIATION_DESCRIPTOR,       /* INTERFACE ASSOCIATION Descriptor */
	0x08,                                   /* Interface number of the CDC Control
											interface that is associated with this function */
	0x02,                                   /* Number of contiguous CDC interfaces
											that are associated with this function */
	CDC_CLASS,                              /* CDC_CC */
	DEVICE_DESC_DEVICE_SUBCLASS,
	DEVICE_DESC_DEVICE_PROTOCOL,
	0x00,                                   /* Index of string */

	/* CIC INTERFACE DESCRIPTOR */
	IFACE_ONLY_DESC_SIZE,
	USB_IFACE_DESCRIPTOR,
	0x08,                                   /* bInterfaceNumber */
	0x00,                                   /* bAlternateSetting */
	CIC_ENDP_COUNT,                         /* management and notification(optional)element present */
	CDC_CLASS,                              /* Communication Interface Class */
	CIC_SUBCLASS_CODE,
	CIC_PROTOCOL_CODE,
	0x00, /* Interface Description String Index*/

	/* CDC Class-Specific descriptor */
	CDC_HEADER_FUNC_DESC_SIZE,              /* size of Functional Desc in bytes */
	USB_CS_INTERFACE,                       /* descriptor type*/
	HEADER_FUNC_DESC,
	0x10, 0x01,                             /* USB Class Definitions for CDC spec release number in BCD */

	CDC_CALL_MANAG_DESC_SIZE,               /* Size of this descriptor */
	USB_CS_INTERFACE,                       /* descriptor type*/
	CALL_MANAGEMENT_FUNC_DESC,
	CM_D0_D1,                                   /*D0(if set): device handles call management itself
											D1(if set): process commands multiplexed over the data interface*/
	0x01,                                   /* Indicates multiplexed commands are handled via data interface */

	CDC_ABSTRACT_DESC_SIZE,                 /* Size of this descriptor */
	USB_CS_INTERFACE,                       /* descriptor type*/
	ABSTRACT_CONTROL_FUNC_DESC,
	0x06,                                   /* Device supports request send break, device supports request
											 combination o set_line_coding, set_control_line_state, 
											 get_line_coding and the notification serial state */

	CDC_UNION_FUNC_DESC_SIZE,               /* size of Functional Desc in bytes */
	USB_CS_INTERFACE,                       /* descriptor type*/
	UNION_FUNC_DESC,
	0x08,                                   /* Interface Number of Control */
	0x09                                    /* Interface Number of Subordinate (Data Class) Interface */

#if CIC_NOTIF_ELEM_SUPPORT                  /*Endpoint descriptor */
	, /* Comma Added if NOTIF ELEM IS TO BE ADDED */
	ENDP_ONLY_DESC_SIZE,
	USB_ENDPOINT_DESCRIPTOR,
	CIC5_NOTIF_ENDPOINT | (USB_SEND << 7),
	USB_INTERRUPT_PIPE,
	USB_uint_16_low(CIC_NOTIF_ENDP_PACKET_SIZE),
	USB_uint_16_high(CIC_NOTIF_ENDP_PACKET_SIZE),
	CIC_NOTIF_ENDP_INTERVAL
#endif

#if DATA_CLASS_SUPPORT
	, /* Comma Added if DATA_CLASS_DESC IS TO BE ADDED */
	IFACE_ONLY_DESC_SIZE,
	USB_IFACE_DESCRIPTOR,
	(uint8_t)(0x08 + DATA_CLASS_SUPPORT),     /* bInterfaceNumber */
	0x00,                                   /* bAlternateSetting */
	DIC_ENDP_COUNT,                         /* notification element included */
	0x0A,                                   /* DATA Interface Class */
	0x00,                                   /* Data Interface SubClass Code */
	DIC_PROTOCOL_CODE,
	0x00, /* Interface Description String Index*/

	/*Endpoint descriptor */
	ENDP_ONLY_DESC_SIZE,
	USB_ENDPOINT_DESCRIPTOR,
	DIC5_BULK_IN_ENDPOINT | (USB_SEND << 7),
	USB_BULK_PIPE,
	USB_uint_16_low(DIC_BULK_IN_ENDP_PACKET_SIZE),
	USB_uint_16_high(DIC_BULK_IN_ENDP_PACKET_SIZE),
	0x00, /* This value is ignored for Bulk ENDPOINT */

	/*Endpoint descriptor */
	ENDP_ONLY_DESC_SIZE,
	USB_ENDPOINT_DESCRIPTOR,
	DIC5_BULK_OUT_ENDPOINT | (USB_RECV << 7),
	USB_BULK_PIPE,
	USB_uint_16_low(DIC_BULK_OUT_ENDP_PACKET_SIZE),
	USB_uint_16_high(DIC_BULK_OUT_ENDP_PACKET_SIZE),
	0x00 /* This value is ignored for Bulk ENDPOINT */
#endif
#endif //MIC_USB_CDC_INF_COUNT > 4
};

#if HIGH_SPEED
uint8_t g_device_qualifier_descriptor[DEVICE_QUALIFIER_DESCRIPTOR_SIZE] =
{
	/* Device Qualifier Descriptor Size */
	DEVICE_QUALIFIER_DESCRIPTOR_SIZE,
	/* Type of Descriptor */
	USB_DEVQUAL_DESCRIPTOR,
	/*  BCD USB version  */
	USB_uint_16_low(BCD_USB_VERSION), USB_uint_16_high(BCD_USB_VERSION),
	/* bDeviceClass */
	DEVICE_DESC_DEVICE_CLASS,
	/* bDeviceSubClass */
	DEVICE_DESC_DEVICE_SUBCLASS,
	/* bDeviceProtocol */
	DEVICE_DESC_DEVICE_PROTOCOL,
	/* bMaxPacketSize0 */
	CONTROL_MAX_PACKET_SIZE,
	/* bNumConfigurations */
	DEVICE_OTHER_DESC_NUM_CONFIG_SUPPORTED,
	/* Reserved : must be zero */
	0x00
};

uint8_t g_other_speed_config_descriptor[OTHER_SPEED_CONFIG_DESCRIPTOR_SIZE] =
{
	CONFIG_ONLY_DESC_SIZE,
	/* This is a Other speed config descr */
	USB_OTHER_SPEED_DESCRIPTOR,
	/*  Total length of the Configuration descriptor */
	USB_uint_16_low(CONFIG_DESC_SIZE), USB_uint_16_high(CONFIG_DESC_SIZE),
	CONFIG_DESC_NUM_INTERFACES_SUPPORTED,
	/*value used to select this configuration : Configuration Value */
	1,
	/*  Configuration Description String Index*/
	0,
	/*Attributes.support RemoteWakeup and self power*/
	(USB_DESC_CFG_ATTRIBUTES_D7_POS) | (USBCFG_DEV_SELF_POWER << USB_DESC_CFG_ATTRIBUTES_SELF_POWERED_SHIFT) | (USBCFG_DEV_REMOTE_WAKEUP << USB_DESC_CFG_ATTRIBUTES_REMOTE_WAKEUP_SHIFT),
	/*  Current draw from bus */
	CONFIG_DESC_CURRENT_DRAWN,

	/* CIC INTERFACE DESCRIPTOR */
	IFACE_ONLY_DESC_SIZE,
	USB_IFACE_DESCRIPTOR,
	0x00, /* bInterfaceNumber */
	0x00, /* bAlternateSetting */
	CIC_ENDP_COUNT, /* management and notification(optional)element present */
	CDC_CLASS, /* Communication Interface Class */
	CIC_SUBCLASS_CODE,
	CIC_PROTOCOL_CODE,
	0x00, /* Interface Description String Index*/

	/* CDC Class-Specific descriptor */
	CDC_HEADER_FUNC_DESC_SIZE, /* size of Functional Desc in bytes */
	USB_CS_INTERFACE, /* descriptor type*/
	HEADER_FUNC_DESC,
	0x10, 0x01, /* USB Class Definitions for CDC spec release number in BCD */

	CDC_CALL_MANAG_DESC_SIZE, /* Size of this descriptor */
	USB_CS_INTERFACE, /* descriptor type*/
	CALL_MANAGEMENT_FUNC_DESC,
	CM_D0_D1, /*D0(if set): device handles call management itself
	  D1(if set): process commands multiplexed over the data interface*/
	0x01, /* Indicates multiplexed commands are handled via data interface */

	CDC_ABSTRACT_DESC_SIZE, /* Size of this descriptor */
	USB_CS_INTERFACE, /* descriptor type*/
	ABSTRACT_CONTROL_FUNC_DESC,
	0x06, /* Device supports request send break, device supports request
	  combination o set_line_coding, set_control_line_state, 
	  get_line_coding and the notification serial state */

	CDC_UNION_FUNC_DESC_SIZE, /* size of Functional Desc in bytes */
	USB_CS_INTERFACE, /* descriptor type*/
	UNION_FUNC_DESC,
	0x00, /* Interface Number of Control */
	0x01 /* Interface Number of Subordinate (Data Class) Interface */

#if CIC_NOTIF_ELEM_SUPPORT    /*Endpoint descriptor */
	, /* Comma Added if NOTIF ELEM IS TO BE ADDED */
	ENDP_ONLY_DESC_SIZE,
	USB_ENDPOINT_DESCRIPTOR,
	CIC_NOTIF_ENDPOINT | (USB_SEND << 7),
	USB_INTERRUPT_PIPE,
	USB_uint_16_low(CIC_NOTIF_ENDP_PACKET_SIZE),
	USB_uint_16_high(CIC_NOTIF_ENDP_PACKET_SIZE),
	0x0A
#endif

#if DATA_CLASS_SUPPORT
	, /* Comma Added if DATA_CLASS_DESC IS TO BE ADDED */
	IFACE_ONLY_DESC_SIZE,
	USB_IFACE_DESCRIPTOR,
	(uint8_t)(0x00 + DATA_CLASS_SUPPORT), /* bInterfaceNumber */
	0x00, /* bAlternateSetting */
	DIC_ENDP_COUNT, /* notification element included */
	0x0A, /* DATA Interface Class */
	0x00, /* Data Interface SubClass Code */
	DIC_PROTOCOL_CODE,
	0x00, /* Interface Description String Index*/

	/*Endpoint descriptor */
	ENDP_ONLY_DESC_SIZE,
	USB_ENDPOINT_DESCRIPTOR,
	DIC_BULK_IN_ENDPOINT | (USB_SEND << 7),
	USB_BULK_PIPE,
	USB_uint_16_low(OTHER_SPEED_DIC_BULK_IN_ENDP_PACKET_SIZE),
	USB_uint_16_high(OTHER_SPEED_DIC_BULK_IN_ENDP_PACKET_SIZE),
	0x00, /* This value is ignored for Bulk ENDPOINT */

	/*Endpoint descriptor */
	ENDP_ONLY_DESC_SIZE,
	USB_ENDPOINT_DESCRIPTOR,
	DIC_BULK_OUT_ENDPOINT | (USB_RECV << 7),
	USB_BULK_PIPE,
	USB_uint_16_low(OTHER_SPEED_DIC_BULK_OUT_ENDP_PACKET_SIZE),
	USB_uint_16_high(OTHER_SPEED_DIC_BULK_OUT_ENDP_PACKET_SIZE),
	0x00 /* This value is ignored for Bulk ENDPOINT */
#endif
};
#endif

uint8_t USB_STR_0[] = { //[USB_STR_0_SIZE + USB_STR_DESC_SIZE] = {
	0x00, //sizeof(USB_STR_0),
	USB_STRING_DESCRIPTOR,
	0x09,
	0x04 /*equivalent to 0x0409*/
};

uint8_t USB_STR_1[] = { //[USB_STR_1_SIZE + USB_STR_DESC_SIZE] = {
	0x00, //sizeof(USB_STR_1),
	USB_STRING_DESCRIPTOR,
	'M', 0,
	'I', 0,
	'C', 0,
	'R', 0,
	'O', 0,
	'N', 0,
	'E', 0,
	'T', 0,
	' ', 0,
	'L', 0,
	'T', 0,
	'D', 0,
	'.', 0
};

uint8_t USB_STR_2[] = { //[USB_STR_2_SIZE + USB_STR_DESC_SIZE] = { 	
	0x00,//sizeof(USB_STR_2),
	USB_STRING_DESCRIPTOR,
	'M', 0,
	'C', 0,
	'U', 0,
	'-', 0,
	'V', 0,
	'I', 0,
	'R', 0,
	'T', 0,
	'U', 0,
	'A', 0,
	'L', 0,
	'-', 0,
	'C', 0,
	'O', 0,
	'M', 0,
	'S', 0,
	'.', 0
};

uint8_t USB_STR_n[] = { //[USB_STR_n_SIZE + USB_STR_DESC_SIZE] = 
 	0x00,//sizeof(USB_STR_n),
	USB_STRING_DESCRIPTOR,
	'B', 0,
	'A', 0,
	'D', 0,
	' ', 0,
	'S', 0,
	'T', 0,
	'R', 0,
	'I', 0,
	'N', 0,
	'G', 0
};

uint16_t g_std_desc_size[USB_MAX_STD_DESCRIPTORS + 1] =
{
	0,
	DEVICE_DESCRIPTOR_SIZE,
	CONFIG_DESC_SIZE,
	0, /* string */
	0, /* Interface */
	0, /* Endpoint */
#if HIGH_SPEED
	DEVICE_QUALIFIER_DESCRIPTOR_SIZE,
	OTHER_SPEED_CONFIG_DESCRIPTOR_SIZE
#else
	0, /* Device Qualifier */
	0 /* other speed config */
#endif
};

uint8_t *g_std_descriptors[USB_MAX_STD_DESCRIPTORS + 1] = {
	NULL,
	g_device_descriptor,
	g_config_descriptor,
	NULL, /* string */
	NULL, /* Interface */
	NULL, /* Endpoint */
#if HIGH_SPEED
	g_device_qualifier_descriptor,
	g_other_speed_config_descriptor
#else
	NULL, /* Device Qualifier */
	NULL /* other speed config*/
#endif
};

uint8_t g_string_desc_size[USB_MAX_STRING_DESCRIPTORS + 1] = { 	
	sizeof(USB_STR_0),
	sizeof(USB_STR_1),
	sizeof(USB_STR_2),
	sizeof(USB_STR_n)
};

uint8_t *g_string_descriptors[USB_MAX_STRING_DESCRIPTORS + 1] = {
	USB_STR_0,
	USB_STR_1,
	USB_STR_2,
	USB_STR_n
};

usb_language_t usb_lang[USB_MAX_LANGUAGES_SUPPORTED] =
{
	{
		(uint16_t)0x0409,
		g_string_descriptors,
		g_string_desc_size,
	},
};

usb_all_languages_t g_languages = { 
	USB_STR_0, 
	sizeof(USB_STR_0),
	USB_MAX_LANGUAGES_SUPPORTED,
	usb_lang
};

static uint8_t g_valid_config_values[USB_MAX_CONFIG_SUPPORTED + 1] = { 0, 1 };

/****************************************************************************
 * Global Variables
 ****************************************************************************/
static uint8_t g_alternate_interface[USB_MAX_SUPPORTED_INTERFACES];

/*****************************************************************************
 * Local Types - None
 *****************************************************************************/

/*****************************************************************************
 * Local Functions Prototypes 
 *****************************************************************************/

/*****************************************************************************
 * Local Variables - None
 *****************************************************************************/

/*****************************************************************************
 * Local Functions - None
 *****************************************************************************/

/*****************************************************************************
 * Global Functions
 *****************************************************************************/

/**************************************************************************//*!
*
* @name  USB_Desc_Get_Descriptor
*
* @brief The function returns the corresponding descriptor
*
* @param handle:        handle     
* @param type:          type of descriptor requested     
* @param str_num:       string index for string descriptor     
* @param index:         string descriptor language Id     
* @param descriptor:    output descriptor pointer
* @param size:          size of descriptor returned
*
* @return USB_OK                              When Success
*         USBERR_INVALID_REQ_TYPE             when Error
*****************************************************************************/
uint8_t USB_Desc_Get_Descriptor( uint32_t handle,
									uint8_t type,
									uint8_t str_num,
									uint16_t index,
									uint8_t **descriptor,
									uint32_t *size )
{
	UNUSED_ARGUMENT(handle)

	/* string descriptors are handled separately */
	if ( type == USB_STRING_DESCRIPTOR )
	{
		if ( index == 0 )
		{
				/* return the string and size of all languages */
#ifdef MIC_DESCR_MEM
				*descriptor = (uint8_t *)g_MIC_g_languages->languages_supported_string;
				*size = g_MIC_g_languages->languages_supported_size;
#else
				*descriptor = (uint8_t *)g_languages.languages_supported_string;
				*size = g_languages.languages_supported_size;
#endif
		} else
		{
			uint8_t lang_id;
			uint8_t lang_index = USB_MAX_STRING_DESCRIPTORS;

			for ( lang_id = 0; lang_id < USB_MAX_LANGUAGES_SUPPORTED; lang_id++ )
			{
					/* check whether we have a string for this language */
#ifdef MIC_DESCR_MEM
				if ( index == g_MIC_g_languages->usb_language[lang_id].language_id )
#else
				if ( index == g_languages.usb_language[lang_id].language_id )
#endif
				{ /* check for max descriptors */
					if ( str_num < USB_MAX_STRING_DESCRIPTORS )
					{ /* setup index for the string to be returned */
						lang_index = (str_num < USB_MAX_STRING_DESCRIPTORS) ? str_num : USB_MAX_STRING_DESCRIPTORS;
					}
					break;
				}
			}
		/* if a language match was found return the sring, otherwise stall the endpoint */
			if ( lang_id < USB_MAX_LANGUAGES_SUPPORTED )
			{
			/* set return val for descriptor and size */
#ifdef MIC_DESCR_MEM
				*descriptor = (uint8_t *)g_MIC_g_languages->usb_language[lang_id].lang_desc[lang_index];
				*size = g_MIC_g_languages->usb_language[lang_id].lang_desc_size[lang_index];
#else
				*descriptor = (uint8_t *)g_languages.usb_language[lang_id].lang_desc[lang_index];
				*size = g_languages.usb_language[lang_id].lang_desc_size[lang_index];
#endif
			}
			else
			{
				/* stall the endpoint in case of request error */
				return USBERR_INVALID_REQ_TYPE;
			}
		}
	} 
	else if ( type < USB_MAX_STD_DESCRIPTORS + 1 )
	{
		/* set return val for descriptor and size*/
#ifdef MIC_DESCR_MEM
		*descriptor = (uint8_t *)g_MIC_g_std_descriptors[type];
#else
		*descriptor = (uint8_t *)g_std_descriptors[type];
#endif
		/* if there is no descriptor then return error */
		if ( *descriptor == NULL )
		{
			return USBERR_INVALID_REQ_TYPE;
		}
		*size = g_std_desc_size[type];
	} 
	else /* invalid descriptor */
	{
		return USBERR_INVALID_REQ_TYPE;
	}
	return USB_OK;
}

/**************************************************************************//*!
 *
 * @name  USB_Desc_Get_Interface
 *
 * @brief The function returns the alternate interface
 *
 * @param handle:         handle     
 * @param interface:      interface number     
 * @param alt_interface:  output alternate interface     
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Desc_Get_Interface( uint32_t handle,
								uint8_t interface,
								uint8_t *alt_interface )
{
	UNUSED_ARGUMENT(handle)

	/* if interface valid */
	if ( interface < USB_MAX_SUPPORTED_INTERFACES )
	{
			/* get alternate interface*/
#ifdef MIC_DESCR_MEM
		*alt_interface = g_MIC_g_alternate_interface[interface];
#else
		*alt_interface = g_alternate_interface[interface];
#endif
		return USB_OK;
	}

	return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Desc_Set_Interface
 *
 * @brief The function sets the alternate interface
 *
 * @param handle:         handle     
 * @param interface:      interface number     
 * @param alt_interface:  input alternate interface     
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Desc_Set_Interface( uint32_t handle,
									uint8_t interface,
									uint8_t alt_interface )
{
	UNUSED_ARGUMENT(handle)

	/* if interface valid */
	if ( interface < USB_MAX_SUPPORTED_INTERFACES )
	{
			/* set alternate interface*/
#ifdef MIC_DESCR_MEM
		g_MIC_g_alternate_interface[interface] = alt_interface;
#else
		g_alternate_interface[interface] = alt_interface;
#endif
		return USB_OK;
	}

	return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Desc_Valid_Configation
 *
 * @brief The function checks whether the configuration parameter 
 *        input is valid or not
 *
 * @param handle          handle    
 * @param config_val      configuration value     
 *
 * @return TRUE           When Valid
 *         FALSE          When Error
 *****************************************************************************/
bool USB_Desc_Valid_Configation( uint32_t handle, uint16_t config_val )
{
	uint8_t loop_index = 0;

	UNUSED_ARGUMENT(handle)

	/* check with only supported val right now */
	while ( loop_index < (USB_MAX_CONFIG_SUPPORTED + 1) )
	{
		if ( config_val == g_valid_config_values[loop_index] )
		{
			return TRUE;
		}
		loop_index++;
	}
	return FALSE;
}

/**************************************************************************//*!
 *
 * @name  USB_Desc_Remote_Wakeup
 *
 * @brief The function checks whether the remote wakeup is supported or not
 *
 * @param handle:        handle     
 *
 * @return USBCFG_DEV_REMOTE_WAKEUP (TRUE) - if remote wakeup supported
 *****************************************************************************/
bool USB_Desc_Remote_Wakeup( uint32_t handle )
{
	UNUSED_ARGUMENT(handle)
	return USBCFG_DEV_REMOTE_WAKEUP;
}

/**************************************************************************//*!
 *
 * @name  USB_Set_Configuration
 *
 * @brief The function set the configuration value of device
 *
 *
 * @param handle          handle
 * @param config_val      configuration value
 *
 * @return TRUE           When Valid
 *         FALSE          When Error
 *****************************************************************************/
uint8_t USB_Set_Configuration ( cdc_handle_t handle, uint8_t config )
{
	UNUSED_ARGUMENT(handle)
	uint32_t i;

#ifdef MIC_DESCR_MEM
	if ( config )
	{
		g_MIC_usb_dec_class[0].interfaces = g_MIC_usb_configuration[config - 1]; /*config num starts from 1*/
	}
#else
	for ( i = 0; i < g_usb_composite_info.count; i++ )
	{
		switch ( g_usb_composite_info.class_handle[i].type )
		{
		case USB_CLASS_COMMUNICATION:
			usb_dec_class[i].interfaces = usb_CDC_configuration[i];
			break;
		default:
			break;
		}
	}
#endif

	return USB_OK;
}

/**************************************************************************//*!
 *
 * @name  USB_Desc_Get_Entity
 *
 * @brief The function retrieves the entity specified by type.
 *
 * @param handle            handle
 *
 * @return USB_OK  - if success
 *****************************************************************************/
uint8_t USB_Desc_Get_Entity( cdc_handle_t handle, entity_type type, uint32_t *object )
{
	uint8_t i;
	//UNUSED_ARGUMENT(handle)
	switch ( type )
	{
	case USB_CLASS_INFO:
/*
#ifdef MIC_DESCR_MEM                             
		*object = (uint32_t) g_MIC_usb_dec_class;
#else                                            
		*object = (uint32_t) usb_dec_class;      
#endif                                           
*/
		break;
	case USB_CLASS_INTERFACE_INDEX_INFO:
		*object = 0xff;

		for ( i = 0; i < MIC_USB_CDC_INF_COUNT; i++ )
		{
			if ( handle == g_app_composite_device.cdc_vcom[i].cdc_handle )
			{
				*object = (uint32_t)i;
				break;
			}
		}
		break;
	case USB_COMPOSITE_INFO:
		*object = (uint32_t)&g_usb_composite_info;
		break;
	default:
		break;
	} /* End Switch */
		return USB_OK;
}

/**************************************************************************//*!
 *
 * @name  USB_Desc_Set_Speed
 *
 * @brief The function is used to set device speed
 *
 * @param handle:         handle     
 * @param speed:          speed    
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Desc_Set_Speed ( uint32_t handle, uint16_t speed )
{
	descriptor_union_t ptr1, ptr2;
#if DATA_CLASS_SUPPORT
	uint16_t bulk_in = 0;
	uint16_t bulk_out = 0;
#endif
#if CIC_NOTIF_ELEM_SUPPORT
	uint16_t interrupt_size = 0;
	uint8_t interrupt_interval = 0;
#endif
	//UNUSED_ARGUMENT (handle);

	if ( USB_SPEED_HIGH == speed )
	{
#if DATA_CLASS_SUPPORT
		bulk_in = HS_DIC_BULK_IN_ENDP_PACKET_SIZE;
		bulk_out = HS_DIC_BULK_OUT_ENDP_PACKET_SIZE;
#endif
#if CIC_NOTIF_ELEM_SUPPORT
		interrupt_size = HS_CIC_NOTIF_ENDP_PACKET_SIZE;
		interrupt_interval = HS_CIC_NOTIF_ENDP_INTERVAL;
#endif
	}
	else
	{
#if DATA_CLASS_SUPPORT
		bulk_in = FS_DIC_BULK_IN_ENDP_PACKET_SIZE;
		bulk_out = FS_DIC_BULK_OUT_ENDP_PACKET_SIZE;
#endif
#if CIC_NOTIF_ELEM_SUPPORT
		interrupt_size = FS_CIC_NOTIF_ENDP_PACKET_SIZE;
		interrupt_interval = FS_CIC_NOTIF_ENDP_INTERVAL;
#endif
	}

	ptr1.pntr = g_config_descriptor;
	ptr2.pntr = g_config_descriptor + CONFIG_DESC_SIZE;

	while ( ptr1.word < ptr2.word )
	{
		if ( ptr1.common->bDescriptorType == USB_DESC_TYPE_EP )
		{
			if ( (CIC1_NOTIF_ENDPOINT == (ptr1.ndpt->bEndpointAddress & 0x7F))
#if MIC_USB_CDC_INF_COUNT > 1
					 || (CIC2_NOTIF_ENDPOINT == (ptr1.ndpt->bEndpointAddress & 0x7F))
#endif
#if MIC_USB_CDC_INF_COUNT > 2
					 || (CIC3_NOTIF_ENDPOINT == (ptr1.ndpt->bEndpointAddress & 0x7F))
#endif
#if MIC_USB_CDC_INF_COUNT > 3
					 || (CIC4_NOTIF_ENDPOINT == (ptr1.ndpt->bEndpointAddress & 0x7F))
#endif
#if MIC_USB_CDC_INF_COUNT > 4
					 || (CIC5_NOTIF_ENDPOINT == (ptr1.ndpt->bEndpointAddress & 0x7F))
#endif
					)
			{
#if CIC_NOTIF_ELEM_SUPPORT
				ptr1.ndpt->wMaxPacketSize[0] = USB_uint_16_low(interrupt_size);
				ptr1.ndpt->wMaxPacketSize[1] = USB_uint_16_high(interrupt_size);
				ptr1.ndpt->iInterval = interrupt_interval;
#endif
			} else
			{
#if DATA_CLASS_SUPPORT
				if ( ptr1.ndpt->bEndpointAddress & 0x80 )
				{
					ptr1.ndpt->wMaxPacketSize[0] = USB_uint_16_low(bulk_in);
					ptr1.ndpt->wMaxPacketSize[1] = USB_uint_16_high(bulk_in);
				} else
				{
					ptr1.ndpt->wMaxPacketSize[0] = USB_uint_16_low(bulk_out);
					ptr1.ndpt->wMaxPacketSize[1] = USB_uint_16_high(bulk_out);
				}
#endif
			}
		}
		ptr1.word += ptr1.common->bLength;
	}
#if CIC_NOTIF_ELEM_SUPPORT
		cic1_ep[0].size = interrupt_size;
#if MIC_USB_CDC_INF_COUNT > 1
		cic2_ep[0].size = interrupt_size;
#endif
#if MIC_USB_CDC_INF_COUNT > 2
		cic3_ep[0].size = interrupt_size;
#endif
#if MIC_USB_CDC_INF_COUNT > 3
		cic4_ep[0].size = interrupt_size;
#endif
#if MIC_USB_CDC_INF_COUNT > 4
		cic5_ep[0].size = interrupt_size;
#endif
#endif

#if DATA_CLASS_SUPPORT
	for ( int i = 0; i < DIC_ENDP_COUNT; i++ )
	{
		if ( USB_SEND == dic1_ep[i].direction )
		{
			dic1_ep[i].size = bulk_in;
		} else
		{
			dic1_ep[i].size = bulk_out;
		}
#if MIC_USB_CDC_INF_COUNT > 1
		if ( USB_SEND == dic2_ep[i].direction )
		{
			dic2_ep[i].size = bulk_in;
		} else
		{
			dic2_ep[i].size = bulk_out;
		}
#endif
#if MIC_USB_CDC_INF_COUNT > 2
		if ( USB_SEND == dic3_ep[i].direction )
		{
			dic3_ep[i].size = bulk_in;
		} else
		{
			dic3_ep[i].size = bulk_out;
		}
#endif
#if MIC_USB_CDC_INF_COUNT > 3
		if ( USB_SEND == dic4_ep[i].direction )
		{
			dic4_ep[i].size = bulk_in;
		} else
		{
			dic4_ep[i].size = bulk_out;
		}
#endif
#if MIC_USB_CDC_INF_COUNT > 4
		if ( USB_SEND == dic5_ep[i].direction )
		{
			dic5_ep[i].size = bulk_in;
		} else
		{
			dic5_ep[i].size = bulk_out;
		}
#endif
	}
#endif
	return USB_OK;
}

usb_desc_request_notify_struct_t desc_callback =
{
	USB_Desc_Get_Descriptor,
	USB_Desc_Get_Interface,
	USB_Desc_Set_Interface,
	USB_Set_Configuration,
	USB_Desc_Get_Entity
};

void USB_init_memory_Desc( )
{
#ifdef MIC_DESCR_MEM
	g_MIC_usb_dec_class = (usb_class_struct_t *)_mem_alloc(sizeof(usb_dec_class));
	if ( NULL != g_MIC_usb_dec_class )
	{
		_mem_copy((void *)g_MIC_usb_dec_class, (void *)usb_dec_class, sizeof(usb_dec_class));
	}
	g_MIC_g_languages = (usb_all_languages_t *)_mem_alloc(sizeof(g_languages));
	if ( NULL != g_MIC_g_languages )
	{
		_mem_copy((void *)g_MIC_g_languages, (void *)&g_languages, sizeof(g_languages));
	}

	g_MIC_g_device_descriptor = (uint8_t *)_mem_alloc(sizeof(g_device_descriptor));
	if ( NULL != g_MIC_g_device_descriptor )
	{
		_mem_copy((void *)g_MIC_g_device_descriptor, (void *)g_device_descriptor, sizeof(g_device_descriptor));
	}
	g_MIC_g_config_descriptor = (uint8_t *)_mem_alloc(sizeof(g_config_descriptor));
	if ( NULL != g_MIC_g_config_descriptor )
	{
		_mem_copy((void *)g_MIC_g_config_descriptor, (void *)g_config_descriptor, sizeof(g_config_descriptor));
	}
#if HIGH_SPEED
	g_MIC_g_device_qualifier_descriptor = (uint8_t *)_mem_alloc(sizeof(g_device_qualifier_descriptor));
	if ( NULL != g_MIC_g_device_qualifier_descriptor )
	{
		_mem_copy((void *)g_MIC_g_device_qualifier_descriptor, (void *)g_device_qualifier_descriptor, sizeof(g_device_qualifier_descriptor));
	}
	g_MIC_g_other_speed_config_descriptor = (uint8_t *)_mem_alloc(sizeof(g_other_speed_config_descriptor));
	if ( NULL != g_MIC_g_other_speed_config_descriptor )
	{
		_mem_copy((void *)g_MIC_g_other_speed_config_descriptor, (void *)g_other_speed_config_descriptor, sizeof(g_other_speed_config_descriptor));
	}
#endif


	g_MIC_g_std_descriptors = (uint8_t **)_mem_alloc_zero(sizeof(g_std_descriptors));
	if ( NULL != g_MIC_g_std_descriptors )
	{
		g_MIC_g_std_descriptors[1] = g_MIC_g_device_descriptor;
		g_MIC_g_std_descriptors[2] = g_MIC_g_config_descriptor;
#if HIGH_SPEED
		g_MIC_g_std_descriptors[6] = g_MIC_g_device_qualifier_descriptor;
		g_MIC_g_std_descriptors[7] = g_MIC_g_other_speed_config_descriptor;
#endif
	}
	g_MIC_g_alternate_interface = (uint8_t *)_mem_alloc(sizeof(g_alternate_interface));
	if ( NULL != g_MIC_g_alternate_interface )
	{
		_mem_copy((void *)g_MIC_g_alternate_interface, (void *)g_alternate_interface, sizeof(g_alternate_interface));
	}

	g_MIC_usb_configuration = (usb_interfaces_struct_t *)_mem_alloc(sizeof(usb_configuration));
	if ( NULL != g_MIC_usb_configuration )
	{
		_mem_copy((void *)g_MIC_usb_configuration, (void *)usb_configuration, sizeof(usb_configuration));
	}
#endif
}

void USB_prepare_descroptors ( void )
{
	USB_STR_0[0] = sizeof( USB_STR_0 );
	USB_STR_1[0] = sizeof ( USB_STR_1 );
	USB_STR_2[0] = sizeof ( USB_STR_2 );
	USB_STR_n[0] = sizeof ( USB_STR_n );
}

/* EOF */
	
