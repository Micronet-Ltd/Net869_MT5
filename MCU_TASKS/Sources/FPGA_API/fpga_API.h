#ifndef __fpga_api_h_
#define __fpga_api_h_

#include <mqx.h>

void FPGA_init         (void);
void FPGA_port_enable  (void);
void FPGA_port_disable (void);

/*****************************************************
* 	REGISTER LIST:                                   *
* 		1. VERSION                                   *
* 		2. IRQ STATUS                                *
* 		3. IRQ MASK                                  *
* 		4. LED2                                      *
* 		5. LED3                                      *
* 		6. J1708 CONTROL                             *
* 		7. J1708 TX LENGTH                           *
* 		8. J1708 RX LENGTH                           *
*****************************************************/



/*****************************************************
*                  VERSION Register                  *
*          API   Address      Data fields            *
*****************************************************/
#define FPGA_REG_ADDR_VERSION					0x00

#define FPGA_REG_VERSION_TYPE_SHIFT 			24
#define FPGA_REG_VERSION_MAJOR_SHIFT			16
#define FPGA_REG_VERSION_MINOR_SHIFT			 8
#define FPGA_REG_VERSION_DEBUG_SHIFT			 0

bool FPGA_read_version    (uint32_t *version);

/*****************************************************
*                IRQ STATUS Register                 *
*                IRQ MASK   Register                 *
*          API   Address      Data fields            *
*****************************************************/
#define FPGA_REG_ADDR_IRQ_STATUS				0x01
#define FPGA_REG_ADDR_IRQ_MASK					0x02

#define FPGA_REG_J1708_RX_IRQ_BIT				0x01

bool FPGA_read_irq_status (uint32_t *status);
bool FPGA_read_irq_mask   (uint32_t *status);
bool FPGA_set_irq         (uint32_t irqMask);
bool FPGA_clear_irq       (uint32_t irqMask);

/*****************************************************
*                  LED(x)  Register                  *
*          API   Address      Data fields            *
*****************************************************/
#define LED_RIGHT			                     0
#define LED_MIDDLE			                     1

#define FPGA_REG_ADDR_LED_MIDDLE				0x10
#define FPGA_REG_ADDR_LED_RIGHT					0x11

#define FPGA_REG_LED_BRIGHTNESS_SHIFT			24
#define FPGA_REG_LED_RED_SHIFT					16
#define FPGA_REG_LED_GREEN_SHIFT				 8
#define FPGA_REG_LED_BLUE_SHIFT					 0

bool FPGA_read_led_status  (uint8_t ledNum, uint8_t *brightness, uint8_t *red, uint8_t *green, uint8_t *blue);
bool FPGA_write_led_status (uint8_t ledNum, uint8_t *brightness, uint8_t *red, uint8_t *green, uint8_t *blue);

/*****************************************************
*              J1708 CONTROL  Register               *
*          API   Address      Data fields            *
*****************************************************/
#define FPGA_REG_ADDR_J1708_CONTROL				0x20

#define FPGA_REG_J1708_CONTROL_ENABLE_SHIFT		0
#define FPGA_REG_J1708_CONTROL_PRIORITY_SHIFT	4

#define FPGA_REG_J1708_CONTROL_ENABLE			(     1      << FPGA_REG_J1708_CONTROL_ENABLE_SHIFT  )
#define FPGA_REG_J1708_CONTROL_PRIO(x)			(((x) & 0x7) << FPGA_REG_J1708_CONTROL_PRIORITY_SHIFT)

bool FPGA_J1708_enable  (void);
bool FPGA_J1708_disable (void);

bool FPGA_read_J1708_status    (bool *status);
bool FPGA_read_J1708_priority  (uint8_t *priority);
bool FPGA_write_J1708_priority (uint8_t *priority);

/*****************************************************
*                J1708 LENGTH  Registers             *
*          API   Address      Data fields            *
*****************************************************/
#define FPGA_REG_ADDR_J1708_TX_LEN				0x21
#define FPGA_REG_ADDR_J1708_RX_LEN				0x22

#define FPGA_REG_J1708_LEN_SHIFT				   0
#define FPGA_REG_J1708_STATUS_SHIFT				  31

#define FPGA_REG_J1708_LEN(x)					(((x) & 0xFF) << FPGA_REG_J1708_LEN_SHIFT   )
#define FPGA_REG_J1708_STATUS(x)				(((x) & 0x01) << FPGA_REG_J1708_STATUS_SHIFT)

bool FPGA_read_J1708_rx_register (bool *status, uint8_t *len);
bool FPGA_read_J1708_tx_register (bool *status, uint8_t *len);
bool FPGA_write_J1708_tx_length  (uint8_t *len);

/*****************************************************
*                  J1708 Data Buffers API            *
*****************************************************/
bool FPGA_read_J1708_packet  (uint8_t *buffer, uint8_t length);
bool FPGA_write_J1708_packet (uint8_t *buffer, uint8_t length);


#endif  /* __fpga_api_h_ */
