/*
 * Copyright (c) 2013-2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#if 1
#include "board.h"
#include "fsl_clock_manager.h"
#include "fsl_smc_hal.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"

#include "mqx_prv.h"
#include "MK20D10_extension.h"

/* Configuration for enter VLPR mode. Core clock = 4MHz. */
const clock_manager_user_config_t g_defaultClockConfigVlpr =
{
    .mcgConfig =
    {
        .mcg_mode           = kMcgModeBLPI,   // Work in BLPI mode.
        .irclkEnable        = true,  // MCGIRCLK enable.
        .irclkEnableInStop  = false, // MCGIRCLK disable in STOP mode.
        .ircs               = kMcgIrcFast, // Select IRC4M.
        .fcrdiv             = 0U,    // FCRDIV is 0.

        .frdiv   = 0U,
        .drs     = kMcgDcoRangeSelLow,  // Low frequency range
        .dmx32   = kMcgDmx32Default,    // DCO has a default range of 25%
        .oscsel  = kMcgOscselOsc,       // Select OSC

        .pll0EnableInFllMode        = false,  // PLL0 disable
        .pll0EnableInStop  = false,  // PLL0 disalbe in STOP mode
        .prdiv0            = 0U,
        .vdiv0             = 0U,
    },
    .simConfig =
    {
        .pllFllSel = kClockPllFllSelFll, // PLLFLLSEL select FLL.
        .er32kSrc  = kClockEr32kSrcRtc,     // ERCLK32K selection, use RTC.
        .outdiv1   = 0U,
        .outdiv2   = 0U,
        .outdiv3   = 0U,
        .outdiv4   = 4U,
    },
    .oscerConfig =
    {
        .enable       = true,  // OSCERCLK enable.
        .enableInStop = false, // OSCERCLK disable in STOP mode.
    }
};

/* Configuration for enter RUN mode. Core clock = 96MHz. */
/* in device low power mode, configuration remains in RUN mode but Core clock = 4MHz */
const clock_manager_user_config_t g_defaultClockConfigRun =
{
    .mcgConfig =
    {
        .mcg_mode           = kMcgModePEE,   // Work in PEE mode.
        .irclkEnable        = true,  // MCGIRCLK enable.
        .irclkEnableInStop  = false, // MCGIRCLK disable in STOP mode.
        .ircs               = kMcgIrcSlow, // Select IRC32k.
        .fcrdiv             = 0U,    // FCRDIV is 0.

        .frdiv   = 5U,
        .drs     = kMcgDcoRangeSelLow,  // Low frequency range
        .dmx32   = kMcgDmx32Default,    // DCO has a default range of 25%
        .oscsel  = kMcgOscselOsc,       // Select OSC

        .pll0EnableInFllMode        = false,  // PLL0 disable
        .pll0EnableInStop  = false,  // PLL0 disalbe in STOP mode
        .prdiv0            = 12,	// support input clock of 26MHz
        .vdiv0             = (CORE_CLOCK_FREQ / 2000000) - 24,		// VDIV = CLK_FRQ / 2MHz - 24
    },
    .simConfig =
    {
        .pllFllSel = kClockPllFllSelPll,    // PLLFLLSEL select PLL.
        .er32kSrc  = kClockEr32kSrcRtc,     // ERCLK32K selection, use RTC.
        .outdiv1   = 0U,
        .outdiv2   = 1U,
        .outdiv3   = 1U,
        .outdiv4   = 3U,
    },
    .oscerConfig =
    {
        .enable       = true,  // OSCERCLK enable.
        .enableInStop = false, // OSCERCLK disable in STOP mode.
    }
};

// this function modifies the MCG configuration from FEI to BLPI
void Board_SetVerySlowClk (void)
{
	uint32_t mode = (uint32_t) CLOCK_HAL_GetMcgMode(MCG);

	//if ((mode & kMcgModeFEI) != kMcgModeFEI)
		Board_SetSlowClk ();

	CLOCK_SYS_BootToBlpi(&g_defaultClockConfigVlpr.mcgConfig);
	SystemCoreClock = CORE_CLOCK_FREQ;
	_bsp_MQX_tick_timer_init ();
}

// this function modifies the MCG configuration from FEI to PEE
void Board_SetFastClk (void)
{
	uint32_t mode = (uint32_t) CLOCK_HAL_GetMcgMode(MCG);

	//if ((mode & kMcgModeFEI) != kMcgModeFEI)
          //Board_SetSlowClk ();

	CLOCK_SYS_BootToPee(&g_defaultClockConfigRun.mcgConfig);
	SystemCoreClock = CORE_CLOCK_FREQ;
	_bsp_MQX_tick_timer_init ();
}

// this function modifies the MCG configuration from any state to FEI
void Board_SetSlowClk (void)
{
	uint32_t mode = (uint32_t) CLOCK_HAL_GetMcgMode(MCG);

	switch (mode) {
		case kMcgModeFEI :
		case kMcgModeFBI :
		case kMcgModeFEE :
		case kMcgModeFBE :	break;

		case kMcgModeBLPE :
		case kMcgModeBLPI :	MCG_WR_C2(MCG, MCG_RD_C2(MCG) &~ MCG_C2_LP_MASK);		// Set MCG mode to FBI \ FBE(clear LP)
							break;

		case kMcgModePEE :	MCG_WR_C1 (MCG, (MCG_RD_C1(MCG) & ~MCG_C1_IRCLKEN_MASK));	// Set MCG mode to PBE (disable MCGIRCLK)
                                        break;

		case kMcgModePBE :	// change MCG mode from PBE to FBE
							MCG_BWR_C1_CLKS(MCG, kMcgClkOutSrcExternal);
							while (CLOCK_HAL_GetClkOutStat(MCG)  != kMcgClkOutStatExternal) {}

							MCG_WR_C6(MCG, 0U);											// Disable PLL and select FLL
							while ((CLOCK_HAL_IsPllSelected(MCG) != false)) {}			// wait for PLLST status bit to clear

							MCG_WR_C5(MCG, 0U);											// Disable PLL
							break;

		default : return;																// all other cases are not supported
	}

    CLOCK_SYS_BootToFei(&g_defaultClockConfigVlpr.mcgConfig);
	//CLOCK_SYS_BootToBlpi(&g_defaultClockConfigVlpr.mcgConfig);
	SystemCoreClock = CORE_LPM_CLOCK_FREQ;
	_bsp_MQX_tick_timer_init ();
}



/* Function to initialize OSC0 base on board configuration. */
void BOARD_InitOsc0(void)
{
#if 1
    // OSC0 configuration.
    osc_user_config_t osc0Config =
    {
        .freq                = OSC0_XTAL_FREQ,
        .hgo                 = MCG_HGO0,
        .range               = MCG_RANGE0,
        .erefs               = MCG_EREFS0,
        .enableCapacitor2p   = OSC0_SC2P_ENABLE_CONFIG,
        .enableCapacitor4p   = OSC0_SC4P_ENABLE_CONFIG,
        .enableCapacitor8p   = OSC0_SC8P_ENABLE_CONFIG,
        .enableCapacitor16p  = OSC0_SC16P_ENABLE_CONFIG,
    };

    CLOCK_SYS_OscInit(0U, &osc0Config);
#endif
}

/* Function to initialize RTC external clock base on board configuration. */
void BOARD_InitRtcOsc(void)
{
#if 1
    rtc_osc_user_config_t rtcOscConfig =
    {
        .freq                = RTC_XTAL_FREQ,
        .enableCapacitor2p   = RTC_SC2P_ENABLE_CONFIG,
        .enableCapacitor4p   = RTC_SC4P_ENABLE_CONFIG,
        .enableCapacitor8p   = RTC_SC8P_ENABLE_CONFIG,
        .enableCapacitor16p  = RTC_SC16P_ENABLE_CONFIG,
        .enableOsc           = RTC_OSC_ENABLE_CONFIG,
    };

    CLOCK_SYS_RtcOscInit(0U, &rtcOscConfig);
#endif
}

static void CLOCK_SetBootConfig(clock_manager_user_config_t const* config)
{
    CLOCK_SYS_SetSimConfigration   (&config->simConfig);
    CLOCK_SYS_SetOscerConfigration (0, &config->oscerConfig);
    //Board_SetVerySlowClk ();
    //Board_SetSlowClk ();
    Board_SetFastClk();
#if (CLOCK_INIT_CONFIG == CLOCK_VLPR)
    CLOCK_SYS_BootToBlpi(&config->mcgConfig);
#else
    CLOCK_SYS_BootToPee(&config->mcgConfig);
#endif

    SystemCoreClock = CORE_CLOCK_FREQ;
}

/* Initialize clock. */
void BOARD_ClockInit(void)
{

    /* Set allowed power mode, allow all. */
    SMC_HAL_SetProtection(SMC, kAllowPowerModeAll);
#if 1
    /* Setup board clock source. */
    // Setup OSC0 if used.
    // Configure OSC0 pin mux.
    PORT_HAL_SetMuxMode(EXTAL0_PORT, EXTAL0_PIN, EXTAL0_PINMUX);
    PORT_HAL_SetMuxMode(XTAL0_PORT, XTAL0_PIN, XTAL0_PINMUX);
    BOARD_InitOsc0();

    // Setup RTC external clock if used.
    BOARD_InitRtcOsc();

    /* Set system clock configuration. */
#if (CLOCK_INIT_CONFIG == CLOCK_VLPR)
    CLOCK_SetBootConfig(&g_defaultClockConfigVlpr);
#else
    CLOCK_SetBootConfig(&g_defaultClockConfigRun);
#endif
#endif
}

/* The function to indicate whether a card is detected or not */
bool BOARD_IsSDCardDetected(void)
{
#if 0
    GPIO_Type * gpioBase = g_gpioBase[GPIO_EXTRACT_PORT(kGpioSdhc0Cd)];
    uint32_t pin = GPIO_EXTRACT_PIN(kGpioSdhc0Cd);

    if(GPIO_HAL_ReadPinInput(gpioBase, pin) == false)
    {
        return true;
    }
    else
    {
        return false;
    }
#endif
    return false;
}

void dbg_uart_init(void)
{
#if 1
    configure_uart_pins(BOARD_DEBUG_UART_INSTANCE);

    DbgConsole_Init(BOARD_DEBUG_UART_INSTANCE, BOARD_DEBUG_UART_BAUD, kDebugConsoleUART);
#endif
}
void update_fw_uart_init(void)
{
    configure_uart_pins(UART_UPDATE_FW_IDX);
}

/******************************************************************************
 *
 *   @name      usb_device_board_init
 *
 *   @brief     This function is to handle board-specified initialization
 *
 *   @param     controller_id:        refer to CONTROLLER_INDEX defined in usb_misc.h
 *                                    "0" stands for USB_CONTROLLER_KHCI_0.
 *   @return    status
 *                                    0 : successful
 *                                    1 : failed
 **
 *****************************************************************************/
uint8_t usb_device_board_init(uint8_t controller_id)
{

    int8_t ret = 0;
#if 0
    if (0 == controller_id)
    {
        /* TO DO */
        /*add board initialization code if have*/
    }
    else
    {
        ret = 1;
    }
#endif
    return ret;

}


/******************************************************************************
 *
 *   @name        usb_host_board_init
 *
 *   @brief       This function is to handle board-specified initialization
 *
 *   @param     controller_id:        refer to CONTROLLER_INDEX defined in usb_misc.h
 *                                    "0" stands for USB_CONTROLLER_KHCI_0.
 *   @return         status
 *                                    0 : successful
 *                                    1 : failed
 **
 *****************************************************************************/
uint8_t usb_host_board_init(uint8_t controller_id)
{
    int8_t ret = 0;
#if 0
    /*"0" stands for USB_CONTROLLER_KHCI_0 */
    if (0 == controller_id)
    {
        /* TO DO */
        /*add board initialization code if have*/
    }
    else
    {
       ret = 1;
    }
#endif
    return ret;


}
/******************************************************************************
 *
 *   @name      usb_otg_board_init
 *
 *   @brief     This function is to handle board-specified initialization
 *
 *   @param     controller_id:        refer to CONTROLLER_INDEX defined in usb_misc.h
 *                                    "0" stands for USB_CONTROLLER_KHCI_0
 *   @return    status
 *                                    0 : successful
 *                                    1 : failed
 **
 *****************************************************************************/
uint8_t usb_otg_board_init(uint8_t controller_id)
{
    uint8_t error = 0;
    /* TO DO */
    /*add board initialization code if have*/

    return error;
}
#endif
/*******************************************************************************
 * EOF
 ******************************************************************************/

