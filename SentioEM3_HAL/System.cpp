/*
 * system.cpp
 *
 *  Created on: Feb 17, 2011
 *      Author: Matthias Krämer
 */

#include "efm32_gpio.h"
#include "efm32_cmu.h"
#include "efm32_emu.h"
#include "efm32_rtc.h"

#include "Statemachine.h"
#include "System.h"


/****************************************************************************************************************************************//**
 * @brief
 *  The method initializes microcontroller clock and IO-pins. The specification is done within the SystemConfig.h
 *
 *******************************************************************************************************************************************/

void System::initializeSentioEM()
{
#if SentioEM_CPU_Clock_MHZ == 32
	CMU_OscillatorEnable( cmuOsc_HFXO, true, true );
	CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFXO );
	CMU_OscillatorEnable( cmuOsc_HFRCO, false, false );
#elif SentioEM_CPU_Clock_MHZ == 1
	CMU_HFRCOBandSet( cmuHFRCOBand_1MHz );
	CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFRCO );
	CMU_OscillatorEnable( cmuOsc_HFXO, false, false );
#elif SentioEM_CPU_Clock_MHZ == 7
	CMU_HFRCOBandSet( cmuHFRCOBand_7MHz );
	CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFRCO );
	CMU_OscillatorEnable( cmuOsc_HFXO, false, false );
#elif SentioEM_CPU_Clock_MHZ == 11
	CMU_HFRCOBandSet( cmuHFRCOBand_11MHz );
	CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFRCO );
	CMU_OscillatorEnable( cmuOsc_HFXO, false, false );
#elif SentioEM_CPU_Clock_MHZ == 14
	CMU_HFRCOBandSet( cmuHFRCOBand_14MHz );
	CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFRCO );
	CMU_OscillatorEnable( cmuOsc_HFXO, false, false );
#elif SentioEM_CPU_Clock_MHZ == 21
	CMU_HFRCOBandSet( cmuHFRCOBand_21MHz );
	CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFRCO );
	CMU_OscillatorEnable( cmuOsc_HFXO, false, false );
#elif SentioEM_CPU_Clock_MHZ == 28
	CMU_HFRCOBandSet( cmuHFRCOBand_28MHz );
	CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFRCO );
	CMU_OscillatorEnable( cmuOsc_HFXO, false, false );
#endif

#if SentioEM_LF_Clock == INTERN
	CMU_ClockSelectSet( cmuClock_LFA, cmuSelect_LFRCO );

	CMU_ClockEnable( cmuClock_CORELE, true );
#endif

#if SentioEM_LF_Clock == EXTERN
	CMU->CTRL = CMU_CTRL_LFXOMODE_DIGEXTCLK;

#endif

	CMU_ClockEnable( cmuClock_GPIO, true );

#if SentioEM_OnBoard_LEDS == ON
	GPIO_PinModeSet( gpioPortC, 8, gpioModePushPull, 0 );
	GPIO_PinModeSet( gpioPortC, 9, gpioModePushPull, 0 );
	GPIO_PinModeSet( gpioPortC, 10, gpioModePushPull, 0 );
#endif

	// Disable SD Card Interface
	GPIO_PinModeSet( gpioPortA, 10, gpioModePushPull, 0 );


#if SentioEM_OnBoard_Button == ON
	GPIO_PinModeSet( gpioPortC, 11, gpioModeInputPullFilter, 1 );
	GPIO_IntConfig( gpioPortC, 11, true, false, true );
#endif


#if SentioEM_GPIO_Interrupt == ON || SentioEM_OnBoard_Button == ON || numberOfButtonsUsed > 0
	// Clear Pending Interrupts before enable
	GPIO->IFC = ~0;

	// Enable Interrupt Service Routines
	NVIC_EnableIRQ( GPIO_EVEN_IRQn );
	NVIC_EnableIRQ( GPIO_ODD_IRQn );
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The method enables the green LED on the SentioEM3.
 *
 *******************************************************************************************************************************************/

void System::LED_SetGreen()
{
#if SentioEM_OnBoard_LEDS == ON
	GPIO->P[gpioPortC].DOUTSET = 1 << 10;
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The method enables the orange LED on the SentioEM3.
 *
 *******************************************************************************************************************************************/

void System::LED_SetOrange()
{
#if SentioEM_OnBoard_LEDS == ON
	GPIO->P[gpioPortC].DOUTSET = 1 << 9;
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The method enables the red LED on the SentioEM3.
 *
 *******************************************************************************************************************************************/

void System::LED_SetRed()
{
#if SentioEM_OnBoard_LEDS == ON
	GPIO->P[gpioPortC].DOUTSET = 1 << 8;
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The method disables the green LED on the SentioEM3.
 *
 *******************************************************************************************************************************************/

void System::LED_ClearGreen()
{
#if SentioEM_OnBoard_LEDS == ON
	GPIO->P[gpioPortC].DOUTCLR = 1 << 10;
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The method disables the orange LED on the SentioEM3.
 *
 *******************************************************************************************************************************************/

void System::LED_ClearOrange()
{
#if SentioEM_OnBoard_LEDS == ON
	GPIO->P[gpioPortC].DOUTCLR = 1 << 9;
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The method disables the red LED on the SentioEM3.
 *
 *******************************************************************************************************************************************/

void System::LED_ClearRed()
{
#if SentioEM_OnBoard_LEDS == ON
	GPIO->P[gpioPortC].DOUTCLR = 1 << 8;
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The method toggles the green LED on the SentioEM3.
 *
 *******************************************************************************************************************************************/

void System::LED_ToggleGreen()
{
#if SentioEM_OnBoard_LEDS == ON
	GPIO->P[gpioPortC].DOUTTGL = 1 << 10;
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The method toggles the orange LED on the SentioEM3.
 *
 *******************************************************************************************************************************************/

void System::LED_ToggleOrange()
{
#if SentioEM_OnBoard_LEDS == ON
	GPIO->P[gpioPortC].DOUTTGL = 1 << 9;
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The method toggles the red LED on the SentioEM3.
 *
 *******************************************************************************************************************************************/

void System::LED_ToggleRed()
{
#if SentioEM_OnBoard_LEDS == ON
	GPIO->P[gpioPortC].DOUTTGL = 1 << 8;
#endif
}

