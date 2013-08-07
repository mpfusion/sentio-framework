/*
 * system.cpp
 *
 *  Created on: Feb 17, 2011
 *      Author: Matthias Krämer
 */

#include "sentio_em.h"
#include "application_config.h"

#include "em_cmu.h"
#include "em_chip.h"

#if MCU_CLOCK_MHZ == 32
      #define OSC_ON cmuOsc_HFXO
#elif MCU_CLOCK_MHZ == 1
      #define OSC_FREQ cmuHFRCOBand_1MHz
#elif MCU_CLOCK_MHZ == 7
      #define OSC_FREQ cmuHFRCOBand_7MHz
#elif MCU_CLOCK_MHZ == 11
      #define OSC_FREQ cmuHFRCOBand_11MHz
#elif MCU_CLOCK_MHZ == 14
      #define OSC_FREQ cmuHFRCOBand_14MHz
#elif MCU_CLOCK_MHZ == 21
      #define OSC_FREQ cmuHFRCOBand_21MHz
#elif MCU_CLOCK_MHZ == 28
      #define OSC_FREQ cmuHFRCOBand_28MHz
#else 
      #error Invalid MCU_CLOCK in application_config
#endif

void SENTIO_EM::init()
{
	//Apply EFM32 chpi dependent Bug-Fixes
	CHIP_Init();
	
#if MCU_CLOCK_MHZ <= 28
	CMU_HFRCOBandSet( OSC_FREQ );
	CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFRCO );
	CMU_OscillatorEnable( cmuOsc_HFXO, false, false );
#else
	CMU_OscillatorEnable( OSC_ON, true, true );
	CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFXO );
	CMU_OscillatorEnable( cmuOsc_HFRCO, false, false );
#endif

	// Enable the GPIO-Pins of the EFM32 MCU
	CMU_ClockEnable( cmuClock_GPIO, true );
	// Disable SD Card Interface
	GPIO_PinModeSet( SD_PWR_PIN, gpioModePushPull, 0 );

#if LF_CLOCK
	CMU_ClockSelectSet( cmuClock_LFA, cmuSelect_LFRCO );
	CMU_ClockEnable( cmuClock_CORELE, true );
#endif

#if CLOCK_TEST_PINS
	CMU->ROUTE = CMU_ROUTE_LOCATION_LOC1 | CMU_ROUTE_CLKOUT1PEN | CMU_ROUTE_CLKOUT0PEN;

	GPIO_PinModeSet( CMU_LF_PIN, gpioModePushPull, 0 );
	GPIO_PinModeSet( CMU_HF_PIN, gpioModePushPull, 0 );
#endif


#if ONBOARD_LEDS
	GPIO_PinModeSet( RED,    gpioModePushPull, 0 );
	GPIO_PinModeSet( ORANGE, gpioModePushPull, 0 );
	GPIO_PinModeSet( GREEN,  gpioModePushPull, 0 );
#endif

#if ONBOARD_BUTTON
	GPIO_PinModeSet( BUTTON, gpioModeInputPullFilter, 1 );
	GPIO_IntConfig(  BUTTON, true, false, true );
#endif
}


void SENTIO_EM::setLED( int port, unsigned int pin )
{
#if ONBOARD_LEDS
	GPIO->P[port].DOUTSET = 1 << pin;
#endif  
}

void SENTIO_EM::tglLED( GPIO_Port_TypeDef port, unsigned int pin )
{
#if ONBOARD_LEDS
	GPIO->P[port].DOUTTGL = 1 << pin;
#endif
}

void SENTIO_EM::clrLED( GPIO_Port_TypeDef port, unsigned int pin )
{
#if ONBOARD_LEDS
	GPIO->P[port].DOUTCLR = 1 << pin;
#endif
}

