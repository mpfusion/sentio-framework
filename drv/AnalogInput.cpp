/*
 * AnalogInput.cpp
 *
 *  Created on: Nov 5, 2011
 *      Author: matthias
 */

#include "em_gpio.h"
#include "em_cmu.h"
#include "em_chip.h"

#include "AnalogInput.h"
#include "SystemConfig.h"


void AnalogInput::initializeInterface()
{
	CMU_ClockEnable( cmuClock_ADC0, true );

	ADC_Reset( ADC0 );

	init.lpfMode    = _INT_ADC_FILTER;
	init.ovsRateSel = _INT_ADC_OVERSAMPLE;
	init.tailgate   = false;
	init.warmUpMode = _INT_ADC_WARM_UP;
	init.timebase   = ADC_TimebaseCalc( 0 );
	init.prescale   = ADC_PrescaleCalc( 7000000, 0 );

	scan.reference  = _INT_ADC_REFERENCE;
	scan.input      = ( _INT_ADC_CH_0 | _INT_ADC_CH_1 | _INT_ADC_CH_2 | _INT_ADC_CH_3 | \
						_INT_ADC_CH_4 | _INT_ADC_CH_5 | _INT_ADC_CH_6 | _INT_ADC_CH_7 );
	scan.resolution = _INT_ADC_RESOLUTION;
	scan.acqTime    = _INT_ADC_ACQ_TIME;
	scan.diff       = false;
	scan.leftAdjust = false;
	scan.prsEnable  = false;
	scan.rep        = false;


	single.reference  = _INT_ADC_REFERENCE_SINGLE;
	single.input      = adcSingleInpCh0;
	single.resolution = _INT_ADC_RESOLUTION;
	single.acqTime    = _INT_ADC_ACQ_TIME;
	single.diff       = false;
	single.leftAdjust = false;
	single.prsEnable  = false;
	single.rep        = false;



	ADC_Init( ADC0, &init );
	ADC_InitScan( ADC0, &scan );
}


void AnalogInput::readMultipleChannels( float *data )
{
	ADC0->CAL = calirationValue;

	ADC_Start( ADC0, adcStartScan );

	for ( uint32_t i = 0; i < _INT_ADC_SCAN_CHANNELS; i++ )
	{
		while ( !( ADC0->STATUS & ADC_STATUS_SCANDV ) );
		data[i] = ( float )( ( ADC0->SCANDATA ) & 0x0000FFFF ) * _COEFFICIENT_SCAN / 65536;
	}
}


void AnalogInput::readChannel( ADC_SingleInput_TypeDef channel, float &data )
{
	ADC0->CAL = calirationValue;
	single.input = channel;

	ADC_InitSingle( ADC0, &single );

	ADC_Start( ADC0, adcStartSingle );

	while ( ADC0->STATUS & ADC_STATUS_SINGLEACT );

	data = ( float )( ADC0->SINGLEDATA ) * _COEFFICIENT_SINGLE / 65536;
}


void AnalogInput::readMultipleChannels( uint16_t *data )
{
	ADC0->CAL = calirationValue;

	ADC_Start( ADC0, adcStartScan );

	for ( uint32_t i = 0; i < _INT_ADC_SCAN_CHANNELS; i++ )
	{
		while ( !( ADC0->STATUS & ADC_STATUS_SCANDV ) );
		data[i] = ( uint16_t ) ADC0->SCANDATA;
	}
}


void AnalogInput::readChannel( ADC_SingleInput_TypeDef channel, uint16_t &data )
{
	ADC0->CAL = calirationValue;
	single.input = channel;

	ADC_InitSingle( ADC0, &single );

	ADC_Start( ADC0, adcStartSingle );

	while ( ADC0->STATUS & ADC_STATUS_SINGLEACT );

	data = ( uint16_t ) ADC0->SINGLEDATA;
}


void AnalogInput::calibrateReference()
{
	int32_t  sample;
	uint32_t cal;

	/* Binary search variables */
	uint8_t high;
	uint8_t mid;
	uint8_t low;

	/* Reset ADC to be sure we have default settings and wait for ongoing */
	/* conversions to be complete. */
	ADC_Reset( ADC0 );

	ADC_Init_TypeDef       l_init       = ADC_INIT_DEFAULT;
	ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

	/* Init common settings for both single conversion and scan mode */
	l_init.timebase = ADC_TimebaseCalc( 0 );
	/* Might as well finish conversion as quickly as possibly since polling */
	/* for completion. */
	/* Set ADC clock to 7 MHz, use default HFPERCLK */
	l_init.prescale = ADC_PrescaleCalc( 7000000, 0 );

	/* Set an oversampling rate for more accuracy */
	l_init.ovsRateSel = adcOvsRateSel4096;
	/* Leave other settings at default values */
	ADC_Init( ADC0, &l_init );

	/* Init for single conversion use, measure diff 0 with selected reference. */
	singleInit.reference = _INT_ADC_REFERENCE;
	singleInit.input     = _INT_ADC_CALIBRATION_OFFSET;
	singleInit.acqTime   = adcAcqTime16;
	singleInit.diff      = true;
	/* Enable oversampling rate */
	singleInit.resolution = adcResOVS;

	ADC_InitSingle( ADC0, &singleInit );



	/* ADC is now set up for offset calibration */
	/* Offset calibration register is a 7 bit signed 2's complement value. */
	/* Use unsigned indexes for binary search, and convert when calibration */
	/* register is written to. */
	high = 128;
	low  = 0;

	/* Do binary search for offset calibration*/
	while ( low < high )
	{
		/* Calculate midpoint */
		mid = low + ( high - low ) / 2;

		/* Midpoint is converted to 2's complement and written to both scan and */
		/* single calibration registers */
		cal      = ADC0->CAL & ~( _ADC_CAL_SINGLEOFFSET_MASK | _ADC_CAL_SCANOFFSET_MASK );
		cal     |= ( mid - 63 ) << _ADC_CAL_SINGLEOFFSET_SHIFT;
		cal     |= ( mid - 63 ) << _ADC_CAL_SCANOFFSET_SHIFT;
		ADC0->CAL = cal;

		/* Do a conversion */
		ADC_Start( ADC0, adcStartSingle );

		/* Wait while conversion is active */
		while ( ADC0->STATUS & ADC_STATUS_SINGLEACT ) ;

		/* Get ADC result */
		sample = ADC_DataSingleGet( ADC0 );

		/* Check result and decide in which part of to repeat search */
		/* Calibration register has negative effect on result */
		if ( sample < 0 )
		{
			/* Repeat search in bottom half. */
			high = mid;
		}
		else if ( sample > 0 )
		{
			/* Repeat search in top half. */
			low = mid + 1;
		}
		else
		{
			/* Found it, exit while loop */
			break;
		}
	}


	/* Now do gain calibration, only input and diff settings needs to be changed */
	ADC0->SINGLECTRL &= ~( _ADC_SINGLECTRL_INPUTSEL_MASK | _ADC_SINGLECTRL_DIFF_MASK );
	ADC0->SINGLECTRL |= ( adcSingleInpCh2 << _ADC_SINGLECTRL_INPUTSEL_SHIFT );
	ADC0->SINGLECTRL |= ( false << _ADC_SINGLECTRL_DIFF_SHIFT );

	/* ADC is now set up for gain calibration */
	/* Gain calibration register is a 7 bit unsigned value. */

	high = 128;
	low  = 0;

	/* Do binary search for gain calibration */
	while ( low < high )
	{
		/* Calculate midpoint and write to calibration register */
		mid = low + ( high - low ) / 2;

		/* Midpoint is converted to 2's complement */
		cal      = ADC0->CAL & ~( _ADC_CAL_SINGLEGAIN_MASK | _ADC_CAL_SCANGAIN_MASK );
		cal     |= mid << _ADC_CAL_SINGLEGAIN_SHIFT;
		cal     |= mid << _ADC_CAL_SCANGAIN_SHIFT;
		ADC0->CAL = cal;

		/* Do a conversion */
		ADC_Start( ADC0, adcStartSingle );

		/* Wait while conversion is active */
		while ( ADC0->STATUS & ADC_STATUS_SINGLEACT ) ;

		/* Get ADC result */
		sample = ADC_DataSingleGet( ADC0 );

		/* Check result and decide in which part to repeat search */
		/* Compare with a value atleast one LSB's less than top to avoid overshooting */
		/* Since oversampling is used, the result is 16 bits, but a couple of lsb's */
		/* applies to the 12 bit result value, if 0xffe is the top value in 12 bit, this */
		/* is in turn 0xffe0 in the 16 bit result. */
		/* Calibration register has positive effect on result */
		if ( sample > 0xffd0 )
		{
			/* Repeat search in bottom half. */
			high = mid;
		}
		else if ( sample < 0xffd0 )
		{
			/* Repeat search in top half. */
			low = mid + 1;
		}
		else
		{
			/* Found it, exit while loop */
			break;
		}
	}

	calirationValue = ADC0->CAL;

}

void AnalogInput::enableSensor( SENSORS input )
{
	switch ( input )
	{
	case radiationSensor:
		GPIO_PinOutSet( radiationSensorEnable );
		break;

	default:
		;
	}
}

void AnalogInput::disableSensor( SENSORS input )
{
	switch ( input )
	{
	case radiationSensor:
		GPIO_PinOutClear( radiationSensorEnable );
		break;

	default:
		;
	}
}

void AnalogInput::initializeSensorEN( SENSORS input )
{
	switch ( input )
	{
	case radiationSensor:
		GPIO_PinModeSet( radiationSensorEnable, gpioModePushPull, 0 );
		break;

	default:
		;
	}
}
