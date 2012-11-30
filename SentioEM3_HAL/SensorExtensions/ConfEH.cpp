/*
 * ConfEH.cpp
 *
 *  Created on: 16 nov 2012
 *      Author: sebastian
 */

#include "ConfEH.h"
#include "efm32_gpio.h"
#include "LTC2990.h"
#include "SystemConfig.h"


void CONFEH::initializeInterface()
{
	setResistor_1( false );
	setResistor_2( false );
	setResistor_3( false );

	setTXS0102_EN( true );

	initializeInterfaceLTC();
}

void CONFEH::setMode( uint8_t storage, uint8_t harvester )
{
	storageType = storage;
	harvestingMode = harvester;

	if ( harvester == DIRECT )
	{
		setLTC3105_EN( false );
		setMAX17710_AE( false );
		setMAX17710_LCE( false );

	}
	else if ( harvester == MAX17710 )
	{
		setLTC3105_EN( false );
		setMAX17710_AE( true );
		setMAX17710_LCE( false );
	}
	else if ( harvester == LTC3105 )
	{
		setLTC3105_EN( true );
		setMAX17710_AE( false );
		setMAX17710_LCE( false );
	}
	else if ( harvester == LTC3105 + MAX17710 )
	{
		setLTC3105_EN( true );
		setMAX17710_AE( true );
		setMAX17710_LCE( false );
	}
}

void CONFEH::getMeasurements( float &storageVoltage, float &solarVoltage, float &solarCurrent )
{
	// Configure LTC2990 for Current-Voltage-Voltage measurement (using 1 ohm sense resistor)
	setLTC_Config( celsius, single, V1_V2V3V4, 1, 0 );
	triggerConversion();

	// Read the storage voltage
	if ( storageType == DLC )
		readVoltage( storageVoltage, Voltage_V3 );
	else if ( storageType == BAT )
		readVoltage( storageVoltage, Voltage_V4 );

	// Read the solar current
	readCurrent( solarCurrent, Current_V12 );

	// Check for LTC3105 reset condition
	if ( solarCurrent < 0.001 )
	{
		// Reset LTC3105
		setLTC3105_EN( false );
		setLTC3105_EN( true );
	}

	// Re-configure LTC2990 for pure voltage measurement
	setLTC_Config( celsius, single, V1V2V3V4, 1, 0 );
	triggerConversion();
	// Read the solar voltage
	readVoltage( solarVoltage, Voltage_V1 );
}

void CONFEH::setTXS0102_EN( bool state )
{
	GPIO_PinModeSet( gpioPortE, 7, gpioModePushPull, state );
}

void CONFEH::setMAX17710_AE( bool state )
{
	GPIO_PinModeSet( gpioPortD, 4, gpioModePushPull, state );
}

void CONFEH::setMAX17710_LCE( bool state )
{
	GPIO_PinModeSet( gpioPortD, 5, gpioModePushPull, state );
}

void CONFEH::setLTC3105_EN( bool state )
{
	GPIO_PinModeSet( gpioPortE, 6, gpioModePushPull, state );
}

void CONFEH::setResistor_1( bool state )
{
	GPIO_PinModeSet( gpioPortE, 3, gpioModePushPull, state );
}

void CONFEH::setResistor_2( bool state )
{
	GPIO_PinModeSet( gpioPortE, 4, gpioModePushPull, state );
}

void CONFEH::setResistor_3( bool state )
{
	GPIO_PinModeSet( gpioPortE, 5, gpioModePushPull, state );
}
