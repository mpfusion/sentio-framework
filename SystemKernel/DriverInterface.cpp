/*
 * DriverInterface.cpp
 *
 *  Created on: Nov 12, 2011
 *      Author: matthias
 */

#include "DriverInterface.h"
#include "SystemConfig.h"

DebugInterface  DriverInterface::debug;
RTC_DS3234      DriverInterface::timer;
System          DriverInterface::sentio;
SHT1X_Sensirion DriverInterface::humid;

void DriverInterface::codeError()
{
	for(ever)
	{
		sentio.LED_ToggleRed();

		for(volatile uint32_t i = 0; i <320000; i++);
	}
}







