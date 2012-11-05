/*
 * DriverInterface.h
 *
 *  Created on: Nov 12, 2011
 *      Author: matthias
 */

#ifndef DRIVERINTERFACE_H_
#define DRIVERINTERFACE_H_


#include "DebugInterface.h"
#include "RTC_DS3234.h"
#include "AnalogInput.h"
#include "System.h"
#include "SensorExtensions/SHT1X_Sensirion.h"


class DriverInterface {
public:
	static DebugInterface  debug;
	static RTC_DS3234      timer;
	static System          sentio;
	static SHT1X_Sensirion humid;
	static AnalogInput     luminance;

	static void codeError();

	DriverInterface(){}
	~DriverInterface(){}
};

#endif /* DRIVERINTERFACE_H_ */
