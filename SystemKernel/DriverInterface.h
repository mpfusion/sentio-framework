/*
 * DriverInterface.h
 *
 *  Created on: Nov 12, 2011
 *      Author: matthias
 */

#ifndef DRIVERINTERFACE_H_
#define DRIVERINTERFACE_H_


#include "DebugInterface.h"
#include "AnalogInput.h"
#include "System.h"
#include "RTC_DS3234.h"
#include "SHT1X_Sensirion.h"
#include "LTC2990.h"


class DriverInterface {
public:
	static DebugInterface  debug;
	static RTC_DS3234      timer;
	static System          sentio;
	static SHT1X_Sensirion humid;
	static AnalogInput     luminance;
	static LTC2990         ltc2990;

	static void codeError();

	DriverInterface(){}
	~DriverInterface(){}
};

#endif /* DRIVERINTERFACE_H_ */
