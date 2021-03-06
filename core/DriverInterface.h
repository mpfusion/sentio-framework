/*
 * DriverInterface.h
 *
 *  Created on: Nov 12, 2011
 *      Author: matthias
 */

#ifndef DRIVERINTERFACE_H_
#define DRIVERINTERFACE_H_


#include "sentio_em.h"

#include "RTC_DS3234.h"
#include "DebugInterface.h"
/*
#include "AnalogInput.h"
#include "SHT1X_Sensirion.h"
#include "ConfEH.h"
#include "XBEE_Radio.h"
#include "CC1101_Radio.h"*/


class DriverInterface {
public:
	static SENTIO_EM        sentio;
	static DebugInterface  debug;
	static RTC_DS3234      timer;
// 	static SHT1X_Sensirion humid;
// 	static AnalogInput     luminance;
// 	static LTC2990         ltc2990;
// 	static CONFEH          confeh;
// 	static XBEE_Radio      xbee;
// 	static CC1101_Radio    cc1101;

	//static void codeError();

	DriverInterface(){}
	~DriverInterface(){}
};

#endif /* DRIVERINTERFACE_H_ */
