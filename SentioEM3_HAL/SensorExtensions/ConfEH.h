/*
 * ConfEH.h
 *
 *  Created on: 16 nov 2012
 *      Author: sebastian
 */

#ifndef CONFEH_H_
#define CONFEH_H_

#include "efm32.h"
#include "LTC2990.h"

// Storage Types
#define DLC         0x01
#define BAT         0x02

// Harvesting Modes
#define DIRECT      0x01
#define LTC3105     0x02
#define MAX17710    0x04

class CONFEH : public LTC2990
{
private:
	bool pinTXS0102_EN;
	bool pinMAX17710_AE;
	bool pinMAX17710_LCE;
	bool pinLTC3105_EN;
	bool pinResistor_1;
	bool pinResistor_2;
	bool pinResistor_3;

	uint8_t storageType;
	uint8_t harvestingMode;

public:
	CONFEH() {}
	~CONFEH() {}

	void initializeInterface();
	void setMode( uint8_t storage, uint8_t harvester );
	void getMeasurements( float &storageVoltage, float &solarVoltage, float &solarCurrent );

	void setTXS0102_EN( bool state );
	void setMAX17710_AE( bool state );
	void setMAX17710_LCE( bool state );
	void setLTC3105_EN( bool state );
	void setResistor_1( bool state );
	void setResistor_2( bool state );
	void setResistor_3( bool state );
};


#endif /* CONFEH_H_ */
