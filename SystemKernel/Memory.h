/*
 * Memory.h
 *
 *  Created on: Nov 23, 2011
 *      Author: matthias
 */

#ifndef Memory_H_
#define Memory_H_

#include "SystemConfig.h"
#include "ApplicationConfig.h"



union CONFIG_FRAME_DB{

	struct{
		uint8_t packetIdentifier;
		uint8_t numberOfAddr;
		uint8_t general[148];
	}var;

	uint8_t frame[150];
};

union CONFIG_FRAME{

	struct{
		uint8_t numberOfLoadStates;
		uint8_t loadInterleave;
		uint8_t samplePeriods;
		uint8_t inputSolar;
		uint8_t harvestingConfig;
		uint8_t timerresolution;
		uint16_t storageSettings;
		uint16_t sleepCurrent;
		uint16_t loadCurrent[_maxNumberOfLoadStates];
		uint16_t timing[_maxNumberOfLoadStates];
		bool    firstRun;
	}var;

	uint8_t frame[28];
};


union DATA_FRAME{
	struct{
		uint8_t packetIdentifier;
		uint8_t packetNumber;
		uint8_t acknoledge;
		uint8_t free;
		float   currentSolar;
		float   voltageSolar;
		float   superCapA;
		float   superCapB;
		float   superCapC;
		float   voltageBattery;
		float   voltageSupply;
		float   voltageLoad;
		float   solarRadiation;
		float   humiditySHT;
		float   temperatureSHT;
	}var;

	uint8_t frame[49];
};


struct MEMORY_ELEMENT
{
	void   *element;
	uint8_t elementSize;
	uint8_t bufferLength;
	bool 	elementBlocked;
};

typedef enum{
	base,
	sleep,
	comPeriod,
	dataFrame,
	activeConfigFrame,
	dbACK,
	guard,
	nodeConfig
}QUEUE;

class Memory {
private:

	static void* test;
	static MEMORY_ELEMENT queue[_maxNumberOfElements];

public:
	Memory();
	~Memory();

	static void initializeElement( void*  element, uint8_t bufferLength, QUEUE varName );

	static bool getAccessToElement( void**  element, uint8_t *bufferLength, QUEUE varName );

};

#endif /* Memory_H_ */
