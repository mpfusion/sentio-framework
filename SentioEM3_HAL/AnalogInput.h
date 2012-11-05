/*
 * AnalogInput.h
 *
 *  Created on: Nov 5, 2011
 *      Author: matthias
 */

#ifndef ANALOGINPUT_H_
#define ANALOGINPUT_H_

#include "efm32_adc.h"
#include "efm32_gpio.h"

typedef enum{
	radiationSensor
}SENSORS;


class AnalogInput
{

private:
	ADC_Init_TypeDef 		init;
	ADC_InitScan_TypeDef 	scan;
	ADC_InitSingle_TypeDef  single;
	uint32_t				calirationValue;

public:
	AnalogInput(){}
	~AnalogInput(){}

	void initializeInterface();

	void readMultipleChannels( uint16_t *data );
	void readChannel( ADC_SingleInput_TypeDef channel, uint16_t &data );

	void readMultipleChannels( float *data );
	void readChannel( ADC_SingleInput_TypeDef channel, float &data );

	void enableSensor( SENSORS input );
	void disableSensor( SENSORS  input);

	void initializeSensorEN( SENSORS input );

	void calibrateReference();

};

#endif /* ANALOGINPUT_H_ */
