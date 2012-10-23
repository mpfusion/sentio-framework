/*
 * system.h
 *
 *  Created on: Feb 17, 2011
 *      Author: Matthias Krämer
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "SystemConfig.h"


typedef enum{
	buttonEventReset = 0x00,
	buttonEvent_1	 = 0x01,
	buttonEvent_2    = 0x02,
	buttonEvent_3    = 0x03,
	buttonEvent_4    = 0x04,
}BUTTON_EVENT;

typedef enum{
	useInitialState   = 0x00,
	useRecallState    = 0x01,
	selectInISR       = 0x02
}RECALL_OPTION;

struct STATUS_BLOCK{
	uint8_t 		numberOfStates;
	uint8_t 		numberOfISR;
	uint8_t 		initialState;
	uint8_t 		recallState;
	uint8_t 		nextState;
	uint8_t 	   	sleepMode;
	uint8_t 	   	applicationID;
	RECALL_OPTION  	recallOption;
	bool    		wantToSleep;
	bool   			restoreClockSetting;
};



/****************************************************************************************************************************************//**
 * @brief
 *  struct handles a pointer to functions. The function is the called from within the ISR-Handler. Application dependent can different
 *  ISR-Functions be specified and can be changed at RUNTIME!!!
 *
 *******************************************************************************************************************************************/
typedef void (*ptISR_Handler) (uint32_t);

struct ISR_ARRAY{
	ptISR_Handler function;
	uint8_t       interruptNumber;
	bool		  anchorISR;
};


/****************************************************************************************************************************************//**
 * @brief
 *  Declaration of the "system"-Class.
 *
 *******************************************************************************************************************************************/

class System
{
public:
	System(){}
	~System(){}

	void initializeSentioEM();

	void LED_SetGreen();
	void LED_SetOrange();
	void LED_SetRed();

	void LED_ClearGreen();
	void LED_ClearOrange();
	void LED_ClearRed();

	void LED_ToggleGreen();
	void LED_ToggleOrange();
	void LED_ToggleRed();

};


#endif /* SYSTEM_H_ */
