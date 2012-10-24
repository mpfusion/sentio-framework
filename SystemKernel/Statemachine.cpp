/*
 * statemachine.cpp
 *
 *  Created on: May 13, 2011
 *      Author: Matthias Krämer
 */

#include "Statemachine.h"
#include "efm32_emu.h"
#include "efm32_gpio.h"
STATUS_BLOCK *Statemachine::currentStatusBlock;


ptISR_Handler Statemachine::wrapperGPIO_ODD_IRQ[maxNumberApplications];


void GPIO_ODD_IRQHandler()
{
	GPIO->P[gpioPortA].DOUTSET = 1 << 0;
	(*Statemachine::wrapperGPIO_ODD_IRQ[Statemachine::currentStatusBlock->applicationID])(GPIO->IF);
}



Statemachine::Statemachine()
{
	initialized = false;
	setupDone   = false;

	for(uint32_t i = 0; i < maxNumberApplications; i++)
	{
		Statemachine::wrapperGPIO_ODD_IRQ[i]	= _wrapperIRQ_default;

	}
}

/****************************************************************************************************************************************//**
 * @brief
 *  The method handles the State-Machine based program execution after the setup has been finalized.
 *
 * @details
 *  The State-Machine is running until on of the State-Functions, defined in the User-Code section returns false. Then the FSM-execution,
 *  stops and runApplication() returns true and therefore setupApplication() returns true to the main()-function. When the State-Function
 *  returns true instead, the EFM32-MCU enters the selected sleep-mode if the parameter wantToSleep == true. Otherwise the next State-Function
 *  is executed directly, without any waiting or delay-period in between.
 *
 * @return
 *
 *******************************************************************************************************************************************/
uint8_t Statemachine::initializeApplication( STATUS_BLOCK *statusBlock )
{
	setupDone = true;

	// Copy the Pointer to the Interrupt Handler
	for(uint32_t count = 0; count < statusBlock->numberOfISR; count++)
	{
		switch(ISR_Definition[count].interruptNumber)
		{

		case GPIO_ODD_IRQn:
			if(ISR_Definition[count].anchorISR)
				for(uint8_t i = 0; i < maxNumberApplications; i++) wrapperGPIO_ODD_IRQ[i] = ISR_Definition[count].function;
			else wrapperGPIO_ODD_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;

		default:
			return 0xFF-ISR_Definition[count].interruptNumber;
		}
	}

	return 0x00;
}


ERROR_CODE Statemachine::startApplication( STATUS_BLOCK *statusBlock )
{
	callingStatusBlock = currentStatusBlock;
	currentStatusBlock = statusBlock;

	if(!setupDone)
		return FSM_NotInitialized;


	// Run the FSM until the execution is stopped by the Application-Code.
	// The State-Function is called within the WHILE-STATEMENT !!!!
	while( ( *(stateDefinition)[statusBlock->nextState] )() )
	{
		// Is the State-Function, which is selected for execution in the next step a valid one?
		if( currentStatusBlock->nextState < maxNumberOfStates )
		{
			// Does the MCU have to enter a sleep-mode?
			if( currentStatusBlock->wantToSleep )
			{
				// Which sleep-mode has to be entered?
				switch( currentStatusBlock->sleepMode )
				{
				case 0:
					break;
				case 1:
					EMU_EnterEM1();
					break;
				case 2:
					EMU_EnterEM2( currentStatusBlock->restoreClockSetting );
					break;
				case 3:
					EMU_EnterEM3( currentStatusBlock->restoreClockSetting );
					break;
				case 4:
					EMU_EnterEM4();
				default:
					return EM_invalid; // Invalid sleep-mode!!!
				}
			}
		}

		else
			return StateID_Invalid; // Invalid State-Function number!!!
	}

	currentStatusBlock = callingStatusBlock;

	return FSM_finalized; // FSM-execution successfully finalized!!!
}


void Statemachine::_wrapperIRQ_default( uint32_t test )
{
	while(1);
}


