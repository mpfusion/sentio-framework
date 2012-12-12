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

ptISR_Handler Statemachine::wrapperDMA_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperGPIO_EVEN_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperTIMER0_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperUSART0_RX_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperUSART0_TX_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperACMP0_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperADC0_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperDAC0_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperI2C0_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperGPIO_ODD_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperTIMER1_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperTIMER2_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperUSART1_RX_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperUSART1_TX_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperUSART2_RX_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperUSART2_TX_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperUART0_RX_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperUART0_TX_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperLEUART0_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperLEUART1_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperLETIMER0_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperPCNT0_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperPCNT1_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperPCNT2_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperRTC_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperCMU_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperVCMP_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperLCD_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperMSC_IRQ[maxNumberApplications];
ptISR_Handler Statemachine::wrapperAES_IRQ[maxNumberApplications];


void DMA_IRQHandler()
{
	( *Statemachine::wrapperDMA_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void GPIO_EVEN_IRQHandler()
{
	( *Statemachine::wrapperGPIO_EVEN_IRQ[Statemachine::currentStatusBlock->applicationID] )( GPIO->IF );
	Statemachine::_wrapperIRQ_EVEN_default( GPIO->IF );
}


void TIMER0_IRQHandler()
{

	( *Statemachine::wrapperTIMER0_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void USART0_RX_IRQHandler()
{


	( *Statemachine::wrapperUSART0_RX_IRQ[Statemachine::currentStatusBlock->applicationID] )( ( uint32_t ) USART0->RXDATA );
}


void USART0_TX_IRQHandler()
{
	( *Statemachine::wrapperUSART0_TX_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void ACMP0_IRQHandler()
{
	( *Statemachine::wrapperACMP0_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void ADC0_IRQHandler()
{
	( *Statemachine::wrapperADC0_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void DAC0_IRQHandler()
{
	( *Statemachine::wrapperDAC0_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void I2C0_IRQHandler()
{
	( *Statemachine::wrapperI2C0_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void GPIO_ODD_IRQHandler()
{
	( *Statemachine::wrapperGPIO_ODD_IRQ[Statemachine::currentStatusBlock->applicationID] )( GPIO->IF );
}


void TIMER1_IRQHandler()
{
	( *Statemachine::wrapperTIMER1_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void TIMER2_IRQHandler()
{
	( *Statemachine::wrapperTIMER2_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void USART1_RX_IRQHandler()
{
	( *Statemachine::wrapperUSART1_RX_IRQ[Statemachine::currentStatusBlock->applicationID] )( USART1->RXDATA );
}


void USART1_TX_IRQHandler()
{
	( *Statemachine::wrapperUSART1_TX_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void USART2_RX_IRQHandler()
{
	( *Statemachine::wrapperUSART2_RX_IRQ[Statemachine::currentStatusBlock->applicationID] )( USART2->RXDATA );
}


void USART2_TX_IRQHandler()
{
	( *Statemachine::wrapperUSART2_TX_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void UART0_RX_IRQHandler()
{
	( *Statemachine::wrapperUART0_RX_IRQ[Statemachine::currentStatusBlock->applicationID] )( UART0->RXDATA );
}


void UART0_TX_IRQHandler()
{
	( *Statemachine::wrapperUART0_TX_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void LEUART0_IRQHandler()
{
	( *Statemachine::wrapperLEUART0_IRQ[Statemachine::currentStatusBlock->applicationID] )( LEUART0->RXDATA );
}


void LEUART1_IRQHandler()
{
	( *Statemachine::wrapperLEUART1_IRQ[Statemachine::currentStatusBlock->applicationID] )( LEUART1->RXDATA );
}


void LETIMER0_IRQHandler()
{
	( *Statemachine::wrapperLETIMER0_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void PCNT0_IRQHandler()
{
	( *Statemachine::wrapperPCNT0_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void PCNT1_IRQHandler()
{
	( *Statemachine::wrapperPCNT1_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void PCNT2_IRQHandler()
{
	( *Statemachine::wrapperPCNT2_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void RTC_IRQHandler()
{
	( *Statemachine::wrapperRTC_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void CMU_IRQHandler()
{
	( *Statemachine::wrapperCMU_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void VCMP_IRQHandler()
{
	( *Statemachine::wrapperVCMP_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void LCD_IRQHandler()
{
	( *Statemachine::wrapperLCD_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void MSC_IRQHandler()
{
	( *Statemachine::wrapperMSC_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


void AES_IRQHandler()
{
	( *Statemachine::wrapperAES_IRQ[Statemachine::currentStatusBlock->applicationID] )( 0x00 );
}


Statemachine::Statemachine()
{
	initialized = false;
	setupDone   = false;

	for ( uint32_t i = 0; i < maxNumberApplications; i++ )
	{
		Statemachine::wrapperDMA_IRQ[i]       = _wrapperIRQ_default;
		Statemachine::wrapperGPIO_EVEN_IRQ[i] = _wrapperIRQ_default;
		Statemachine::wrapperTIMER0_IRQ[i]    = _wrapperIRQ_default;
		Statemachine::wrapperUSART0_RX_IRQ[i] = _wrapperIRQ_default;
		Statemachine::wrapperUSART0_TX_IRQ[i] = _wrapperIRQ_default;
		Statemachine::wrapperACMP0_IRQ[i]     = _wrapperIRQ_default;
		Statemachine::wrapperADC0_IRQ[i]      = _wrapperIRQ_default;
		Statemachine::wrapperDAC0_IRQ[i]      = _wrapperIRQ_default;
		Statemachine::wrapperI2C0_IRQ[i]      = _wrapperIRQ_default;
		Statemachine::wrapperGPIO_ODD_IRQ[i]  = _wrapperIRQ_default;
		Statemachine::wrapperTIMER1_IRQ[i]    = _wrapperIRQ_default;
		Statemachine::wrapperTIMER2_IRQ[i]    = _wrapperIRQ_default;
		Statemachine::wrapperUSART1_RX_IRQ[i] = _wrapperIRQ_default;
		Statemachine::wrapperUSART1_TX_IRQ[i] = _wrapperIRQ_default;
		Statemachine::wrapperUSART2_RX_IRQ[i] = _wrapperIRQ_default;
		Statemachine::wrapperUSART2_TX_IRQ[i] = _wrapperIRQ_default;
		Statemachine::wrapperUART0_RX_IRQ[i]  = _wrapperIRQ_default;
		Statemachine::wrapperUART0_TX_IRQ[i]  = _wrapperIRQ_default;
		Statemachine::wrapperLEUART0_IRQ[i]   = _wrapperIRQ_default;
		Statemachine::wrapperLEUART1_IRQ[i]   = _wrapperIRQ_default;
		Statemachine::wrapperLETIMER0_IRQ[i]  = _wrapperIRQ_default;
		Statemachine::wrapperPCNT0_IRQ[i]     = _wrapperIRQ_default;
		Statemachine::wrapperPCNT1_IRQ[i]     = _wrapperIRQ_default;
		Statemachine::wrapperPCNT2_IRQ[i]     = _wrapperIRQ_default;
		Statemachine::wrapperRTC_IRQ[i]       = _wrapperIRQ_default;
		Statemachine::wrapperCMU_IRQ[i]       = _wrapperIRQ_default;
		Statemachine::wrapperVCMP_IRQ[i]      = _wrapperIRQ_default;
		Statemachine::wrapperLCD_IRQ[i]       = _wrapperIRQ_default;
		Statemachine::wrapperMSC_IRQ[i]       = _wrapperIRQ_default;
		Statemachine::wrapperAES_IRQ[i]       = _wrapperIRQ_default;
	}

	Statemachine::wrapperGPIO_EVEN_IRQ[0] = Statemachine::_wrapperIRQ_EVEN_default;
	Statemachine::wrapperGPIO_ODD_IRQ[0]  = Statemachine::_wrapperIRQ_ODD_default;
	//Statemachine::wrapperRTC_IRQ[0]     = Statemachine::_wrapperIRQ_RTC;
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
	for ( uint32_t count = 0; count < statusBlock->numberOfISR; count++ )
	{
		switch ( ISR_Definition[count].interruptNumber )
		{
		case DMA_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperDMA_IRQ[i] = ISR_Definition[count].function;
			else wrapperDMA_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case GPIO_EVEN_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperGPIO_EVEN_IRQ[i] = ISR_Definition[count].function;
			else wrapperGPIO_EVEN_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case TIMER0_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperTIMER0_IRQ[i] = ISR_Definition[count].function;
			else wrapperTIMER0_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case USART0_RX_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperUSART0_RX_IRQ[i] = ISR_Definition[count].function;
			else wrapperUSART0_RX_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case USART0_TX_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperUSART0_TX_IRQ[i] = ISR_Definition[count].function;
			else wrapperUSART0_TX_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case ACMP0_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperACMP0_IRQ[i] = ISR_Definition[count].function;
			else wrapperACMP0_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case ADC0_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperADC0_IRQ[i] = ISR_Definition[count].function;
			else wrapperADC0_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case DAC0_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperDAC0_IRQ[i] = ISR_Definition[count].function;
			else wrapperDAC0_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case I2C0_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperI2C0_IRQ[i] = ISR_Definition[count].function;
			else wrapperI2C0_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case GPIO_ODD_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperGPIO_ODD_IRQ[i] = ISR_Definition[count].function;
			else wrapperGPIO_ODD_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case TIMER1_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperTIMER1_IRQ[i] = ISR_Definition[count].function;
			else wrapperTIMER1_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case TIMER2_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperTIMER2_IRQ[i] = ISR_Definition[count].function;
			else wrapperTIMER2_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case USART1_RX_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperUSART1_RX_IRQ[i] = ISR_Definition[count].function;
			else wrapperUSART1_RX_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case USART1_TX_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperUSART1_TX_IRQ[i] = ISR_Definition[count].function;
			else wrapperUSART1_TX_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case USART2_RX_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperUSART2_RX_IRQ[i] = ISR_Definition[count].function;
			else wrapperUSART2_RX_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case USART2_TX_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperUSART2_TX_IRQ[i] = ISR_Definition[count].function;
			else wrapperUSART2_TX_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case UART0_RX_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperUART0_RX_IRQ[i] = ISR_Definition[count].function;
			else wrapperUART0_RX_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case UART0_TX_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperUART0_TX_IRQ[i] = ISR_Definition[count].function;
			else wrapperUART0_TX_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case LEUART0_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperLEUART0_IRQ[i] = ISR_Definition[count].function;
			else wrapperLEUART0_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case LEUART1_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperLEUART1_IRQ[i] = ISR_Definition[count].function;
			else wrapperLEUART1_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case LETIMER0_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperLETIMER0_IRQ[i] = ISR_Definition[count].function;
			else wrapperLETIMER0_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case PCNT0_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperPCNT0_IRQ[i] = ISR_Definition[count].function;
			else wrapperPCNT0_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case PCNT1_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperPCNT1_IRQ[i] = ISR_Definition[count].function;
			else wrapperPCNT1_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case PCNT2_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperPCNT2_IRQ[i] = ISR_Definition[count].function;
			else wrapperPCNT2_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case RTC_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperRTC_IRQ[i] = ISR_Definition[count].function;
			else wrapperRTC_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case CMU_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperCMU_IRQ[i] = ISR_Definition[count].function;
			else wrapperCMU_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case VCMP_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperVCMP_IRQ[i] = ISR_Definition[count].function;
			else wrapperVCMP_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case MSC_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperMSC_IRQ[i] = ISR_Definition[count].function;
			else wrapperMSC_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		case AES_IRQn:
			if ( ISR_Definition[count].anchorISR )
				for ( uint8_t i = 0; i < maxNumberApplications; i++ ) wrapperAES_IRQ[i] = ISR_Definition[count].function;
			else wrapperAES_IRQ[statusBlock->applicationID] = ISR_Definition[count].function;
			break;
		default:
			return 0xFF - ISR_Definition[count].interruptNumber;
		}
	}

	return 0x00;
}


ERROR_CODE Statemachine::startApplication( STATUS_BLOCK *statusBlock )
{
	callingStatusBlock = currentStatusBlock;
	currentStatusBlock = statusBlock;

	if ( !setupDone )
		return FSM_NotInitialized;

	if ( initialized )
		switch ( statusBlock->recallOption )
		{
		case useInitialState:
			currentStatusBlock->nextState = currentStatusBlock->initialState;
			break;

		case useRecallState:
			currentStatusBlock->nextState = currentStatusBlock->recallState;
			break;

		case selectInISR:
		default:
			;
		}
	else
		initialized = true;

	// Run the FSM until the execution is stopped by the Application-Code.
	// The State-Function is called within the WHILE-STATEMENT !!!!
	while ( ( *( stateDefinition )[statusBlock->nextState] )() )
	{
		// Is the State-Function, which is selected for execution in the next step a valid one?
		if ( currentStatusBlock->nextState < maxNumberOfStates )
		{
			// Does the MCU have to enter a sleep-mode?
			if ( currentStatusBlock->wantToSleep )
			{
				// Which sleep-mode has to be entered?
				switch ( currentStatusBlock->sleepMode )
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
	while ( 1 );
}


void Statemachine::_wrapperIRQ_EVEN_default( uint32_t temp )
{

}


void Statemachine::_wrapperIRQ_ODD_default( uint32_t temp )
{

}
