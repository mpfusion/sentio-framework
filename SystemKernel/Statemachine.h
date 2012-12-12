/*
 * Statemachine.h
 *
 *  Created on: May 11, 2011
 *      Author: Matthias Krämer
 */

#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

#include "System.h"

/****************************************************************************************************************************************//**
 * @brief
 *  Definition of maximal-values, which determine size of system arrays in the FSM-handling structures
 *
 *******************************************************************************************************************************************/

#ifndef maxNumberOfStates
#define maxNumberOfStates 10
#endif

#ifndef maxNumberOfISR
#define maxNumberOfISR 5
#endif

/****************************************************************************************************************************************//**
 * @brief
 *  Type-Definition: used to define the State-Functions, within the Array that handles the FSM
 *
 *******************************************************************************************************************************************/

typedef bool ( *ptStateFunction )();


/****************************************************************************************************************************************//**
 * @brief
 * The Application Control Block combines a set of parameters and variables that is used to configure the FSM-handling structures
 *
 *******************************************************************************************************************************************/

struct APPLICATION_BLOCK
{
	BUTTON_EVENT   buttonEvent;
};

typedef enum
{
	FSM_finalized      = 0x00,
	FSM_NotInitialized = 0xFC,
	ISR_Invalid        = 0xFD,
	EM_invalid         = 0xFE,
	StateID_Invalid    = 0xFF
} ERROR_CODE;

extern "C" {
	void DMA_IRQHandler()       __attribute__( ( used, externally_visible ) );
	void GPIO_EVEN_IRQHandler() __attribute__( ( used, externally_visible ) );
	void TIMER0_IRQHandler()    __attribute__( ( used, externally_visible ) );
	void USART0_RX_IRQHandler() __attribute__( ( used, externally_visible ) );
	void USART0_TX_IRQHandler() __attribute__( ( used, externally_visible ) );
	void ACMP0_IRQHandler()     __attribute__( ( used, externally_visible ) );
	void ADC0_IRQHandler()      __attribute__( ( used, externally_visible ) );
	void DAC0_IRQHandler()      __attribute__( ( used, externally_visible ) );
	void I2C0_IRQHandler()      __attribute__( ( used, externally_visible ) );
	void GPIO_ODD_IRQHandler()  __attribute__( ( used, externally_visible ) );
	void TIMER1_IRQHandler()    __attribute__( ( used, externally_visible ) );
	void TIMER2_IRQHandler()    __attribute__( ( used, externally_visible ) );
	void USART1_RX_IRQHandler() __attribute__( ( used, externally_visible ) );
	void USART1_TX_IRQHandler() __attribute__( ( used, externally_visible ) );
	void USART2_RX_IRQHandler() __attribute__( ( used, externally_visible ) );
	void USART2_TX_IRQHandler() __attribute__( ( used, externally_visible ) );
	void UART0_RX_IRQHandler()  __attribute__( ( used, externally_visible ) );
	void UART0_TX_IRQHandler()  __attribute__( ( used, externally_visible ) );
	void LEUART0_IRQHandler()   __attribute__( ( used, externally_visible ) );
	void LEUART1_IRQHandler()   __attribute__( ( used, externally_visible ) );
	void LETIMER0_IRQHandler()  __attribute__( ( used, externally_visible ) );
	void PCNT0_IRQHandler()     __attribute__( ( used, externally_visible ) );
	void PCNT1_IRQHandler()     __attribute__( ( used, externally_visible ) );
	void PCNT2_IRQHandler()     __attribute__( ( used, externally_visible ) );
	void RTC_IRQHandler()       __attribute__( ( used, externally_visible ) );
	void CMU_IRQHandler()       __attribute__( ( used, externally_visible ) );
	void VCMP_IRQHandler()      __attribute__( ( used, externally_visible ) );
	void LCD_IRQHandler()       __attribute__( ( used, externally_visible ) );
	void MSC_IRQHandler()       __attribute__( ( used, externally_visible ) );
	void AES_IRQHandler()       __attribute__( ( used, externally_visible ) );
}


/****************************************************************************************************************************************//**
 * @brief
 *  Declaration of the Statemachine-Class.
 *
 *******************************************************************************************************************************************/

class Statemachine
{
private:
	static void _wrapperIRQ_default( uint32_t temp );
	static void _wrapperIRQ_EVEN_default( uint32_t temp );
	static void _wrapperIRQ_ODD_default( uint32_t temp );
	static void _wrapperIRQ_RTC( uint32_t temp );

	friend void DMA_IRQHandler();
	friend void GPIO_EVEN_IRQHandler();
	friend void TIMER0_IRQHandler();
	friend void USART0_RX_IRQHandler();
	friend void USART0_TX_IRQHandler();
	friend void ACMP0_IRQHandler();
	friend void ADC0_IRQHandler();
	friend void DAC0_IRQHandler();
	friend void I2C0_IRQHandler();
	friend void GPIO_ODD_IRQHandler();
	friend void TIMER1_IRQHandler();
	friend void TIMER2_IRQHandler();
	friend void USART1_RX_IRQHandler();
	friend void USART1_TX_IRQHandler();
	friend void USART2_RX_IRQHandler();
	friend void USART2_TX_IRQHandler();
	friend void UART0_RX_IRQHandler();
	friend void UART0_TX_IRQHandler();
	friend void LEUART0_IRQHandler();
	friend void LEUART1_IRQHandler();
	friend void LETIMER0_IRQHandler();
	friend void PCNT0_IRQHandler();
	friend void PCNT1_IRQHandler();
	friend void PCNT2_IRQHandler();
	friend void RTC_IRQHandler();
	friend void CMU_IRQHandler();
	friend void VCMP_IRQHandler();
	friend void LCD_IRQHandler();
	friend void MSC_IRQHandler();
	friend void AES_IRQHandler();

	static ptISR_Handler wrapperDMA_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperGPIO_EVEN_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperTIMER0_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperUSART0_RX_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperUSART0_TX_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperACMP0_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperADC0_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperDAC0_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperI2C0_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperGPIO_ODD_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperTIMER1_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperTIMER2_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperUSART1_RX_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperUSART1_TX_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperUSART2_RX_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperUSART2_TX_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperUART0_RX_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperUART0_TX_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperLEUART0_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperLEUART1_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperLETIMER0_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperPCNT0_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperPCNT1_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperPCNT2_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperRTC_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperCMU_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperVCMP_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperLCD_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperMSC_IRQ[maxNumberApplications];
	static ptISR_Handler wrapperAES_IRQ[maxNumberApplications];

	STATUS_BLOCK            *callingStatusBlock;
	static STATUS_BLOCK     *currentStatusBlock;

	bool                    initialized;
	bool                    setupDone;

protected:
	ISR_ARRAY               ISR_Definition[maxNumberOfISR];
	ptStateFunction         stateDefinition[maxNumberOfStates];

	uint8_t initializeApplication( STATUS_BLOCK *statusBlock );
	ERROR_CODE startApplication( STATUS_BLOCK *statusBlock );

public:
	Statemachine();
	~Statemachine() {}
};

#endif /* STATEMACHINE_H_ */
