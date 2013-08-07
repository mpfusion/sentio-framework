/*
 * Statemachine.h
 *
 *  Created on: May 11, 2011
 *      Author: Matthias Krämer
 */

#ifndef FSM_H_
#define FSM_H_

#include <stdint.h>
#include "application_config.h"


#ifndef MAX_NUM_STATES
	#define MAX_NUM_STATES 10
#endif

#ifndef MAX_NUM_ISR
	#define MAX_NUM_ISR 5
#endif

#ifndef MAX_NUM_APPS
	#define MAX_NUM_APPS 1
#endif

typedef bool (*STATE_FUNCTION) ();
typedef void (*ISR_HANDLER) (uint32_t);

struct ISR_ARRAY{
	ISR_HANDLER function;
	uint8_t     interruptNumber;
	bool        anchorISR;
};

enum SLEEP_MODE{
	on      = 0,
	sleep   = 1,
	stop    = 2,
	deepstop= 3,
	off     = 4
};


struct STATUS_BLOCK{
	SLEEP_MODE sleepMode;
	uint32_t   applicationID;
	uint32_t   nextState;
	uint32_t   initialState;
	uint32_t   numOfISR;
};


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

class FSM
{
private:
	static void _wrapperIRQ_default( uint32_t temp );

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

	static ISR_HANDLER wrapperDMA_IRQ       [MAX_NUM_APPS];
	static ISR_HANDLER wrapperGPIO_EVEN_IRQ [MAX_NUM_APPS];
	static ISR_HANDLER wrapperTIMER0_IRQ    [MAX_NUM_APPS];
	static ISR_HANDLER wrapperUSART0_RX_IRQ [MAX_NUM_APPS];
	static ISR_HANDLER wrapperUSART0_TX_IRQ [MAX_NUM_APPS];
	static ISR_HANDLER wrapperACMP0_IRQ     [MAX_NUM_APPS];
	static ISR_HANDLER wrapperADC0_IRQ      [MAX_NUM_APPS];
	static ISR_HANDLER wrapperDAC0_IRQ      [MAX_NUM_APPS];
	static ISR_HANDLER wrapperI2C0_IRQ      [MAX_NUM_APPS];
	static ISR_HANDLER wrapperGPIO_ODD_IRQ  [MAX_NUM_APPS];
	static ISR_HANDLER wrapperTIMER1_IRQ    [MAX_NUM_APPS];
	static ISR_HANDLER wrapperTIMER2_IRQ    [MAX_NUM_APPS];
	static ISR_HANDLER wrapperUSART1_RX_IRQ [MAX_NUM_APPS];
	static ISR_HANDLER wrapperUSART1_TX_IRQ [MAX_NUM_APPS];
	static ISR_HANDLER wrapperUSART2_RX_IRQ [MAX_NUM_APPS];
	static ISR_HANDLER wrapperUSART2_TX_IRQ [MAX_NUM_APPS];
	static ISR_HANDLER wrapperUART0_RX_IRQ  [MAX_NUM_APPS];
	static ISR_HANDLER wrapperUART0_TX_IRQ  [MAX_NUM_APPS];
	static ISR_HANDLER wrapperLEUART0_IRQ   [MAX_NUM_APPS];
	static ISR_HANDLER wrapperLEUART1_IRQ   [MAX_NUM_APPS];
	static ISR_HANDLER wrapperLETIMER0_IRQ  [MAX_NUM_APPS];
	static ISR_HANDLER wrapperPCNT0_IRQ     [MAX_NUM_APPS];
	static ISR_HANDLER wrapperPCNT1_IRQ     [MAX_NUM_APPS];
	static ISR_HANDLER wrapperPCNT2_IRQ     [MAX_NUM_APPS];
	static ISR_HANDLER wrapperRTC_IRQ       [MAX_NUM_APPS];
	static ISR_HANDLER wrapperCMU_IRQ       [MAX_NUM_APPS];
	static ISR_HANDLER wrapperVCMP_IRQ      [MAX_NUM_APPS];
	static ISR_HANDLER wrapperLCD_IRQ       [MAX_NUM_APPS];
	static ISR_HANDLER wrapperMSC_IRQ       [MAX_NUM_APPS];
	static ISR_HANDLER wrapperAES_IRQ       [MAX_NUM_APPS];

protected:
	uint8_t setup( STATUS_BLOCK *statusBlock );
	void execute( STATUS_BLOCK *statusBlock );

	static STATUS_BLOCK *currentStatusBlock;

	ISR_ARRAY isrDefinition        [MAX_NUM_ISR];
	STATE_FUNCTION stateDefinition [MAX_NUM_STATES];
public:
	FSM();
	~FSM(){}
};

#endif /* FSM_H_ */
