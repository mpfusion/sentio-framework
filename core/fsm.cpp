/*
 * statemachine.cpp
 *
 *  Created on: May 13, 2011
 *      Author: Matthias Krämer
 */

#include "application_config.h"

#include "fsm.h"
#include "em_emu.h"

STATUS_BLOCK *FSM::currentStatusBlock;

#if DMA_IRQ_ENABLE
ISR_HANDLER FSM::wrapperDMA_IRQ[MAX_NUM_APPS];
void DMA_IRQHandler()
{
	(*FSM::wrapperDMA_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if GPIO_EVEN_IRQ_ENABLE
ISR_HANDLER FSM::wrapperGPIO_EVEN_IRQ[MAX_NUM_APPS];
void GPIO_EVEN_IRQHandler()
{
	(*FSM::wrapperGPIO_EVEN_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
	GPIO->IFC = 0xFFFF;
}
#endif
#if TIMER0_IRQ_ENABLE
ISR_HANDLER FSM::wrapperTIMER0_IRQ[MAX_NUM_APPS];
void TIMER0_IRQHandler()
{
	(*FSM::wrapperTIMER0_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if USART0_RX_IRQ_ENABLE
ISR_HANDLER FSM::wrapperUSART0_RX_IRQ[MAX_NUM_APPS];
void USART0_RX_IRQHandler()
{
	(*FSM::wrapperUSART0_RX_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if USART0_TX_IRQ_ENABLE
ISR_HANDLER FSM::wrapperUSART0_TX_IRQ[MAX_NUM_APPS];
void USART0_TX_IRQHandler()
{
	(*FSM::wrapperUSART0_TX_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if ACMP0_IRQ_ENABLE
ISR_HANDLER FSM::wrapperACMP0_IRQ[MAX_NUM_APPS];
void ACMP0_IRQHandler()
{
	(*FSM::wrapperACMP0_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if ADC0_IRQ_ENABLE
ISR_HANDLER FSM::wrapperADC0_IRQ[MAX_NUM_APPS];
void ADC0_IRQHandler()
{
	(*FSM::wrapperADC0_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if DAC0_IRQ_ENABLE
ISR_HANDLER FSM::wrapperDAC0_IRQ[MAX_NUM_APPS];
void DAC0_IRQHandler()
{
	(*FSM::wrapperDAC0_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if I2C0_IRQ_ENABLE
ISR_HANDLER FSM::wrapperI2C0_IRQ[MAX_NUM_APPS];
void I2C0_IRQHandler()
{
	(*FSM::wrapperI2C0_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if GPIO_ODD_IRQ_ENABLE
ISR_HANDLER FSM::wrapperGPIO_ODD_IRQ[MAX_NUM_APPS];
void GPIO_ODD_IRQHandler()
{
	(*FSM::wrapperGPIO_ODD_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
	GPIO->IFC = 0xFFFF;
}
#endif
#if TIMER1_IRQ_ENABLE
ISR_HANDLER FSM::wrapperTIMER1_IRQ[MAX_NUM_APPS];
void TIMER1_IRQHandler()
{
	(*FSM::wrapperTIMER1_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if TIMER2_IRQ_ENABLE
ISR_HANDLER FSM::wrapperTIMER2_IRQ[MAX_NUM_APPS];
void TIMER2_IRQHandler()
{
	(*FSM::wrapperTIMER2_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if USART1_RX_IRQ_ENABLE
ISR_HANDLER FSM::wrapperUSART1_RX_IRQ[MAX_NUM_APPS];
void USART1_RX_IRQHandler()
{
	(*FSM::wrapperUSART1_RX_IRQ[FSM::currentStatusBlock->applicationID])(USART1->RXDATA);
}
#endif
#if USART1_TX_IRQ_ENABLE
ISR_HANDLER FSM::wrapperUSART1_TX_IRQ[MAX_NUM_APPS];
void USART1_TX_IRQHandler()
{
	(*FSM::wrapperUSART1_TX_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if USART2_RX_IRQ_ENABLE
ISR_HANDLER FSM::wrapperUSART2_RX_IRQ[MAX_NUM_APPS];
void USART2_RX_IRQHandler()
{
	(*FSM::wrapperUSART2_RX_IRQ[FSM::currentStatusBlock->applicationID])(USART2->RXDATA);
}
#endif
#if USART2_TX_IRQ_ENABLE
ISR_HANDLER FSM::wrapperUSART2_TX_IRQ[MAX_NUM_APPS];
void USART2_TX_IRQHandler()
{
	(*FSM::wrapperUSART2_TX_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if UART0_RX_IRQ_ENABLE
ISR_HANDLER FSM::wrapperUART0_RX_IRQ[MAX_NUM_APPS];
void UART0_RX_IRQHandler()
{
	(*FSM::wrapperUART0_RX_IRQ[FSM::currentStatusBlock->applicationID])(UART0->RXDATA);
}
#endif
#if UART0_TX_IRQ_ENABLE
ISR_HANDLER FSM::wrapperUART0_TX_IRQ[MAX_NUM_APPS];
void UART0_TX_IRQHandler()
{
	(*FSM::wrapperUART0_TX_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if LEUART0_IRQ_ENABLE
ISR_HANDLER FSM::wrapperLEUART0_IRQ[MAX_NUM_APPS];
void LEUART0_IRQHandler()
{
	(*FSM::wrapperLEUART0_IRQ[FSM::currentStatusBlock->applicationID])(LEUART0->RXDATA);
}
#endif
#if LEUART1_IRQ_ENABLE
ISR_HANDLER FSM::wrapperLEUART1_IRQ[MAX_NUM_APPS];
void LEUART1_IRQHandler()
{
	(*FSM::wrapperLEUART1_IRQ[FSM::currentStatusBlock->applicationID])(LEUART1->RXDATA);
}
#endif
#if LETIMER0_IRQ_ENABLE
ISR_HANDLER FSM::wrapperLETIMER0_IRQ[MAX_NUM_APPS];
void LETIMER0_IRQHandler()
{
	(*FSM::wrapperLETIMER0_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if PCNT0_IRQ_ENABLE
ISR_HANDLER FSM::wrapperPCNT0_IRQ[MAX_NUM_APPS];
void PCNT0_IRQHandler()
{
	(*FSM::wrapperPCNT0_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if PCNT1_IRQ_ENABLE
ISR_HANDLER FSM::wrapperPCNT1_IRQ[MAX_NUM_APPS];
void PCNT1_IRQHandler()
{
	(*FSM::wrapperPCNT1_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if PCNT2_IRQ_ENABLE
ISR_HANDLER FSM::wrapperPCNT2_IRQ[MAX_NUM_APPS];
void PCNT2_IRQHandler()
{
	(*FSM::wrapperPCNT2_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if RTC_IRQ_ENABLE
ISR_HANDLER FSM::wrapperRTC_IRQ[MAX_NUM_APPS];
void RTC_IRQHandler()
{
	(*FSM::wrapperRTC_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if CMU_IRQ_ENABLE
ISR_HANDLER FSM::wrapperCMU_IRQ[MAX_NUM_APPS];
void CMU_IRQHandler()
{
	(*FSM::wrapperCMU_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if VCMP_IRQ_ENABLE
ISR_HANDLER FSM::wrapperVCMP_IRQ[MAX_NUM_APPS];
void VCMP_IRQHandler()
{
	(*FSM::wrapperVCMP_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if LCD_IRQ_ENABLE
ISR_HANDLER FSM::wrapperLCD_IRQ[MAX_NUM_APPS];
void LCD_IRQHandler()
{
	(*FSM::wrapperLCD_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if MSC_IRQ_ENABLE
ISR_HANDLER FSM::wrapperMSC_IRQ[MAX_NUM_APPS];
void MSC_IRQHandler()
{
	(*FSM::wrapperMSC_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif
#if AES_IRQ_ENABLE
ISR_HANDLER FSM::wrapperAES_IRQ[MAX_NUM_APPS];

void AES_IRQHandler()
{
	(*FSM::wrapperAES_IRQ[FSM::currentStatusBlock->applicationID])(GPIO->IF);
}
#endif

FSM::FSM()
{

	for(uint32_t i = 0; i < MAX_NUM_APPS; i++)
	{
#if DMA_IRQ_ENABLE
	FSM::wrapperDMA_IRQ[i]			= _wrapperIRQ_default;
#endif
#if GPIO_EVEN_IRQ_ENABLE
	FSM::wrapperGPIO_EVEN_IRQ[i]	= _wrapperIRQ_default;
#endif
#if TIMER0_IRQ_ENABLE
	FSM::wrapperTIMER0_IRQ[i]		= _wrapperIRQ_default;
#endif
#if USART0_RX_IRQ_ENABLE
	FSM::wrapperUSART0_RX_IRQ[i]	= _wrapperIRQ_default;
#endif
#if USART0_TX_IRQ_ENABLE
	FSM::wrapperUSART0_TX_IRQ[i]	= _wrapperIRQ_default;
#endif
#if ACMP0_IRQ_ENABLE
	FSM::wrapperACMP0_IRQ[i]		= _wrapperIRQ_default;
#endif
#if ADC0_IRQ_ENABLE
	FSM::wrapperADC0_IRQ[i]		= _wrapperIRQ_default;
#endif
#if DAC0_IRQ_ENABLE
	FSM::wrapperDAC0_IRQ[i]		= _wrapperIRQ_default;
#endif
#if I2C0_IRQ_ENABLE
	FSM::wrapperI2C0_IRQ[i]		= _wrapperIRQ_default;
#endif
#if GPIO_ODD_IRQ_ENABLE
	FSM::wrapperGPIO_ODD_IRQ[i]	= _wrapperIRQ_default;
#endif
#if TIMER1_IRQ_ENABLE
	FSM::wrapperTIMER1_IRQ[i]		= _wrapperIRQ_default;
#endif
#if TIMER2_IRQ_ENABLE
	FSM::wrapperTIMER2_IRQ[i]		= _wrapperIRQ_default;
#endif
#if USART1_RX_IRQ_ENABLE
	FSM::wrapperUSART1_RX_IRQ[i]	= _wrapperIRQ_default;
#endif
#if USART1_TX_IRQ_ENABLE
	FSM::wrapperUSART1_TX_IRQ[i]	= _wrapperIRQ_default;
#endif
#if USART2_RX_IRQ_ENABLE
	FSM::wrapperUSART2_RX_IRQ[i]	= _wrapperIRQ_default;
#endif
#if USART2_TX_IRQ_ENABLE
	FSM::wrapperUSART2_TX_IRQ[i]	= _wrapperIRQ_default;
#endif
#if UART0_RX_IRQ_ENABLE
	FSM::wrapperUART0_RX_IRQ[i]	= _wrapperIRQ_default;
#endif
#if UART0_TX_IRQ_ENABLE
	FSM::wrapperUART0_TX_IRQ[i]	= _wrapperIRQ_default;
#endif
#if LEUART0_IRQ_ENABLE
	FSM::wrapperLEUART0_IRQ[i]		= _wrapperIRQ_default;
#endif
#if LEUART1_IRQ_ENABLE
	FSM::wrapperLEUART1_IRQ[i]		= _wrapperIRQ_default;
#endif
#if LETIMER0_IRQ_ENABLE
	FSM::wrapperLETIMER0_IRQ[i]	= _wrapperIRQ_default;
#endif
#if PCNT0_IRQ_ENABLE
	FSM::wrapperPCNT0_IRQ[i]		= _wrapperIRQ_default;
#endif
#if PCNT1_IRQ_ENABLE
	FSM::wrapperPCNT1_IRQ[i]		= _wrapperIRQ_default;
#endif
#if PCNT2_IRQ_ENABLE
	FSM::wrapperPCNT2_IRQ[i]		= _wrapperIRQ_default;
#endif
#if RTC_IRQ_ENABLE
	FSM::wrapperRTC_IRQ[i]			= _wrapperIRQ_default;
#endif
#if CMU_IRQ_ENABLE
	FSM::wrapperCMU_IRQ[i]			= _wrapperIRQ_default;
#endif
#if VCMP_IRQ_ENABLE
	FSM::wrapperVCMP_IRQ[i]		= _wrapperIRQ_default;
#endif
#if LCD_IRQ_ENABLE
	FSM::wrapperLCD_IRQ[i]			= _wrapperIRQ_default;
#endif
#if MSC_IRQ_ENABLE
	FSM::wrapperMSC_IRQ[i]			= _wrapperIRQ_default;
#endif
#if AES_IRQ_ENABLE
	FSM::wrapperAES_IRQ[i]			= _wrapperIRQ_default;
#endif
	}
}
uint8_t FSM::setup( STATUS_BLOCK *statusBlock )
{
	// Copy the Pointer to the Interrupt Handler
	for(uint32_t count = 0; count < statusBlock->numOfISR; count++)
	{
		switch(isrDefinition[count].interruptNumber)
		{
#if DMA_IRQ_ENABLE
		case DMA_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperDMA_IRQ[i] = isrDefinition[count].function;
			else wrapperDMA_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(DMA_IRQn);
			break;
#endif
#if GPIO_EVEN_IRQ_ENABLE
		case GPIO_EVEN_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperGPIO_EVEN_IRQ[i] = isrDefinition[count].function;
			else wrapperGPIO_EVEN_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(GPIO_EVEN_IRQn);
			break;
#endif
#if TIMER0_IRQ_ENABLE
		case TIMER0_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperTIMER0_IRQ[i] = isrDefinition[count].function;
			else wrapperTIMER0_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(TIMER0_IRQn);
			break;
#endif
#if USART0_RX_IRQ_ENABLE
		case USART0_RX_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperUSART0_RX_IRQ[i] = isrDefinition[count].function;
			else wrapperUSART0_RX_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(USART0_RX_IRQn);
			break;
#endif
#if USART0_TX_IRQ_ENABLE
		case USART0_TX_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperUSART0_TX_IRQ[i] = isrDefinition[count].function;
			else wrapperUSART0_TX_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(USART0_TX_IRQn);
			break;
#endif
#if ACMP0_IRQ_ENABLE
		case ACMP0_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperACMP0_IRQ[i] = isrDefinition[count].function;
			else wrapperACMP0_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(ACMP0_IRQn);
			break;
#endif
#if ADC0_IRQ_ENABLE
		case ADC0_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperADC0_IRQ[i] = isrDefinition[count].function;
			else wrapperADC0_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(ADC0_IRQn);
			break;
#endif
#if DAC0_IRQ_ENABLE
		case DAC0_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperDAC0_IRQ[i] = isrDefinition[count].function;
			else wrapperDAC0_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(DAC0_IRQn);
			break;
#endif
#if I2C0_IRQ_ENABLE
		case I2C0_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperI2C0_IRQ[i] = isrDefinition[count].function;
			else wrapperI2C0_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(I2C0_IRQn);
			break;
#endif
#if GPIO_ODD_IRQ_ENABLE
		case GPIO_ODD_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperGPIO_ODD_IRQ[i] = isrDefinition[count].function;
			else wrapperGPIO_ODD_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			GPIO->IFC = ~0;
			NVIC_EnableIRQ(GPIO_ODD_IRQn);
			break;
#endif
#if TIMER1_IRQ_ENABLE
		case TIMER1_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperTIMER1_IRQ[i] = isrDefinition[count].function;
			else wrapperTIMER1_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(TIMER1_IRQn);
			break;
#endif
#if TIMER2_IRQ_ENABLE
		case TIMER2_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperTIMER2_IRQ[i] = isrDefinition[count].function;
			else wrapperTIMER2_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(TIMER2_IRQn);
			break;
#endif
#if USART1_RX_IRQ_ENABLE
		case USART1_RX_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperUSART1_RX_IRQ[i] = isrDefinition[count].function;
			else wrapperUSART1_RX_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(USART1_RX_IRQn);
			break;
#endif
#if USART1_TX_IRQ_ENABLE
		case USART1_TX_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperUSART1_TX_IRQ[i] = isrDefinition[count].function;
			else wrapperUSART1_TX_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(USART1_TX_IRQn);
			break;
#endif
#if USART2_RX_IRQ_ENABLE
		case USART2_RX_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperUSART2_RX_IRQ[i] = isrDefinition[count].function;
			else wrapperUSART2_RX_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(USART2_RX_IRQn);
			break;
#endif
#if USART2_TX_IRQ_ENABLE
		case USART2_TX_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperUSART2_TX_IRQ[i] = isrDefinition[count].function;
			else wrapperUSART2_TX_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(USART2_TX_IRQn);
			break;
#endif
#if UART0_RX_IRQ_ENABLE
		case UART0_RX_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperUART0_RX_IRQ[i] = isrDefinition[count].function;
			else wrapperUART0_RX_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(UART0_RX_IRQn);
			break;
#endif
#if UART0_TX_IRQ_ENABLE
		case UART0_TX_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperUART0_TX_IRQ[i] = isrDefinition[count].function;
			else wrapperUART0_TX_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(UART0_TX_IRQn);
			break;
#endif
#if LEUART0_IRQ_ENABLE
		case LEUART0_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperLEUART0_IRQ[i] = isrDefinition[count].function;
			else wrapperLEUART0_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(LEUART0_IRQn);
			break;
#endif
#if LEUART1_IRQ_ENABLE
		case LEUART1_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperLEUART1_IRQ[i] = isrDefinition[count].function;
			else wrapperLEUART1_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(LEUART1_IRQn);
			break;
#endif
#if LETIMER0_IRQ_ENABLE
		case LETIMER0_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperLETIMER0_IRQ[i] = isrDefinition[count].function;
			else wrapperLETIMER0_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(LETIMER0_IRQn);
			break;
#endif
#if PCNT0_IRQ_ENABLE
		case PCNT0_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperPCNT0_IRQ[i] = isrDefinition[count].function;
			else wrapperPCNT0_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(PCNT0_IRQn);
			break;
#endif
#if PCNT1_IRQ_ENABLE
		case PCNT1_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperPCNT1_IRQ[i] = isrDefinition[count].function;
			else wrapperPCNT1_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(USART0_TX_IRQn);
			break;
#endif
#if PCNT2_IRQ_ENABLE
		case PCNT2_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperPCNT2_IRQ[i] = isrDefinition[count].function;
			else wrapperPCNT2_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(PCNT1_IRQn);
			break;
#endif
#if RTC_IRQ_ENABLE
		case RTC_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperRTC_IRQ[i] = isrDefinition[count].function;
			else wrapperRTC_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(RTC_IRQn);
			break;
#endif
#if CMU_IRQ_ENABLE
		case CMU_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperCMU_IRQ[i] = isrDefinition[count].function;
			else wrapperCMU_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(CMU_IRQn);
			break;
#endif
#if VCMP_IRQ_ENABLE
		case VCMP_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperVCMP_IRQ[i] = isrDefinition[count].function;
			else wrapperVCMP_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			NVIC_EnableIRQ(VCMP_IRQn);
			break;
#endif
#if MSC_IRQ_ENABLE
		case MSC_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperMSC_IRQ[i] = isrDefinition[count].function;
			else wrapperMSC_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			MSC->IFC = ~0;
			NVIC_EnableIRQ(MSC_IRQn);
			break;
#endif
#if AES_IRQ_ENABLE
		case AES_IRQn:
			if(isrDefinition[count].anchorISR)
				for(uint8_t i = 0; i < MAX_NUM_APPS; i++) wrapperAES_IRQ[i] = isrDefinition[count].function;
			else wrapperAES_IRQ[currentStatusBlock->applicationID] = isrDefinition[count].function;
			AES->IFC = ~0;
			NVIC_EnableIRQ(AES_IRQn);
			break;
#endif

		default:
			return 0xFF-isrDefinition[count].interruptNumber;
		}
	}
	
	return 0x00;
}


void FSM::execute(STATUS_BLOCK *statusBlock)
{
	currentStatusBlock = statusBlock;
	currentStatusBlock->nextState = currentStatusBlock->initialState;

	// Run the FSM until the execution is stopped by the Application-Code.
	// The State-Function is called within the WHILE-STATEMENT !!!!
	while( currentStatusBlock->sleepMode ?:( *(stateDefinition)[currentStatusBlock->nextState] )() )
	{
		// Which sleep-mode has to be entered?
		switch( currentStatusBlock->sleepMode )
		{
		case 1:
			EMU_EnterEM1();
			break;
		case 2:
			EMU_EnterEM2( true );
			break;
		case 3:
			EMU_EnterEM3( true );
			break;
		case 4:
			EMU_EnterEM4();
		default:;
		}
	}
}

void FSM::_wrapperIRQ_default( uint32_t test )
{
	(int) test;
}
