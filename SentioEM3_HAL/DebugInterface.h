/*
 * Debug.h
 *
 *  Created on: Jan 21, 2011
 *      Author: Matthias Kr�mer
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include "SystemConfig.h"
#include "time.h"
#include "efm32_usart.h"
#include "efm32_gpio.h"



/****************************************************************************************************************************************//**
 * @brief
 *  Declaration of the DebugInterface-Class
 *
 * @details
 * The DebugInterface includes UART communication and GPIO-Pins (LED and Pushbutton) used for debugging purpose. Specifications are done in
 * the SystemConfig.h (UART-Parameters, used modules and functions are configurable)
 *
 *******************************************************************************************************************************************/

class DebugInterface {

private:
	USART_InitAsync_TypeDef  initDebugUart;
	void printHexadecimal( uint32_t input, uint8_t size, bool newline = false );

public:
	DebugInterface();
	~DebugInterface(){};

	void initializeInterface( uint32_t numberOfLEDs, uint32_t numberOfButtons );
	void initializeDebugUart();

	void pinSet( GPIO_Port_TypeDef port, unsigned int pin );
	void pinClear( GPIO_Port_TypeDef port, unsigned int pin );
	void pinToggle( GPIO_Port_TypeDef port, unsigned int pin );

	void printHex(uint8_t, bool newline = false );
	void printHex(uint16_t, bool newline = false );
	void printHex(uint32_t, bool newline = false );
	void printHex(int8_t, bool newline = false );
	void printHex(int16_t, bool newline = false );
	void printHex(int32_t, bool newline = false );


	uint8_t printDecimal(uint32_t input, bool newline = false );

	void printFloat(float input, uint8_t length = 10, bool newline = false );
	void printLine( const char *input, bool newline = true );

	time printTime(time input, bool newline = false  );
	time printTimeDet( time input, bool newline = false   );
};

#endif /* DEBUG_H_ */
