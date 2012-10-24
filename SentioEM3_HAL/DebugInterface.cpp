/*
 * Debug.cpp
 *
 *  Created on: Jan 21, 2011
 *      Author: Matthias Krämer
 */

#include "efm32.h"
#include "efm32_gpio.h"
#include "efm32_usart.h"
#include "efm32_cmu.h"

#include "DebugInterface.h"
#include "SystemConfig.h"


/****************************************************************************************************************************************//**
 * @brief
 *  Constructor used create an instance of the Debuginterface object
 *
 * @details
 * The required parameters of the object are initialized. No dynamic memmory allocation is needed. The Parameter-values can be found in the
 * SystemConfig.h
 *
 *******************************************************************************************************************************************/

DebugInterface::DebugInterface()
{
	// Setup the Parameters used to configure the selected UART/USART module
	initDebugUart.enable       = _DEBUG_enable;      // Enable the UART after the Configuration is finalized
	initDebugUart.refFreq      = _DEBUG_refFreq;	 // The Default HF-Clock Frequency (see CMU) is used to calculate the Baudrate-Clock-Divider Setting
	initDebugUart.baudrate     = _DEBUG_baudrate;	 // Resulting Baudrate at which the UART module runs
	initDebugUart.oversampling = _DEBUG_oversampling;	   // RX-Pin Over-sampling-Rate
	initDebugUart.databits     = _DEBUG_databits;    // Number of Data-Bits
	initDebugUart.parity       = _DEBUG_parity;      // Parity-Bit setup
	initDebugUart.stopbits     = _DEBUG_stopbits;    // Number of Stop-Bits
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method initializes GPIO-Pins used for debugging purposes.
 *
 * @details
 * Dependent on the SystemConfig.h free IO-Pins are activated. The configuration takes into count radio-moduels and SentioEM3 revision blocks
 * the activation of already Pins which are already in use.
 *
 *******************************************************************************************************************************************/

void DebugInterface::initializeInterface( uint32_t numberOfLEDs, uint32_t numberOfButtons )
{
#if SentioEM_Emulator_Interface == OFF && SentioEM_Debug_Interface == ON
	uint32_t permitButtonConfiguration = 4;

	// Enable LED's
	switch( numberOfLEDs )
	{
	case 13:
	#if SentioEM_OnBoard_Button == OFF
		GPIO_PinModeSet( gpioPortC, 11, gpioModePushPull, 0 );
		permitButtonConfiguration = 0;
	#endif

	case 12:
	#ifdef _RADIO_DIGIMESH_
		#if SentioEM_Revision != 2
			GPIO_PinModeSet( digi12, gpioModePushPull, 0);
		#endif
	#endif
	#ifndef _RADIO_DIGIMESH_
		GPIO_PinModeSet( digi12, gpioModePushPull, 0);
		permitButtonConfiguration = 1;
	#endif

	case 11:
		GPIO_PinModeSet( digi11, gpioModePushPull, 0 );
		permitButtonConfiguration = 2;
	case 10:
		GPIO_PinModeSet( digi10, gpioModePushPull, 0 );
		permitButtonConfiguration = 3;
	case 9:
		GPIO_PinModeSet( digi9, gpioModePushPull,0);
	case 8:
		GPIO_PinModeSet( digi8, gpioModePushPull, 0 );
	case 7:
		GPIO_PinModeSet( digi7, gpioModePushPull, 0);
	case 6:
		GPIO_PinModeSet( digi6, gpioModePushPull, 0 );
	case 5:
		GPIO_PinModeSet( digi5 ,gpioModePushPull, 0 );
	case 4:
		GPIO_PinModeSet( digi4, gpioModePushPull, 0 );
	case 3:
		GPIO_PinModeSet( digi3, gpioModePushPull, 0 );
	case 2:
		GPIO_PinModeSet( digi2, gpioModePushPull, 0 );
	case 1:
		GPIO_PinModeSet( digi1, gpioModePushPull, 0 );
	default:
		;
	}

	// Configuration of Pushbuttons on the Debug/Gateway
	if( permitButtonConfiguration <= numberOfButtons )
	switch( numberOfButtons )
	{
	case 4:
		GPIO_PinModeSet(gpioPortC,14,gpioModeInputPullFilter,1);
		GPIO_IntConfig(gpioPortC,14,true,false,true);

	case 3:
		GPIO_PinModeSet(gpioPortC,13,gpioModeInputPullFilter,1);
		GPIO_IntConfig(gpioPortC,13,true,false,true);

	case 2:
	#ifdef _RADIO_DIGIMESH_
		#if SentioEM_Revision != 2
		GPIO_PinModeSet(gpioPortC,12,gpioModeInputPullFilter,1);
		GPIO_IntConfig(gpioPortC,12,true,false,true);
		#endif
	#endif
	#ifndef _RADIO_DIGIMESH_
		GPIO_PinModeSet(gpioPortC,12,gpioModeInputPullFilter,1);
		GPIO_IntConfig(gpioPortC,12,true,false,true);
	#endif

	case 1:
	#if SentioEM_OnBoard_Button == ON
		GPIO_PinModeSet(gpioPortC,11,gpioModeInputPullFilter,1);
		GPIO_IntConfig(gpioPortC,11,true,false,true);
	#endif

	default:;
	}
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method initializes UART/USART used for debugging purposes.
 *
 * @details
 * The driver selects the required Pins required for communication dependent on configuration of UART/USART-number and module location
 * specified in the SystemConfig.h. Then the routing-register is configured and the module is configured (BAUDRATE,...). At last the
 * module-clock, the UART-RX interrupt and the GPIO-Pins are is activated.
 *
 *******************************************************************************************************************************************/

void DebugInterface::initializeDebugUart()
{
#if SentioEM_Emulator_Interface == OFF && SentioEM_Debug_Interface == ON
	// Configuration of the GPIO-Pins which belong to the UART/USART configured. Furthermore the RX-Interrupt
	//Service Routine of the UART is enabled.
	// In a first switch-case statement the UART/USART Module is selected. Then the RX/TX Pins which have
	// to be activated are selected

// Select module UART0, valid Location on the EFM32G280 are (0,1,2,3)
#if _DEBUG_USART_ < 8

	//Enable the required periphery clock
	CMU_ClockEnable(cmuClock_UART0,true);

	// Enable the RX-Interrupt Service Routine
	NVIC_EnableIRQ( UART0_RX_IRQn );

	#define DEBUG_USART  UART0

	// Select the IO-Pins dependent on Module-Location
	#if _DEBUG_USART_ == _UART0_LOC0_
		#define port  gpioPortF
		#define pinRX 7
		#define pinTX 6
		#define DEBUG_LOCATION  USART_ROUTE_LOCATION_LOC0

	#elif _DEBUG_USART_ == _UART0_LOC1_
		#define port  gpioPortE
		#define pinRX 1
		#define pinTX 0
		#define DEBUG_LOCATION  USART_ROUTE_LOCATION_LOC1

	#elif _DEBUG_USART_ == _UART0_LOC2_
		#define port  gpioPortA
		#define pinRX 4
		#define pinTX 3
		#define DEBUG_LOCATION  USART_ROUTE_LOCATION_LOC2

	#elif _DEBUG_USART_ == _UART0_LOC3_
		#define port  gpioPortC
		#define pinRX 15
		#define pinTX 14
		#define DEBUG_LOCATION  USART_ROUTE_LOCATION_LOC3
	#endif

// Select module USART0, valid Location on the EFM32G280 are (0,1,2)
#elif ( _DEBUG_USART_ > 9 ) && ( _DEBUG_USART_ < 19 )

	//Enable the required periphery clock
	CMU_ClockEnable(cmuClock_USART0,true);

	// Enable the RX-Interrupt Service Routine
	NVIC_EnableIRQ( USART0_RX_IRQn );

	#define _DEBUG_USART_  USART0

	// Select the IO-Pins dependent on Module-Location
	#if _DEBUG_USART_ == _USART0_LOC0_
		#define port  gpioPortE
		#define pinRX 11
		#define pinTX 10
		#define DEBUG_LOCATION  USART_ROUTE_LOCATION_LOC0

	#elif _DEBUG_USART_ == _USART0_LOC1_
		#define port  gpioPortE
		#define pinRX 6
		#define pinTX 7
		#define DEBUG_LOCATION  USART_ROUTE_LOCATION_LOC1

	#elif _DEBUG_USART_ == _USART0_LOC2_
		#define port  gpioPortC
		#define	pinRX 10
		#define pinTX 11
		#define DEBUG_LOCATION  USART_ROUTE_LOCATION_LOC2
	#endif


// Select module USART1, valid Location on the EFM32G280 are (0,1)
#elif( _DEBUG_USART_ > 19 ) && ( _DEBUG_USART_ < 29 )

	//Enable the required periphery clock
	CMU_ClockEnable(cmuClock_USART1,true);

	// Enable the RX-Interrupt Service Routine
	NVIC_EnableIRQ( USART1_RX_IRQn );

	#define DEBUG_USART  USART1

	// Select the IO-Pins dependent on Module-Location
	#if _DEBUG_USART_ == _USART1_LOC0_
		#define	port  gpioPortC
		#define	pinRX 1
		#define pinTX 0
		#define DEBUG_LOCATION  USART_ROUTE_LOCATION_LOC0

	#elif _DEBUG_USART_ == _USART1_LOC1_
		#define port  gpioPortD
		#define pinRX 1
		#define pinTX 0
		#define DEBUG_LOCATION  USART_ROUTE_LOCATION_LOC1
	#endif

		// Select module USART2, valid Location on the EFM32G280 are (0,1)
#elif( _DEBUG_USART_ > 29 ) && ( _DEBUG_USART_ < 39 )

	//Enable the required periphery clock
	CMU_ClockEnable(cmuClock_USART1,true);

	// Enable the RX-Interrupt Service Routine
	NVIC_EnableIRQ( USART2_RX_IRQn );

	#define XBEE_USART  USART2

	// Select the IO-Pins dependent on Module-Location
	#if _DEBUG_USART_ == _USART2_LOC0_
		#define port  gpioPortC
		#define pinRX 3
		#define pinTX 2
		#define DEBUG_LOCATION  USART_ROUTE_LOCATION_LOC0

	#elif _DEBUG_USART_ == _USART2_LOC1_
		#define port  gpioPortB
		#define	pinRX 4
		#define	pinTX 3
		#define DEBUG_LOCATION  USART_ROUTE_LOCATION_LOC1
	#endif
#endif


	USART_InitAsync( DEBUG_USART, &initDebugUart );

	// The Routing Register of the selected UART/USART Register is configured. The following register-access
	// enables the UART modules RX/TX shift-register and furthermore selects the one of the possible Locations of the modules IO-Pins
	//
	// NOTE!!!
	// Beside setting a modules Routing Register the functionality of the GPIO-Pins IO-Port-driver has to be configured separately

	DEBUG_USART->ROUTE = USART_ROUTE_RXPEN|
					     USART_ROUTE_TXPEN|
					     DEBUG_LOCATION;

	// Configure the IO-Port driver of the EFM32-GPIO Pins which were selected before
	GPIO_PinModeSet( port, pinRX, gpioModeInputPullFilter, 1 ); // RX needs to be a Input
	GPIO_PinModeSet( port, pinTX, gpioModePushPull, 0 );		   // TX needs to be a Output


	// Configure the Interrupt
	DEBUG_USART->IFC = ~0;  			   // Clear pending Interrupts
	DEBUG_USART->IEN = USART_IEN_RXDATAV;  // Enable the RX-Interrupt when One-Byte is in the buffer,
										   // Flag is cleared when the RX buffer is read-out
#endif
}


/****************************************************************************************************************************************//**
 * @brief
  *  The driver-method sets an EFM32-Pin, which has been configured as output.
 *
 * @param[in]
 *  GPIO_Port_TypeDef port: gpioPortA-F
 * @param[in]
 *  unsigned int      pin:  0-15
 *
 *******************************************************************************************************************************************/

void DebugInterface::pinSet( GPIO_Port_TypeDef port, unsigned int pin )
{
#if SentioEM_Emulator_Interface == OFF && SentioEM_Debug_Interface == ON
	GPIO->P[port].DOUTSET = 1 << pin;
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method clears an EFM32-Pin, which has been configured as output.
 *
 * @param[in]
 *  GPIO_Port_TypeDef port: gpioPortA-F
 * @param[in]
 *  unsigned int      pin:  0-15
 *
 *******************************************************************************************************************************************/

void DebugInterface::pinClear( GPIO_Port_TypeDef port, unsigned int pin )
{
#if SentioEM_Emulator_Interface == OFF && SentioEM_Debug_Interface == ON
	GPIO->P[port].DOUTCLR = 1 << pin;
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method toggels an EFM32-Pin, which has been configured as output.
 *
 * @param[in]
 *  GPIO_Port_TypeDef port: gpioPortA-F
 * @param[in]
 *  unsigned int      pin:  0-15
 *
 *******************************************************************************************************************************************/

void DebugInterface::pinToggle( GPIO_Port_TypeDef port, unsigned int pin )
{
#if SentioEM_Emulator_Interface == OFF && SentioEM_Debug_Interface == ON
	GPIO->P[port].DOUTTGL = 1 << pin;
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method converts a unsigned Hex-Value into a string of ASCII-Characters.
 *
 * @info
 *  !!! Used internally !!!
 *
 *******************************************************************************************************************************************/

void DebugInterface::printHexadecimal( uint32_t input, uint8_t size, bool newline )
{
#if SentioEM_Emulator_Interface == OFF && SentioEM_Debug_Interface == ON

	uint8_t temp[8];
	uint8_t i = size;

	do
	{
		temp[i - 1] = ( input &  0x000F );
		input >>= 4;
		--i;
	} while( i );

	for( i = 0; i < size; i++ )
	{
		USART_Tx( DEBUG_USART, ( temp[i] + ( (  temp[i] <= 0x09 ) ? '0' : '7' ) ) );
	}

	if( newline )
	{
		USART_Tx( DEBUG_USART, '\n' );
		USART_Tx( DEBUG_USART, '\r' );
	}
	while( !( DEBUG_USART->STATUS & USART_STATUS_TXC ) );
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method prints an unsigned Hex-Value (8-Bit) on the Debug-UART
 *
 * @param[in]
 *	input: HEX-Value to be written on UART
 *******************************************************************************************************************************************/

void DebugInterface::printHex( uint8_t input, bool newline )
{
#if SentioEM_Emulator_Interface == OFF && SentioEM_Debug_Interface == ON
	printHexadecimal( (uint32_t) input, 2, newline );
#endif
}



/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method prints an unsigned Hex-Value (16-Bit) on the Debug-UART
 *
 * @param[in]
 *	input: HEX-Value to be written on UART
 *******************************************************************************************************************************************/

void DebugInterface::printHex( uint16_t input, bool newline )
{
#if SentioEM_Emulator_Interface == OFF && SentioEM_Debug_Interface == ON
	printHexadecimal( (uint32_t) input, 4, newline );
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method prints an unsigned Hex-Value (32-Bit) on the Debug-UART
 *
 * @param[in]
 *	input: HEX-Value to be written on UART
 *******************************************************************************************************************************************/

void DebugInterface::printHex( uint32_t input, bool newline )
{
#if SentioEM_Emulator_Interface == OFF && SentioEM_Debug_Interface == ON
	printHexadecimal( (uint32_t) input, 8, newline );
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method prints an signed Hex-Value (8-Bit) on the Debug-UART
 *
 * @param[in]
 *	input: HEX-Value to be written on UART
 *******************************************************************************************************************************************/

void DebugInterface::printHex( int8_t input, bool newline )
{
#if SentioEM_Emulator_Interface == OFF && SentioEM_Debug_Interface == ON

	if(input&0x80)
	{
		USART_Tx( DEBUG_USART, '-' );

		input = ~input + 1;
	}

	printHexadecimal( (uint32_t) input, 2, newline );
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method prints an signed Hex-Value (16-Bit) on the Debug-UART
 *
 * @param[in]
 *	input: HEX-Value to be written on UART
 *******************************************************************************************************************************************/

void DebugInterface::printHex( int16_t input, bool newline )
{
#if SentioEM_Emulator_Interface == OFF && SentioEM_Debug_Interface == ON

	if(input&0x8000)
	{
		USART_Tx( DEBUG_USART, '-' );
		input = ~input + 1;
	}

	printHexadecimal( (uint32_t) input, 4, newline );
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method prints an signed Hex-Value (32-Bit) on the Debug-UART
 *
 * @param[in]
 *	input: HEX-Value to be written on UART
 *******************************************************************************************************************************************/

void DebugInterface::printHex( int32_t input, bool newline )
{
#if SentioEM_Emulator_Interface == OFF && SentioEM_Debug_Interface == ON

	if(input&0x80000000)
	{
		USART_Tx( DEBUG_USART, '-' );
		input = ~input + 1;
	}

	printHexadecimal( (uint32_t) input, 8, newline );
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method prints an unsigned Decimal-Value (8Bit to 32-Bit) on the Debug-UART
 *
 * @param[in]
 *	input: DEC-Value to be written on UART
 *******************************************************************************************************************************************/

uint8_t DebugInterface::printDecimal( uint32_t input, bool newline )
{
#if SentioEM_Emulator_Interface == OFF && SentioEM_Debug_Interface == ON

	uint8_t temp[11];
	uint8_t length;

	uint8_t  i = 0;

	do
	{
		temp[i] = input%10;
		input /=10;
		i++;
	} while( input );

	length = i;

	do
	{
		USART_Tx( DEBUG_USART, temp[i-1] + '0' );

		i--;
	}while( i );

	if( newline )
	{
		USART_Tx( DEBUG_USART, '\n' );
		USART_Tx( DEBUG_USART, '\r' );
	}

	while( !( DEBUG_USART->STATUS & USART_STATUS_TXC ) );
#endif

	return length;
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method prints an Floating-Point-Value (32-Bit) on the Debug-UART
 *
 * @param[in]
 *	input: float-Value to be written on UART
 *******************************************************************************************************************************************/

void DebugInterface::printFloat( float input, uint8_t displayLength, bool newline )
{
#if SentioEM_Emulator_Interface == OFF && SentioEM_Debug_Interface == ON

	uint8_t length;

	if(input < 0)
	{
		USART_Tx( DEBUG_USART, '-' );
		input *= -1;
	}

	length = printDecimal( (uint32_t) input, false );

	if( length <= displayLength )
		USART_Tx( DEBUG_USART, '.' );

	while( length < displayLength )
	{

		input = input - (uint32_t)input;

		input *= 10;
		USART_Tx( DEBUG_USART, ( (uint8_t) input ) + ( ( (uint8_t) input > 0x09 ) ? '7' : '0' ) );

		length++;
	}

	if( newline )
	{
		USART_Tx( DEBUG_USART, '\n' );
		USART_Tx( DEBUG_USART, '\r' );
	}

	while( !( DEBUG_USART->STATUS & USART_STATUS_TXC ) );
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method prints an signed Hex-Value (32-Bit) on the Debug-UART
 *
 * @param[in]
 *	input: character Array to be written
 * @param[in]
 *  newline: en/disable the line-break at the end of the string
 *******************************************************************************************************************************************/

void DebugInterface::printLine( const char *input, bool newline )
{
#if SentioEM_Emulator_Interface == OFF && SentioEM_Debug_Interface == ON

	uint8_t i = 0;

	do
	{
		USART_Tx( DEBUG_USART, input[i] );
		i++;

	} while( ( input[i] != '\0' ) );

	if( newline )
	{
		USART_Tx( DEBUG_USART, '\n' );
		USART_Tx( DEBUG_USART, '\r' );
	}

	while( !( DEBUG_USART->STATUS & USART_STATUS_TXC ) );
#endif
}

time DebugInterface::printTime( time input, bool newline )
{
#if SentioEM_Emulator_Interface == OFF && SentioEM_Debug_Interface == ON

	printLine("\n\rTime in Sec:", false);
	printDecimal( input.getInSeconds(), newline );

	while( !( DEBUG_USART->STATUS & USART_STATUS_TXC ) );
#endif

	return input;
}

time DebugInterface::printTimeDet( time input, bool newline )
{
#if SentioEM_Emulator_Interface == OFF && SentioEM_Debug_Interface == ON

	printLine("\n\rSec:", false );
	printDecimal( input.getSecond(), false );
	printLine(" Min:", false );
	printDecimal( input.getMinute(), false );
	printLine(" Hr:", false );
	printDecimal( input.getHour(),false );

	if( newline )
	{
		USART_Tx( DEBUG_USART, '\n' );
		USART_Tx( DEBUG_USART, '\r' );
	}

	while( !( DEBUG_USART->STATUS & USART_STATUS_TXC ) );
#endif

	return input;
}
