/*
 *  CC1101_Radio.cpp
 *
 *  Created on: 10/01/2013
 *  Author: Linlin, Sebastian
 */

#include "CC1101_Radio.h"

CC1101_Radio::CC1101_Radio()
{
	interfaceInit.baudrate  = _CC1101_baudrate;
	interfaceInit.databits  = _CC1101_databits;
	interfaceInit.enable    = _CC1101_enable;
	interfaceInit.refFreq   = _CC1101_refFreq;
	interfaceInit.master    = _CC1101_slave;
	interfaceInit.msbf      = _CC1101_msbf;
	interfaceInit.clockMode = _CC1101_clockMode;

	cc1101Location          = _CC1101_Location;
	cc1101USART             = _CC1101_USART;
}

/**************************************************************************************************************************
 *
 * User functions
 *
 *************************************************************************************************************************/

/*
 *  Hardware initialization for the radio connection
 */
void CC1101_Radio::initializeInterface()
{
	CMU_ClockEnable( cmuClock_USART0, true );

	USART_InitSync( cc1101USART, &interfaceInit );
	USART_Enable( cc1101USART, usartEnable );

	cc1101USART->ROUTE = USART_ROUTE_RXPEN | // Enable the MISO-Pin
						 USART_ROUTE_TXPEN |   // Enable the MOSI-Pin
						 USART_ROUTE_CLKPEN |  // Enable the SPI-Clock Pin
						 cc1101Location;

	GPIO_PinModeSet( _CC1101_SPI_MOSI_PIN, gpioModePushPull, 1 );
	GPIO_PinModeSet( _CC1101_SPI_MISO_PIN, gpioModeInputPull, 1 );
	GPIO_PinModeSet( _CC1101_SPI_CLK_PIN, gpioModePushPull, 1 );
	GPIO_PinModeSet( _CC1101_SPI_CS_PIN, gpioModePushPull, 1 );
}

/*
 *  Set the radio configuration
 */
void CC1101_Radio::setRfConfig()
{
	// Write the default configuration in the radio registers
	multi_register_access( 0x00, CC1101_WRITE_BURST, ( uint8_t* )rf_conf.bytes, 47 );
}

/*
 *  Interrupt GDO0 init
 */
void CC1101_Radio::initializeRadioInterrupt0( bool rising, bool falling )
{
	GPIO_PinModeSet( _CC1101_INT_PIN_ONE, gpioModeInputPull, 1 );
	GPIO->IFC = ~0;
	GPIO_IntConfig( _CC1101_INT_PIN_ONE, rising, falling, true );
	NVIC_EnableIRQ( GPIO_EVEN_IRQn );
}

/*
 *  Interrupt GDO2 init
 */
void CC1101_Radio::initializeRadioInterrupt2( bool rising, bool falling )
{
	GPIO_PinModeSet( _CC1101_INT_PIN_TWO, gpioModeInputPull, 1 );
	GPIO->IFC = ~0;
	GPIO_IntConfig( _CC1101_INT_PIN_TWO, rising, falling, true );
	NVIC_EnableIRQ( GPIO_EVEN_IRQn );
}

/*
 *  Send RF packet
 */
void CC1101_Radio::sendPacket( uint8_t type, uint8_t addr = 0, uint8_t* payload = 0, uint8_t payload_length = 0 )
{
	uint8_t buffer[61];
	uint16_t delay;

	for ( uint8_t i = 0; i < 61; i++ )
		buffer[i] = 0;

	buffer[0] = payload_length + 2;
	buffer[1] = addr;
	buffer[2] = type;

	for ( uint8_t j = 0; j < payload_length; j++ )
		buffer[j + 3] = payload[j];

	strobe( CC1101_SIDLE ); // Set radio idle
	strobe( CC1101_SFTX );  // Flush the TXFIFO

	for ( delay = 0; delay <= 10000; delay++ );

	// Write the data into the TXFIFO
	multi_register_access( CC1101_TXFIFO, CC1101_WRITE_BURST, ( uint8_t* )buffer, payload_length + 3 );

	for ( delay = 0; delay <= 10000; delay++ );

	strobe( CC1101_STX );   // Set radio to transmit
}

/*
 *  Read RF packet
 */
void CC1101_Radio::readPacket()
{
	receive_buffer.fields.length = read_register_SPI( CC1101_RXFIFO, CC1101_READ_SINGLE );

	// Read the data from the RXFIFO
	multi_register_access( CC1101_RXFIFO, CC1101_READ_BURST, ( uint8_t* )&receive_buffer.bytes[1], receive_buffer.fields.length );
	multi_register_access( CC1101_RXFIFO, CC1101_READ_BURST, ( uint8_t* )&receive_buffer.fields.rssi, 2 );
}

/*
 *  Get the length of the last packet
 */
uint8_t CC1101_Radio::getPacketLength()
{
	return receive_buffer.fields.length;
}

/*
 *  Get the address of the last packet
 */
uint8_t CC1101_Radio::getPacketAddress()
{
	return receive_buffer.fields.addr;
}

/*
 *  Get the type of the last packet
 */
uint8_t CC1101_Radio::getPacketType()
{
	return ( uint8_t )receive_buffer.fields.type;
}

/*
 *  Get the payload of the last packet
 */
void CC1101_Radio::getPacketPayload( uint8_t* array, uint8_t start, uint8_t end )
{
	for ( uint8_t i = start; i <= end; i++ )
		array[i - start] = receive_buffer.fields.payload[i];
}

/*
 *  Set radio to receive status
 */
void CC1101_Radio::setReceiveMode()
{
	uint16_t delay;

	strobe( CC1101_SIDLE ); // Set radio idle
	strobe( CC1101_SFRX );  // Flush the RXFIFO
	for ( delay = 0; delay <= 1000; delay++ );
	strobe( CC1101_SRX );   // Set radio to receive
}

/*
 *  Set RF output power
 */
void CC1101_Radio::setOutputPower( int8_t power )
{
	uint8_t paSetting;
	uint8_t frend0_temp;

	// Translate the user power setting
	switch ( power )
	{
	case -30:
		paSetting = 0x12;
		break;
	case -20:
		paSetting = 0x0E;
		break;
	case -15:
		paSetting = 0x1D;
		break;
	case -10:
		paSetting = 0x34;
		break;
	case 0:
		paSetting = 0x60;
		break;
	case 5:
		paSetting = 0x84;
		break;
	case 7:
		paSetting = 0xC8;
		break;
	case 10:
		paSetting = 0xC0;
		break;

	default :
		paSetting = 0xC0;
		break;
	}

	frend0_temp = read_register_SPI( CC1101_FREND0, CC1101_READ_SINGLE ); // Read the FREND0 register
	write_register_SPI( CC1101_FREND0, frend0_temp & 0xF8 );            // Clear the POWER bits (i.e. single power value)
	write_register_SPI( CC1101_PATABLE, paSetting );                    // Set the respective power value
}

/*
 *  Get the RF output power setting
 */
int8_t CC1101_Radio::getOutputPower()
{
	uint8_t paSetting;

	paSetting = read_register_SPI( CC1101_PATABLE, CC1101_READ_SINGLE ); // Read the power value
	int8_t power;

	// Translate the radio power setting
	switch ( paSetting )
	{
	case 0x12:
		power = -30;
		break;
	case 0x0E:
		power = -20;
		break;
	case 0x1D:
		power = -15;
		break;
	case 0x34:
		power = -10;
		break;
	case 0x60:
		power = 0;
		break;
	case 0x84:
		power = 5;
		break;
	case 0xC8:
		power = 7;
		break;
	case 0xC0:
		power = 10;
		break;

	default :
		power = 10;
		break;
	}

	return power;
}

/*
 *  Enable CRC checking
 */
void CC1101_Radio::setCrcCheck( bool state )
{
	uint8_t pktctrl0_temp;

	pktctrl0_temp = read_register_SPI( CC1101_PKTCTRL0, CC1101_READ_SINGLE ); // Read the PKTCTRL0 register

	if ( state )
		write_register_SPI( CC1101_PKTCTRL0, pktctrl0_temp | 0x04 ); // Set the CRC bit
	else
		write_register_SPI( CC1101_PKTCTRL0, pktctrl0_temp & 0xFB ); // Clear the CRC bit
}

/*
 *  Get the packets CRC status
 */
uint8_t CC1101_Radio::getCrcStatus()
{
	return receive_buffer.fields.crc >> 7;
}

/*
 *  Get the packets RSSI value
 */
int8_t CC1101_Radio::getRssiValue()
{
	uint8_t rssi_temp;
	int8_t rssi;

	rssi_temp = receive_buffer.fields.rssi;

	if ( rssi_temp >= 128 )
		rssi = ( rssi_temp - 256 ) / 2 - 74;
	else
		rssi = rssi_temp / 2 - 74;

	return rssi;
}

/*
 *  Get the packets LQI value
 */
uint8_t CC1101_Radio::getLqiValue()
{
	return ( uint8_t )receive_buffer.fields.crc & 0x7F;
}

/*
 *  Enable hardware address check
 */
void CC1101_Radio::setAddressCheck( bool state )
{
	uint8_t pktctrl1_temp;

	pktctrl1_temp = read_register_SPI( CC1101_PKTCTRL1, CC1101_READ_SINGLE ); // Read the PKTCTRL1 register

	if ( state )
		write_register_SPI( CC1101_PKTCTRL1, pktctrl1_temp | 0x02 ); // Set the addr_check bits
	else
		write_register_SPI( CC1101_PKTCTRL1, pktctrl1_temp & 0xFC ); // Clear the addr_check bits
}

/*
 *  Set the radio address
 */
void CC1101_Radio::setAddress( uint8_t address )
{
	write_register_SPI( CC1101_ADDR, address ); // Set the specified device address
}

/*
 *  Get the radio address
 */
uint8_t CC1101_Radio::getAddress()
{
	return read_register_SPI( CC1101_ADDR, CC1101_READ_SINGLE ); // Read the device address
}

/*
 *  Set radio to sleep
 */
void CC1101_Radio::setSleepMode()
{
	strobe( CC1101_SIDLE ); // Go first to idle
	strobe( CC1101_SPWD );  // Then power down
}

/**************************************************************************************************************************
 *
 * Low-level functions
 *
 *************************************************************************************************************************/

/*
 *  SPI register access
 */
uint8_t CC1101_Radio::register_access_SPI( uint8_t address, uint8_t access_mode, uint8_t data )
{
	uint8_t reg = 0;

	GPIO_PinOutClear( _CC1101_SPI_CS_PIN );
	USART_Tx( _CC1101_USART, address | access_mode ); // write the address byte
	USART_Rx( _CC1101_USART );                      // read reply
	USART_Tx( _CC1101_USART, data );                // write the data byte(s)
	reg = USART_Rx( _CC1101_USART );                // read and save the reply
	GPIO_PinOutSet( _CC1101_SPI_CS_PIN );

	return reg;
}

/*
 *  Access multiple registers
 */
uint8_t CC1101_Radio::multi_register_access( uint8_t address, uint8_t access_mode, uint8_t* data, uint8_t length )
{
	uint8_t reg;
	uint8_t status;
	uint8_t count;

	GPIO_PinOutClear( _CC1101_SPI_CS_PIN );
	USART_Tx( _CC1101_USART, address | access_mode );
	status = USART_Rx( _CC1101_USART );

	for ( count = 0; count < length; count++ )
	{
		if ( ( access_mode == CC1101_READ_SINGLE ) || ( access_mode == CC1101_READ_BURST ) )
		{
			USART_Tx( _CC1101_USART, CC1101_SNOP );
			reg = USART_Rx( _CC1101_USART );
			data[count] = ( uint8_t )reg;
		}
		else
		{
			USART_Tx( _CC1101_USART, data[count] );
			status = USART_Rx( _CC1101_USART );
		}
	}

	GPIO_PinOutSet( _CC1101_SPI_CS_PIN );

	return status;
}

/*
 *  SPI register read
 */
uint8_t CC1101_Radio::read_register_SPI( uint8_t address, uint8_t access_mode )
{
	return register_access_SPI( address, access_mode, CC1101_SNOP );
}

/*
 *  SPI register write
 */
uint8_t CC1101_Radio::write_register_SPI( uint8_t address, uint8_t data )
{
	return register_access_SPI( address, CC1101_WRITE_BURST, data );
}

/*
 *  Send a strobe command
 */
uint8_t CC1101_Radio::strobe( uint8_t l_strobe )
{
	uint8_t reg = 0;

	GPIO_PinOutClear( _CC1101_SPI_CS_PIN );
	USART_Tx( _CC1101_USART, l_strobe );    // Write strobe command
	reg = USART_Rx( _CC1101_USART );        // Read reply
	GPIO_PinOutSet( _CC1101_SPI_CS_PIN );

	return reg;
}

/*
 *  Get RF status
 */
uint8_t CC1101_Radio::getStatus()
{
	return read_register_SPI( CC1101_SNOP, CC1101_READ_BURST );
}
