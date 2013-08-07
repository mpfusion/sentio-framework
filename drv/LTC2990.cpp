/*
 * LTC2990.cpp
 *
 *  Created on: Sep 26, 2011
 *      Author: matthias
 */

#include "LTC2990.h"
#include "SystemConfig.h"

#include "efm32.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_i2c.h"
#include "em_usart.h"


void LTC2990::initializeInterfaceLTC()
{
	i2cInit.master  = true;
	i2cInit.enable  = true;
	i2cInit.refFreq = 0;
	i2cInit.freq    = 350000;
	i2cInit.refFreq = _I2C_CTRL_CLHR_STANDARD;

	CMU_ClockEnable( cmuClock_I2C0, true );

	GPIO_PinModeSet( gpioPortD, 7, gpioModeWiredAndPullUpFilter, 1 );
	GPIO_PinModeSet( gpioPortD, 6, gpioModeWiredAndPullUpFilter, 1 );

	/* Enable pins at location 1 */
	I2C0->ROUTE = I2C_ROUTE_SDAPEN |
				  I2C_ROUTE_SCLPEN |
				  I2C_ROUTE_LOCATION_LOC1;

	I2C_Init( I2C0, &i2cInit );

	i2cTransfer.buf[0].data = i2c_txBuffer;
	i2cTransfer.buf[1].data = i2c_rxBuffer;
	i2cTransfer.addr        = I2C_ADDRESS_LTC2990;
}


void LTC2990::setLTC_Config( LTC_TEMP temperature, LTC_AQUIRE aquire, LTC_MODE modeLSB, float _shuntV12_, float _shuntV34_ )
{
	uint8_t modeMSB = 0x18;
	shuntV12 = _shuntV12_;
	shuntV34 = _shuntV34_;

	i2c_txBuffer[0]         = Control_Reg;
	i2c_txBuffer[1]         = ( temperature | aquire | modeMSB | modeLSB );

	i2cTransfer.buf[0].len  = 0x02;
	i2cTransfer.buf[1].len  = 0x00;

	i2cTransfer.flags       = I2C_FLAG_WRITE_WRITE;

	I2C_TransferInit( I2C0, &i2cTransfer );

	/* Sending data */
	while ( I2C_Transfer( I2C0 ) == i2cTransferInProgress );
}


void LTC2990::readLTC_Config( LTC_TEMP &temperature, LTC_AQUIRE &aquire, LTC_MODE &mode, float &_shuntV12_, float &_shuntV34_ )
{
	_shuntV12_ = shuntV12;
	_shuntV34_ = shuntV34;

	i2c_txBuffer[0]         = Control_Reg;

	i2cTransfer.buf[0].len  = 0x01;
	i2cTransfer.buf[1].len  = 0x01;

	i2cTransfer.flags       = I2C_FLAG_WRITE_READ;

	I2C_TransferInit( I2C0, &i2cTransfer );

	/* Sending data */
	while ( I2C_Transfer( I2C0 ) == i2cTransferInProgress );

	temperature = ( LTC_TEMP )( i2c_rxBuffer[0] & 0x80 );
	aquire      = ( LTC_AQUIRE )( i2c_rxBuffer[0] & 0x40 );
	mode         = ( LTC_MODE )( i2c_rxBuffer[0] & 0x07 );
}


uint8_t LTC2990::readStatusLTC_2990()
{
	i2c_txBuffer[0]         = Status_Reg;

	i2cTransfer.buf[0].len  = 0x01;
	i2cTransfer.buf[1].len  = 0x01;

	i2cTransfer.flags       = I2C_FLAG_WRITE_READ;

	I2C_TransferInit( I2C0, &i2cTransfer );

	/* Sending data */
	while ( I2C_Transfer( I2C0 ) == i2cTransferInProgress );

	return i2c_rxBuffer[0];
}


void LTC2990::readTemperature( uint16_t &temperature )
{

	i2c_txBuffer[0]         = Temp_MSB_Reg;

	i2cTransfer.buf[0].len  = 0x01;
	i2cTransfer.buf[1].len  = 0x02;

	i2cTransfer.flags       = I2C_FLAG_WRITE_READ;

	for ( ever )
	{
		I2C_TransferInit( I2C0, &i2cTransfer );

		/* Sending data */
		while ( I2C_Transfer( I2C0 ) == i2cTransferInProgress );

		if ( !( i2c_rxBuffer[0] & 0x80 ) )
		{
			triggerConversion();
		}

		else
			break;
	}

#ifdef MSB_FIRST
	temperature = ( ( i2c_rxBuffer[0] << 8 ) | i2c_rxBuffer[1] ) & 0x1FFF;
#else
	temperature = ( ( i2c_rxBuffer[1] << 8 ) | i2c_rxBuffer[0] ) & 0x1FFF;
#endif

}


void LTC2990::readSupplyVoltage( uint16_t &supply )
{
	i2c_txBuffer[0]         = VCC_MSB_Reg;

	i2cTransfer.buf[0].len  = 0x01;
	i2cTransfer.buf[1].len  = 0x02;

	i2cTransfer.flags       = I2C_FLAG_WRITE_READ;

	for ( ever )
	{
		I2C_TransferInit( I2C0, &i2cTransfer );

		/* Sending data */
		while ( I2C_Transfer( I2C0 ) == i2cTransferInProgress );

		if ( !( i2c_rxBuffer[0] & 0x80 ) )
		{
			triggerConversion();
		}
		else
			break;
	}

#ifdef MSB_FIRST
	supply = ( ( i2c_rxBuffer[0] << 8 ) | i2c_rxBuffer[1] ) & 0x1FFF;
#else
	supply = ( ( i2c_rxBuffer[1] << 8 ) | i2c_rxBuffer[0] ) & 0x1FFF;
#endif

}


void LTC2990::readVoltage( uint16_t &voltage, REG_Voltage selection )
{
	i2c_txBuffer[0]         = selection;

	i2cTransfer.buf[0].len  = 0x01;
	i2cTransfer.buf[1].len  = 0x02;

	i2cTransfer.flags       = I2C_FLAG_WRITE_READ;

	for ( ever )
	{
		I2C_TransferInit( I2C0, &i2cTransfer );

		/* Sending data */
		while ( I2C_Transfer( I2C0 ) == i2cTransferInProgress );

		if ( !( i2c_rxBuffer[0] & 0x80 ) )
		{
			triggerConversion();
		}
		else
			break;
	}

#ifdef MSB_FIRST
	voltage = ( ( i2c_rxBuffer[0] << 8 ) | i2c_rxBuffer[1] ) & 0x1FFF;
#else
	voltage = ( ( i2c_rxBuffer[1] << 8 ) | i2c_rxBuffer[0] ) & 0x1FFF;
#endif

}


void LTC2990::readCurrent( uint16_t &current, REG_Current selection )
{
	i2c_txBuffer[0]         = selection;

	i2cTransfer.buf[0].len  = 0x01;
	i2cTransfer.buf[1].len  = 0x02;

	i2cTransfer.flags       = I2C_FLAG_WRITE_READ;

	for ( ever )
	{
		I2C_TransferInit( I2C0, &i2cTransfer );

		/* Sending data */
		while ( I2C_Transfer( I2C0 ) == i2cTransferInProgress );

		if ( !( i2c_rxBuffer[0] & 0x80 ) )
		{
			triggerConversion();
		}
		else
			break;
	}

#ifdef MSB_FIRST
	current = ( ( i2c_rxBuffer[0] << 8 ) | i2c_rxBuffer[1] ) & 0x1FFF;
#else
	current = ( ( i2c_rxBuffer[1] << 8 ) | i2c_rxBuffer[0] ) & 0x1FFF;
#endif

}


void LTC2990::readTemperature( float &temperature )
{
	uint8_t auxillary;

	do
	{
		auxillary = ( uint8_t ) readStatusLTC_2990();
	}
	while ( auxillary & 0x01 );

	i2c_txBuffer[0]         = Temp_MSB_Reg;

	i2cTransfer.buf[0].len  = 0x01;
	i2cTransfer.buf[1].len  = 0x02;

	i2cTransfer.flags       = I2C_FLAG_WRITE_READ;

	I2C_TransferInit( I2C0, &i2cTransfer );

	/* Sending data */
	while ( I2C_Transfer( I2C0 ) == i2cTransferInProgress );

	temperature = ( float )( ( ( i2c_rxBuffer[0] << 8 ) | i2c_rxBuffer[1] ) & 0x1FFF ) * 0.0625;
}


void LTC2990::readSupplyVoltage( float &supply )
{

	uint8_t auxillary;

	do
	{
		auxillary = ( uint8_t ) readStatusLTC_2990();

	}
	while ( auxillary & 0x01 );

	i2c_txBuffer[0]         = VCC_MSB_Reg;

	i2cTransfer.buf[0].len  = 0x01;
	i2cTransfer.buf[1].len  = 0x02;

	i2cTransfer.flags       = I2C_FLAG_WRITE_READ;

	I2C_TransferInit( I2C0, &i2cTransfer );

	/* Sending data */
	while ( I2C_Transfer( I2C0 ) == i2cTransferInProgress );


	supply = ( float )( ( ( ( i2c_rxBuffer[0] << 8 ) | i2c_rxBuffer[1] ) & 0x7FFF ) * 305.18e-6 ) + 2.5;
}


void LTC2990::readVoltage( float &voltage, REG_Voltage selection )
{
	uint8_t auxillary;

	do
	{
		auxillary = readStatusLTC_2990();
	}
	while ( auxillary & 0x01 );


	i2c_txBuffer[0]         = selection;

	i2cTransfer.buf[0].len  = 0x01;
	i2cTransfer.buf[1].len  = 0x02;

	i2cTransfer.flags       = I2C_FLAG_WRITE_READ;

	I2C_TransferInit( I2C0, &i2cTransfer );

	/* Sending data */
	while ( I2C_Transfer( I2C0 ) == i2cTransferInProgress );


	if ( i2c_rxBuffer[0] & 0x40 )
		voltage = ( ( float )( ~( ( ( i2c_rxBuffer[0] << 8 ) | i2c_rxBuffer[1] ) & 0x3FFF ) + 1 ) ) * ( -305.18e-6 );
	else
		voltage = ( ( float )( ( ( i2c_rxBuffer[0] << 8 ) | i2c_rxBuffer[1] ) & 0x3FFF ) ) * 305.18e-6;
}


void LTC2990::readCurrent( float &current, REG_Current selection )
{
	uint8_t auxillary;

	do
	{
		auxillary = ( uint8_t ) readStatusLTC_2990();
	}
	while ( auxillary & 0x01 );

	float shunt = 0.0;

	i2c_txBuffer[0]         = selection;

	i2cTransfer.buf[0].len  = 0x01;
	i2cTransfer.buf[1].len  = 0x02;

	i2cTransfer.flags       = I2C_FLAG_WRITE_READ;

	for ( ever )
	{
		I2C_TransferInit( I2C0, &i2cTransfer );

		/* Sending data */
		while ( I2C_Transfer( I2C0 ) == i2cTransferInProgress );

		if ( !( i2c_rxBuffer[0] & 0x80 ) )
		{
			triggerConversion();
		}

		else
			break;
	}

	switch ( selection )
	{
	case Current_V12:
		shunt = shuntV12;
		break;

	case Current_V34:
		shunt = shuntV34;
		break;

	default:
		;
	}

	if ( i2c_rxBuffer[0] & 0x40 )
		current = ( ( float )( ~( ( ( i2c_rxBuffer[0] << 8 ) | i2c_rxBuffer[1] ) & 0x3FFF ) + 1 ) * ( -19.42e-6 ) ) / shunt;
	else
		current = ( ( float )( ( ( i2c_rxBuffer[0] << 8 ) | i2c_rxBuffer[1] ) & 0x3FFF ) * ( 19.42e-6 ) ) / shunt;
}


void LTC2990::triggerConversion()
{
	i2c_txBuffer[0]         = Trigger_Reg;
	i2c_txBuffer[1]         = 0x00;


	i2cTransfer.buf[0].len  = 0x02;
	i2cTransfer.buf[1].len  = 0x00;

	i2cTransfer.flags       = I2C_FLAG_WRITE_WRITE;

	I2C_TransferInit( I2C0, &i2cTransfer );

	/* Sending data */
	while ( I2C_Transfer( I2C0 ) == i2cTransferInProgress );
}
