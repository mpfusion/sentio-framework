/*
 * LTC2990.h
 *
 *  Created on: Sep 26, 2011
 *      Author: matthias
 */

#ifndef LTC2990_H_
#define LTC2990_H_

#include "em_i2c.h"

typedef enum
{
	Status_Reg   = 0x00,
	Control_Reg  = 0x01,
	Trigger_Reg  = 0x02,
	Temp_MSB_Reg = 0x04,
	VCC_MSB_Reg  = 0x0E
} REG_DRIVER;

typedef enum
{
	Voltage_V1   = 0x06,
	Voltage_V2   = 0x08,
	Voltage_V3   = 0x0A,
	Voltage_V4   = 0x0C
} REG_Voltage;

typedef enum
{
	Current_V12   = 0x06,
	Current_V34   = 0x08
} REG_Current;


typedef enum
{
	kelvin  = 0x80,
	celsius = 0x00
} LTC_TEMP;


typedef enum
{
	repeat  = 0x00,
	single  = 0x40
} LTC_AQUIRE;


typedef enum
{
	V1V2TR2    = 0x00,
	V1_V2TR2   = 0x01,
	V1_V2V3V4  = 0x02,
	TR1V3V4    = 0x03,
	TR1V3_V4   = 0x04,
	TR1TR2     = 0x05,
	V1_V2V3_V4 = 0x06,
	V1V2V3V4   = 0x07
} LTC_MODE;


union LTC_STATUS
{
	struct _LTC_STATUS
	{
		uint8_t zero      : 1;
		uint8_t VCC_Valid : 1;
		uint8_t V4_Vaild  : 1;
		uint8_t V3_Valid  : 1;
		uint8_t V2_Valid  : 1;
		uint8_t V1_Valid  : 1;
		uint8_t TempValid : 1;
		uint8_t Busy      : 1;
	};
	uint8_t array;
};


class LTC2990
{
private:
	I2C_Init_TypeDef i2cInit;
	I2C_TransferSeq_TypeDef i2cTransfer;

	static const uint8_t I2C_ADDRESS_LTC2990 = 0x98;

	uint8_t i2c_txBuffer[2];
	uint8_t i2c_rxBuffer[2];

	float shuntV12;
	float shuntV34;

public:
	LTC2990() {}
	~LTC2990() {}

	void initializeInterfaceLTC();
	void triggerConversion();

	void setLTC_Config( LTC_TEMP temperature, LTC_AQUIRE aquire, LTC_MODE mode, float shuntV12 = 1.0, float shuntV34 = 1.0 );
	void readLTC_Config( LTC_TEMP &temperature, LTC_AQUIRE &aquire, LTC_MODE &mode, float &shuntV12, float &shuntV34 );

	uint8_t readStatusLTC_2990();

	void readTemperature( uint16_t &temperature );
	void readSupplyVoltage( uint16_t &supply );
	void readVoltage( uint16_t &voltage, REG_Voltage selection );
	void readCurrent( uint16_t &current, REG_Current selection );

	void readTemperature( float &temperature );
	void readSupplyVoltage( float &supply );
	void readVoltage( float &voltage, REG_Voltage selection );
	void readCurrent( float &current, REG_Current selection );
};

#endif /* LTC2990_H_ */
