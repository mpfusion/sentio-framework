/*
 * SHT1X_Sensirion.h
 *
 *  Created on: Apr 11, 2011
 *      Author: Matthias Krämer
 */

#ifndef SHT1X_HUMID_TEMP_H_
#define SHT1X_HUMID_TEMP_H_

#include "efm32_gpio.h"
#include "SystemConfig.h"


/****************************************************************************************************************************************//**
 * @brief
 *  Definition of SHT1X_Sensirion related parameters
 *
 *******************************************************************************************************************************************/

#define noACK 0
#define ACK   1
                            //adr  command  r/w
#define STATUS_REG_W 0x06   //000   0011    0
#define STATUS_REG_R 0x07   //000   0011    1
#define MEASURE_TEMP 0x03   //000   0001    1
#define MEASURE_HUMI 0x05   //000   0010    1
#define RESET        0x1e   //000   1111    0


/****************************************************************************************************************************************//**
 * @brief
 *  Declaration of the SHT1X_Sensirion driver-class.
 *
 *******************************************************************************************************************************************/

class SHT1X_Sensirion
{
private:

	union INPUT_VAR
	{
		uint8_t in[2];
		uint16_t out;
	};
	enum {TEMP, HUMI};

	void delayDriver();
	uint8_t s_write_byte( uint8_t value );
	uint8_t s_read_byte( uint8_t ack );

	void s_transstart();
	void s_connectionreset();
	uint8_t s_softreset();

	uint8_t s_read_statusreg( uint8_t *p_value, uint8_t *p_checksum );
	uint8_t s_write_statusreg( uint8_t *p_value );

	uint8_t s_measure( uint8_t *p_value, uint8_t *p_checksum, uint8_t mode );
	void  calc_sth11( float *p_humidity , float *p_temperature );
	float calc_dewpoint( float h, float t );

public:
	SHT1X_Sensirion() {};
	~SHT1X_Sensirion() {};

	void initializeInterface();
	void setLowPowerMode();
	uint8_t getMeasurement( float &humidity, float &temperature );
	uint8_t getMeasurement( uint16_t &humidity, uint16_t &temperature );
	float getDewpoint( );

};

#endif /* SHT1X_Sensirion */
