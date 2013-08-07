/*
 * RTC.h
 *
 *  Created on: Mar 13, 2011
 *      Author: Matthias Krämer
 */

#ifndef RTC_DS3234_H_
#define RTC_DS3234_H_

#include "time.h"

#include "em_gpio.h"
#include "em_usart.h"
#include "em_cmu.h"
#include "em_rtc.h"


/****************************************************************************************************************************************//**
 * @brief
 *  Make sure the Oscillator is stable
 *
 *******************************************************************************************************************************************/

#define stableWait 4000


/****************************************************************************************************************************************//**
 * @brief
 *  Definitions of DS3234 System Register Addresses
 *
 *******************************************************************************************************************************************/

#define controlRegisterAddr  0x0E
#define statusRegisterAddr   0x0F
#define agingCoefficientAddr 0x10
#define batteryBackedAddr    0x13
#define SRAM_Addr            0x18
#define SRAM_Data            0x19


/****************************************************************************************************************************************//**
 * @brief
 *  Definition of possible Alarm-MatchingSettings
 *
 *******************************************************************************************************************************************/

typedef enum
{
	alarmOncePerSecond                 = 0x0F,
	alarmMatchSeconds                  = 0x0E,
	alarmMatchMinutes_Seconds          = 0x0C,
	alarmMatchHour_Minutes_Seconds     = 0x08
} ALARM_REG_SETTING;


typedef enum
{
	alarm1 = 0x01,
	alarm2 = 0x02
} ALARM_NUMBER;


/****************************************************************************************************************************************//**
 * @brief
 *  Definition of possible square-wave frequencies. When the Alarm is deactivated,the Pin can be used to output an low-frequency square-wave
 *
 *******************************************************************************************************************************************/

typedef enum
{
	square_1Hz    = 0x00,
	square_1024Hz = 0x01,
	square_4096Hz = 0x02,
	square_8192Hz = 0x03
} SQUAREW;


/****************************************************************************************************************************************//**
 * @brief
 *  Definition of possible sampling-frequencies of the internal temperature sensor.
 *
 *******************************************************************************************************************************************/

typedef enum
{
	seconds_64     = ( 0x00 << 4 ),
	seconds_128    = ( 0x01 << 4 ),
	seconds_256    = ( 0x02 << 4 ),
	seconds_512    = ( 0x03 << 4 )
} TEMP_CONV;


/****************************************************************************************************************************************//**
 * @brief
 *  Declaration of interrupt configuration structure
 *
 *******************************************************************************************************************************************/

struct INTERRUPT_CONFIG
{
	bool    interruptControl;
	bool    enableAlarm1;
	bool    enableAlarm2;
	bool    enableBatteryBackedSQW;
	SQUAREW squareWaveFrequency;
};


/****************************************************************************************************************************************//**
 * @brief
 *  Declaration of the configuration structure, that defines the 32kHz output
 *
 *******************************************************************************************************************************************/

struct OUTPUT_32KHZ
{
	bool    enable32kHz;
	bool    enableBatteryBacked32kHz;
};



/****************************************************************************************************************************************//**
 * @brief
 *  Declaration of the RTC-DS3234 driver-class.
 *
 *******************************************************************************************************************************************/
class RTC_DS3234
{
private:
	uint32_t                rtcLocation;
	USART_InitSync_TypeDef  interfaceInit;
	USART_TypeDef           *rtcUSART  ;

	uint8_t                 statusRegisterValue;
	uint8_t                 controlRegisterValue;

	uint8_t calculateTimeReg( uint8_t time );
	uint8_t calculateInverseTimeReg( uint8_t timeReg );


public:
	RTC_DS3234();
	~RTC_DS3234() {}

	uint8_t getSystemRegister( uint8_t address );
	void    setSystemRegister( uint8_t address, uint8_t data );

	void initializeInterface();
	void initializeMCU_Interrupt();
	void initialize32kHzClock( bool batteryBacked, bool enableLFA, bool enableLFB );
	void initializeInternalRTC( bool startRTC, bool enableISR, bool COMP0_isTopValue );

	void setLowPowerMode();
	void resetInterrupts();

	void setInterruptConfig( INTERRUPT_CONFIG interruptConfig );
	void getInterruptConfig( INTERRUPT_CONFIG *interruptConfig );

	bool    getAlarmStatusFlag( uint8_t alarm );
	uint8_t getAlarmMatchingConfig( uint8_t alarm );

	void setBatteryBackedTempConv( bool status );
	bool getBatteyBackedTempConv();

	bool forceManualTempConv();

	void set32kHzOutputConfig( OUTPUT_32KHZ config32kHz );
	void get32kHzOutputConfig( OUTPUT_32KHZ *config32kHz );

	void setTempConvRate( TEMP_CONV tempConfig );
	void getTempConvRate( TEMP_CONV *tempConfig );
	bool getTempConvFlag();
	bool getBusyFlag();


	void    setCrystalAgingOffset( uint8_t agingConfig );
	uint8_t getCrystalAgingOffset();

	void setOscillatorEnableFlag( bool flagStatus );
	bool getOscillatorEnableFlag();

	bool getOscillatorStopFlag();
	bool resetOscillatorStopFlag();

	bool setBaseTime( time systemTime );
	void getBaseTime( time &systemTime );
	bool setAlarmPeriod( time alarmPeriod, ALARM_NUMBER alarm, ALARM_REG_SETTING MSB_Setting );
	//bool setAlarmPeriod( time systemTime, time alarmPeriod, ALARM_NUMBER alarm, ALARM_REG_SETTING MSB_Setting );

	bool setAlarmTime( time alarmTime, ALARM_NUMBER alarm, ALARM_REG_SETTING MSB_Setting );
	bool getAlarmTime( time &alarmTime, ALARM_NUMBER alarm, ALARM_REG_SETTING &MSB_Setting );

	void setDefaultConfig();

	void getTemperature( float *temperature );
	void getTemperature( uint8_t *temperature );

	void writeToSRAM( uint8_t address, uint8_t data );
	void readFromSRAM( uint8_t address, uint8_t *data );


	void setControlRegister( uint8_t control );
	void setStatusRegister( uint8_t status );
	void batteryBackedTemperatureConv( bool status );

};

#endif /* RTC_DS3234_H_ */
