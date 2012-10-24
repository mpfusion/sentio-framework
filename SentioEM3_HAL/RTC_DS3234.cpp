/*
 * RTC_DS3234.cpp
 *
 *  Created on: Mar 13, 2011
 *      Author: Matthias Krämer
 */

#include "SystemConfig.h"
#include "RTC_DS3234.h"


/****************************************************************************************************************************************//**
 * @brief
 *  Constructor used create an instance of the RTC_DS3234 driver-class object
 *
 * @details
 * The required parameters of the object are initialized. No dynamic memory allocation is needed. The Parameter-values are specified in the
 * SystemConfig.h
 *
 *******************************************************************************************************************************************/

RTC_DS3234::RTC_DS3234()
{
	// Set the internal variables of the DS3234 module to the default values

	// Initialize the Parameters used to configure the USART in SPI-Master mode
	interfaceInit.baudrate  = _RTC_baudrate;
	interfaceInit.clockMode = _RTC_clockMode;
	interfaceInit.databits  = _RTC_databits;
	interfaceInit.enable 	= _RTC_enable;
	interfaceInit.refFreq   = _RTC_refFreq;
	interfaceInit.master 	= _RTC_master;
	interfaceInit.msbf  	= _RTC_msbf;

	// Define the USART-Number and the Location of the Module-Pins
	rtcLocation 			= _RTC_Location;
	rtcUSART 				= _RTC_USART;

	// Initialize Register-Settings
	statusRegisterValue     = 0b11001000;
	controlRegisterValue    = 0b00011100;
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method initializes GPIO-Pins used for serial communication and as digital-interface to connect to the RTC DS3234.
 *
 * @details
 *  Dependent on the SystemConfig.h Module Clock and GPIO-Pins are activated, moreover Configuration Registers of the EFM32 are set.
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::initializeInterface()
{
	// Configure the GPIO-Pins which are used to power the RTC (See schematic of the SentioEM3 version 32 or higher: VDD_RTC and VBAT)
	GPIO_PinModeSet( _RTC_VBAT_PIN, gpioModePushPull, 0 );
	GPIO_PinModeSet( _RTC_VDD_PIN,  gpioModePushPull, 0 );

	CMU_ClockEnable( cmuClock_USART2, true );

	// Initialize the USART-interface with the settings, which were specified in the Constructor of the RTC_DS3234-Module
	USART_InitSync( rtcUSART, &interfaceInit );
	USART_Enable( rtcUSART, usartEnable );

	// Set the Routing Register (Location and used Pins are specified)of the USART used for communication to the DS3234
	rtcUSART->ROUTE = USART_ROUTE_RXPEN|	// Enable the MISO-Pin
					  USART_ROUTE_TXPEN|	// Enable the MOSI-Pin
					  USART_ROUTE_CLKPEN|	// Enable the SPI-Clock Pin
					  USART_ROUTE_CSPEN|	// Enable the SPI-Modules Chip-Select pin
					  rtcLocation;

	// The Chip-Select is controlled by the USART module automatically, no Software Interaction is required
	rtcUSART->CTRL |= USART_CTRL_AUTOCS;


	// Configure the GPIO Pins used for SPI communication
	// SPI->MOSI (US2_TX, LOCATION 0)
	GPIO_PinModeSet( _RTC_SPI_MOSI_PIN, gpioModePushPull, 1 );

	// SPI->MISO (US2_RX, LOCATION 0)
	GPIO_PinModeSet( _RTC_SPI_MISO_PIN, gpioModeInputPull, 1 );

	// SPI->CLK (US2_CLK, LOCATION 0)
	GPIO_PinModeSet( _RTC_SPI_CLK_PIN, gpioModePushPull, 1 );

	// SPI->CS (US2_CS, LOCATION 0)
	GPIO_PinModeSet( _RTC_SPI_CS_PIN, gpioModePushPull, 1 );

	// Enable the VDD_RTC and VBAT_RTC
	GPIO_PinOutSet( _RTC_VBAT_PIN );
	GPIO_PinOutSet( _RTC_VDD_PIN );

	//Make sure the Oscillator is stable
	for( volatile uint32_t i=0; i<4000; i++);
}


/****************************************************************************************************************************************//**
 * @brief
 *  This method configures the interrupt EFM32-GPIO pin, which is connected to the Alarm-Interrupt/SQW_Pin of the DS3234. The Pin is set
 *  as input and the GPIO-Interrupt Group7 is configured to port A.
 *  The interrupt triggers always on the falling edge.
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::initializeMCU_Interrupt()
{
	// Configure the Pin PA7 as Input
	GPIO_PinModeSet( gpioPortA, 7, gpioModeInputPull, 1 );

	// Configure the Source of an External Interrupt 7 to be PortA and trigger the interrupt on the falling-edge
	GPIO_IntConfig( gpioPortA, 7, false, true, true );
}


/****************************************************************************************************************************************//**
 * @brief
 *  The 32kHz-output of the DS3234 is activated and the Low-Frequency clock of the EFM32-microcontroller is initialized.
 *
 * @param[in]
 *  batteryBacked: Shall the DS3234 be activated during RTC-in Low-Power mode
 *  enableLFA:     Enable the EFM32-Low frequency clock tree A
 *  enableLFB:     Enable the EFM32-Low frequency clock tree B
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::initialize32kHzClock( bool batteryBacked, bool enableLFA, bool enableLFB )
{
	// Configure the Pin PB 8 as Input, this pin can be used to drive the EFM32s internal LFXO path
	// (Set the LFXO as clock-source and set the "CMU_CTRL_LFXOMODE_DIGEXTCLK" Bit in the CMU's CTRL-Register)
	// More Information can be found In the EFM32 Reference Manual in the section CMU -> Register Description
	GPIO_PinModeSet( gpioPortB, 8, gpioModeInputPull, 1 );

	statusRegisterValue &= 0xB7;

	if(batteryBacked)
		statusRegisterValue |= 0x08;
	else
		statusRegisterValue |= 0x48;

	setSystemRegister( statusRegisterAddr, statusRegisterValue );

	CMU->CTRL |= ( (CMU->CTRL) | CMU_CTRL_LFXOMODE_DIGEXTCLK );

	CMU_ClockEnable( cmuClock_CORELE, true );

	if( enableLFA )
		CMU_ClockSelectSet( cmuClock_LFA, cmuSelect_LFXO );

	if( enableLFB )
		CMU_ClockSelectSet( cmuClock_LFB, cmuSelect_LFXO );
}


/****************************************************************************************************************************************//**
 * @brief
 *
 *
 * @details
 *
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::initializeInternalRTC( bool startRTC, bool enableISR, bool COMP0_isTopValue  )
{
	RTC_Init_TypeDef init;

	init.comp0Top = COMP0_isTopValue;
	init.debugRun = false;
	init.enable   = startRTC;

	RTC_Init( &init );

	if( enableISR )
	{
		NVIC_EnableIRQ( RTC_IRQn );
		RTC_IntEnable( RTC_IFC_COMP0 );
	}
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method sets the internal configuration registers of the DS3234 to standard-values.
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::setDefaultConfig()
{
	setSystemRegister(0x0E,0b00000100);
	setSystemRegister(0x0F,0b01111000);
}


/****************************************************************************************************************************************//**
 * @brief
 *  This driver-method deactivates the VDD_RTC-power line and forces the RTC in Low-Power mode.
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::setLowPowerMode()
{
	// When the VDD_RTC is disconnected the DS3234 reduces the current consumption.
	// Remember that BatteryBacked Operation needs to be enabled.
	GPIO_PinOutClear( gpioPortA,8 );
}


/****************************************************************************************************************************************//**
 * @brief
 *  The method will reset the DS3234 Alarm-Status bits. The Alarm/SQW-Pin of the RTC will go high after the reset is executed.
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::resetInterrupts()
{
	// The Interrupt Flags are reseted and the DS3234's SWQ/Interrupt-Pin is pulled high again.
	setSystemRegister( statusRegisterAddr, ( statusRegisterValue & 0xFC ) );
}


/****************************************************************************************************************************************//**
 * @brief
 *  This method is used to configure Alarm/SQW related system-registers/bits of the DS3234.
 *
 * @details
 * The following parameters are specified within the INTERRUPT_CONFIG
 *   bool    interruptControl;
 *   bool    enableAlarm1;
 *   bool    enableAlarm2;
 *	 bool    enableBatteryBackedSQW;
 *	 SQUAREW squareWaveFrequency;
 *
 * @param[in]
 *  INTERRUPT_CONFIG: struct that contains the new set of parameters, which will be written to the DS3234
 *
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::setInterruptConfig( INTERRUPT_CONFIG interruptConfig )
{
	uint8_t temp = 0;
	temp |= interruptConfig.enableBatteryBackedSQW;
	temp <<= 3;
	temp |= (uint8_t) interruptConfig.squareWaveFrequency;
	temp <<= 1;
	temp |= interruptConfig.interruptControl;
	temp <<= 1;
	temp |= interruptConfig.enableAlarm2;
	temp <<= 1;
	temp |= interruptConfig.enableAlarm1;

	temp |= ( controlRegisterValue & 0x80 );
	controlRegisterValue = temp;

	setSystemRegister(controlRegisterAddr, controlRegisterValue);
}


/****************************************************************************************************************************************//**
 * @brief
 *  This method is used to read Alarm/SQW related system-registers/bits of the DS3234.
 *
 * @details
 *  The following parameters are specified within the INTERRUPT_CONFIG
 *   bool    interruptControl;
 *   bool    enableAlarm1;
 *   bool    enableAlarm2;
 *	 bool    enableBatteryBackedSQW;
 *	 SQUAREW squareWaveFrequency;
 *
 * @param[out]
 *  INTERRUPT_CONFIG: pointer to struct, that contains the register-values, which are read out from the DS3234
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::getInterruptConfig( INTERRUPT_CONFIG *interruptConfig )
{
	uint8_t temp = getSystemRegister( controlRegisterAddr );

	interruptConfig->enableAlarm1 = ( temp & 0x01 );
	temp >>= 1;

	interruptConfig->enableAlarm2 = ( temp & 0x01 );
	temp >>= 1;

	interruptConfig->interruptControl = ( temp & 0x01 );
	temp >>= 1;

	interruptConfig->squareWaveFrequency = (SQUAREW) ( temp & 0x03 );
	temp >>= 2;

	interruptConfig->enableBatteryBackedSQW = ( temp & 0x01 );
}


/****************************************************************************************************************************************//**
 * @brief
 *  This method is used to read Alarm-Statusbits of the DS3234.
 *
 * @param[in]
 *  uint8_t: Number of the Alarm-Flag that will be read-out
 *
 * @return
 * 	bool: Status of the Alarm-Flag
 *******************************************************************************************************************************************/

bool RTC_DS3234::getAlarmStatusFlag( uint8_t alarm )
{
	if(alarm == 1)
	{
		return ( getSystemRegister( statusRegisterAddr ) & 0x01 );
	}

	return ( ( getSystemRegister( statusRegisterAddr )  >> 1 ) & 0x01 );
}


/****************************************************************************************************************************************//**
 * @brief
 *  This method is used to read Alarm-Matching Configuration-Register of the DS3234.
 *
 * @details
 *  The following Alarm-Matching Configurations are possible:
 *  alarmOncePerSecond                 = 0x0F,
 *	alarmMatchSeconds                  = 0x0E,
 *	alarmMatchMinutes_Seconds          = 0x0C,
 *	alarmMatchHour_Minutes_Seconds     = 0x08,
 *	alarmMatchDay_Hour_Minutes_Seconds = 0x00
 *
 * @return
 *  uint8_t: Status of the Matching-Configuration Registers
 *
 *
 *******************************************************************************************************************************************/

uint8_t RTC_DS3234::getAlarmMatchingConfig( uint8_t alarm )
{
	uint8_t RTC_AddressOffset = 0;
	uint8_t MSB_SettingTemp = 0;

	switch( alarm )
	{
	case 1:  RTC_AddressOffset = 0x07; break;
	case 2:  RTC_AddressOffset = 0x0B; break;
	default: alarm = 5;
	}

	for( uint8_t i = 0; i < ( 5 - alarm ) ; i++ )
	{

		if( ( getSystemRegister( RTC_AddressOffset + i )  & 0x80 ) )
		{
			MSB_SettingTemp |= ( 0x01 << i );

		}

	}

	return (ALARM_REG_SETTING) MSB_SettingTemp;
}


/****************************************************************************************************************************************//**
 * @brief
 *  This driver-method enables/disables battery-backed Temperature-Conversion.
 *
 * @param[in]
 *  bool: Status-Flag shall be set true/false
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::setBatteryBackedTempConv( bool status )
{
	setSystemRegister ( batteryBackedAddr, status );
}


/****************************************************************************************************************************************//**
 * @brief
 *  This driver-method reads the Status of the battery-backed Temperature-Conversion Register.
 *
 * @return
 *  bool: Status of the Configuration-Register
 *
 *******************************************************************************************************************************************/

bool RTC_DS3234::getBatteyBackedTempConv()
{
	return ( getSystemRegister( batteryBackedAddr ) );
}


/****************************************************************************************************************************************//**
 * @brief
 *  The method will force a Temperature-Sensor reading and an adaption of the 32kHz Oscillator-System of the DS3234.
 *
 *******************************************************************************************************************************************/

bool RTC_DS3234::forceManualTempConv()
{
	if( !( getSystemRegister( statusRegisterAddr ) & 0x03 ) )
	{
		setSystemRegister ( controlRegisterAddr, ( controlRegisterValue | 0x20 ) );
		return true;
	}
	else
		return false;
}


/****************************************************************************************************************************************//**
 * @brief
 *  This method is used to specify the functionality of the DS3234-32kHz Output.
 *
 * @details
 *  The following DS3234-32kHz Configurations are possible:
 *  bool	enable32kHz: enable/disable 32kHz-Output
 *  bool	enableBatteryBacked32kHz: enable/disable 32kHz-Output in lowpower/battery-backed Mode
 *
 * @param[in]
 *  OUTPUT_32KHZ: struct that contains the new set of parameters, which will be written to the DS3234
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::set32kHzOutputConfig( OUTPUT_32KHZ config32kHz )
{
	statusRegisterValue = ( statusRegisterValue & 0xB7 ) | ( ( ( (uint8_t) config32kHz.enableBatteryBacked32kHz) << 6 ) | ( ( (uint8_t) config32kHz.enable32kHz ) << 3 ) );

	setSystemRegister( statusRegisterAddr, statusRegisterValue );
}


/****************************************************************************************************************************************//**
 * @brief
 *  This method is used to read 32kHz-Output Configuration-Register of the DS3234.
 *
 * @details
 *  The following parameters are specified within the 32kHz-Output:
 *  bool	enable32kHz: enable/disable 32kHz-Output
 *  bool	enableBatteryBacked32kHz: enable/disable 32kHz-Output in lowpower/battery-backed Mode
 *
 * @param[in]
 *  OUTPUT_32KHZ: pointer to struct, that will contain the register-values, which are read out from the DS3234
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::get32kHzOutputConfig( OUTPUT_32KHZ *config32kHz )
{
	uint8_t temp = getSystemRegister( controlRegisterAddr );
	temp >>= 3;
	config32kHz->enable32kHz = ( temp & 0x01 );
	temp >>= 3;
	config32kHz->enableBatteryBacked32kHz = ( temp & 0x01 );
}


/****************************************************************************************************************************************//**
 * @brief
 *  This method is specify the rate at which the Temperature-Sensor is read and an adaption of the 32kHz Oscillator-System of the
 *  DS3234is done.
 *
 * @details
 *  The following configurations are possible:
 *  seconds_64
 *	seconds_128
 *	seconds_256
 * 	seconds_512
 *
 * @param[in]
 *  TEMP_CONV: Temperature-Conversion Rate
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::setTempConvRate( TEMP_CONV tempConfig )
{
	statusRegisterValue = ( ( statusRegisterValue & 0xCF ) | (uint8_t) tempConfig );
	setSystemRegister( statusRegisterAddr, statusRegisterValue );
}


/****************************************************************************************************************************************//**
 * @brief
 *  This method is reads the rate at which the Temperature-Sensor is read and an adaption of the 32kHz Oscillator-System of the
 *  DS3234is done.
 *
 * @details
 *  The following configurations are possible:
 *  seconds_64
 *	seconds_128
 *	seconds_256
 * 	seconds_512
 *
 * @param[out]
 *  TEMP_CONV: Temperature-Conversion Rate
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::getTempConvRate( TEMP_CONV *tempConfig )
{
	*tempConfig = (TEMP_CONV) ( getSystemRegister(statusRegisterAddr) & 0xCF );
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method reads a Status-Flag that indicates a ongoing Temperature Conversion
 *
 * @return
 *  bool: Status of the Flag
 *
 *******************************************************************************************************************************************/

bool RTC_DS3234::getTempConvFlag()
{
	return ( getSystemRegister(controlRegisterAddr) >> 5 ) & 0x01;
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method reads a Status-Flag that indicates the device is busy executing TCXO functions.
 *
 * @return
 *  bool Status of the Flag
 *
 *******************************************************************************************************************************************/

bool RTC_DS3234::getBusyFlag()
{
	return ( getSystemRegister(statusRegisterAddr) >> 2 ) & 0x01;
}


void RTC_DS3234::setCrystalAgingOffset( uint8_t agingConfig )
{
	setSystemRegister ( agingCoefficientAddr, agingConfig );
}


uint8_t RTC_DS3234::getCrystalAgingOffset()
{
	return getSystemRegister ( agingCoefficientAddr );
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method configures the Oscillator-Enable Flag
 *
 * @details
 *  When set to logic 0, the oscillator is started. When set to logic 1, the oscillator is stopped when the DS3234 switches
 *  to battery power.
 *
 * @param[in]
 *  bool Status of the Oscillator-Enable Flag
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::setOscillatorEnableFlag( bool flagStatus )
{
	controlRegisterValue = ( controlRegisterValue & 0x7F ) | ( ( (uint8_t) flagStatus ) << 7 );
	setSystemRegister( controlRegisterAddr, controlRegisterValue );
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method reads the Oscillator-Enable Flag
 *
 * @details
 *  When set to logic 0, the oscillator is started. When set to logic 1, the oscillator is stopped when the DS3234 switches
 *  to battery power. (DS3234 datasheet)
 *
 * @return
 *  bool Status of the Oscillator-Enable Flag
 *
 *******************************************************************************************************************************************/

bool RTC_DS3234::getOscillatorEnableFlag()
{
	return ( getSystemRegister( controlRegisterAddr ) >> 7 ) & 0x01;
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method reads the Oscillator-Stop Flag
 *
 * @details
 * A logic 1 in this bit indicates that the oscillator either is stopped or was stopped for some period and may be used to judge the validity
 * of the timekeeping data. This bit is set to logic 1 any  time  that  the  oscillator  stops. (DS3234 datasheet)
 *
 * @return
 *  bool Status of the Oscillator-Stop Flag
 *
 *******************************************************************************************************************************************/

bool RTC_DS3234::getOscillatorStopFlag()
{
	return ( getSystemRegister( statusRegisterAddr ) >> 7 ) & 0x01;
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method resets (set to false) the Oscillator-Stop Flag
 *
 * @details
 * A logic 1 in this bit indicates that the oscillator either is stopped or was stopped for some period and may be used to judge the validity
 * of the timekeeping data. This bit is set to logic 1 any  time  that  the  oscillator  stops. (DS3234 datasheet)
 *
 * @return
 *  bool Status of the Oscillator-Stop Flag, before the reset
 *
 *******************************************************************************************************************************************/

bool RTC_DS3234::resetOscillatorStopFlag()
{
	bool temp =  ( ( getSystemRegister( statusRegisterAddr ) >> 7 ) & 0x01 );
	setSystemRegister( statusRegisterAddr, ( statusRegisterValue & 0x7F ) );

	return temp;
}


/****************************************************************************************************************************************//**
 * @brief
 *  This method is used to configure the System-Time, the time bases of the DS3234.
 *
 * @details
 * The following parameters are specified within the SYSTEMTIME-structure
		uint8_t seconds
		uint8_t minutes
		uint8_t hours
		uint8_t day
		uint8_t date
		uint8_t month
		uint8_t year
 *
 * @param[in]
 *  SYSTEMTIME struct that contains the new values of the parameters, which will be written to the DS3234
 *
 * @return
 *  bool: Valid/Invalid Alarm-Number, Invalid Time-Register Values
 *
 *******************************************************************************************************************************************/

bool RTC_DS3234::setBaseTime( time systemTime )
{
	uint8_t *arrayAccess = systemTime.getArrayAccess();

	// Perform Write-Access to the DS3234-Base time Registers, which have the Address (0x00 to 0x06)
	for( uint8_t i = 0; i<7; i++ )
	{
		// The MS-Bit of the Register 0x05 is always high, ( it contains the information, if we are in the 20 or 21th century )
		// As long as our time-machine is not ordered jet :-) it is quite useless to make this bit dynamic.
		if( i == 5 )
			setSystemRegister( i, ( ( calculateTimeReg( arrayAccess[i] ) ) | 0x80 ) );
		else
			setSystemRegister( i, ( calculateTimeReg( arrayAccess[i] ) ) );
	}

	// All user-defined input was valid and the write Access to the SPI-Interface is completed.
	return true;
}


/****************************************************************************************************************************************//**
 * @brief
 *  This driver-method is used to read the System-Time, the Time-Base of the DS3234.
 *
 * @details
 * The following parameters are specified within the SYSTEMTIME-structure
		uint8_t seconds
		uint8_t minutes
		uint8_t hours
		uint8_t day
		uint8_t date
		uint8_t month
		uint8_t year
 *
 * @param[out]
 *  SYSTEMTIME: pointer to struct that contains the values of the parameters, which will be read from the DS3234
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::getBaseTime( time &systemTime )
{
	uint8_t *arrayAccess = systemTime.getArrayAccess();


	for( uint8_t i = 0; i < 7; i++ )
	{
		arrayAccess[i] = calculateInverseTimeReg( ( getSystemRegister( i ) & 0x7F ) );
	}
}


/****************************************************************************************************************************************//**
 * @brief
 *  This driver-method is used to read the current Base-Time from the DS3234 and adds the Alarm-Period. Then the result is written to
 *  back as Alarm-Time. So a delay Period can be specified.
 *
 * @details
 * The following parameters are specified within the ALARMTIME-structure
 *	uint8_t seconds
 *	uint8_t minutes
 *	uint8_t hours
 *	uint8_t day
 *	ALARM_REG_SETTING MSB_Setting
 *	uint8_t alarmNumber
 *
 * @param[out]
 *  ALARMTIME: struct that contains the new set of parameters, which will be written to the DS3234
 *
 * @return
 *  bool: Invalid Alarm-Number, Invalid Time-Register Values or Time-Span to long
 *
 *******************************************************************************************************************************************/

bool RTC_DS3234::setAlarmPeriod( time alarmPeriod, ALARM_NUMBER alarm, ALARM_REG_SETTING MSB_Setting )
{
	time baseT;

	uint8_t RTC_AddressOffset;
	uint8_t temp;

	switch( alarm )
	{
	case 1:  RTC_AddressOffset = 0x07; break;
	case 2:  RTC_AddressOffset = 0x0B; break;
	default: return false;
	}

	getBaseTime(baseT);
	baseT = baseT + alarmPeriod;

	uint8_t *arrayAccess = baseT.getArrayAccess();


	temp = (uint8_t) MSB_Setting;

	// Update the control registers of the DS3234 RTC, Alarm1: four registers beginning form the Offset are set,
	// while Alarm 2 requires just the configuration of three control registers
	for( uint8_t i = 0; i < ( 5 - alarm ); i++ )
	{
		// The MASK of this Registers selects the different Alarm/Matching Options
		if( temp & 0x01 )
		{
			setSystemRegister( ( i + RTC_AddressOffset), calculateTimeReg( arrayAccess[i] ) | 0x80 );
		}
		else
			setSystemRegister( ( i + RTC_AddressOffset), calculateTimeReg( arrayAccess[i] ) );

		// Prepare the Status register value
		temp >>= 1;
	}

	// Configuration done
	return true;
}



/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method is used to configure the Alarm-Time of the DS3234.
 *
 * @details
 * The following parameters are specified within the ALARMTIME-structure
 *	uint8_t seconds
 *	uint8_t minutes
 *	uint8_t hours
 *	uint8_t day
 *	ALARM_REG_SETTING MSB_Setting
 *	uint8_t alarmNumber
 *
 * @param[out]
 *  ALARMTIME: struct that contains the new set of parameters, which will be written to the DS3234
 *
 * @return
 *  bool: Valid/Invalid Alarm-Number, Invalid Time-Register Values
 *
 *******************************************************************************************************************************************/

bool RTC_DS3234::setAlarmTime( time alarmTime, ALARM_NUMBER alarm, ALARM_REG_SETTING MSB_Setting )
{
	uint8_t *arrayAccess = alarmTime.getArrayAccess();

	uint8_t RTC_AddressOffset;
	uint8_t temp = (uint8_t) MSB_Setting;

	switch( alarm )
	{
	case 1:  RTC_AddressOffset = 0x07; break;
	case 2:  RTC_AddressOffset = 0x0B; break;
	default: return false;
	}

	// Update the control registers of the DS3234 RTC, Alarm1: four registers beginning form the Offset are set,
	// while Alarm 2 requires just the configuration of three control registers
	for( uint8_t i = 0; i < ( 5 - alarm ); i++ )
	{
		// The MASK of this Registers selects the different Alarm/Matching Options
		if( temp & 0x01 )
		{
			setSystemRegister( ( i + RTC_AddressOffset), ( calculateTimeReg( arrayAccess[i] ) ) | 0x80 );
		}
		else
			setSystemRegister( ( i + RTC_AddressOffset), calculateTimeReg( arrayAccess[i] ) );

		// Prepare the Status register value
		temp >>= 1;
	}

		// Configuration-Parameters are valid
		return true;
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method is used to read the alarm-time configured for Alarm 1/2 of the DS3234.
 *
 * @details
 * The following parameters are specified within the ALARMTIME-structure
 *	uint8_t seconds
 *	uint8_t minutes
 *	uint8_t hours
 *	uint8_t day
 *	ALARM_REG_SETTING MSB_Setting
 *	uint8_t alarmNumber
 *
 * @param[out]
 *  ALARMTIME: pointer to struct that contains the values of the parameters, which will be read from the DS3234
 *
 * @return
 *  bool: Valid/Invalid Alarm-Number
 *
 *******************************************************************************************************************************************/

bool RTC_DS3234::getAlarmTime( time &alarmTime, ALARM_NUMBER alarm, ALARM_REG_SETTING &MSB_Setting )
{
	uint8_t RTC_AddressOffset;
	uint8_t temp;
	uint8_t MSB_SettingTemp = 0;

	uint8_t *arrayAccess = alarmTime.getArrayAccess();

	switch( alarm )
	{
	case 1:  RTC_AddressOffset = 0x07; break;
	case 2:  RTC_AddressOffset = 0x0B; break;
	default: return false;
	}

	for( uint8_t i = 0; i < ( 5 - alarm ) ; i++ )
	{
		temp = getSystemRegister( RTC_AddressOffset + i );

		if( ( temp  & 0x80) )
		{
			MSB_SettingTemp |= ( 0x01 << i );

		}

		arrayAccess[i] = calculateInverseTimeReg( temp & 0x7F );
	}

	MSB_Setting = (ALARM_REG_SETTING) MSB_SettingTemp;

	return true;
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method reads the leading-byte of the temperature-value of the DS3234 internal sensor.
 *
 * @param[out]
 *  uint8_t: pointer to a unit8_t, that will hold the temperature-value
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::getTemperature( uint8_t *temperature )
{
	*temperature = getSystemRegister( 0x11 );
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method reads the temperature-value of the DS3234 internal sensor.
 *
 * @param[out]
 *  float: pointer to a float, that will hold the temperature-value
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::getTemperature( float *temperature )
{
	uint8_t RegisterValue;
	int32_t tempTemp;

	RegisterValue = getSystemRegister( 0x11 );

	tempTemp = (int32_t)(RegisterValue & 0x7F);

	if(RegisterValue & 0x80)
		tempTemp *= (-1);

	*temperature = tempTemp;

	RegisterValue = getSystemRegister( 0x12 );

	if( RegisterValue & 0x80 )
		*temperature += 0.5;
	if( RegisterValue & 0x40 )
		*temperature += 0.25;
}


/****************************************************************************************************************************************//**
 * @brief
 *  The method writes a single byte to a specific address of the DS3234s internal SRAM
 *
 * @param[in]
 *  uint8_t: SRAM-Address which will be written
 *
 * @param[in]
 *  uint8_t: Data-Byte
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::writeToSRAM( uint8_t address, uint8_t data )
{
	setSystemRegister(SRAM_Addr, address);
	setSystemRegister(SRAM_Data, data);
}


/****************************************************************************************************************************************//**
 * @brief
 *  The method reads a single byte to a specific address of the DS3234s internal SRAM
 *
 * @param[in]
 *  uint8_t: SRAM-Address which will be written
 *
 * @param[out]
 *  uint8_t: Data-Byte
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::readFromSRAM( uint8_t address, uint8_t *data)
{
	setSystemRegister(SRAM_Addr, address);
	*data = getSystemRegister(SRAM_Data);
}


/****************************************************************************************************************************************//**
 * @brief
 *  The method enables/disables Temperature Conversion when the DS3234 is in low-power/battery-backed mode
 *
 * @param[in]
 *  bool enable/disable Conversion
 *
 *******************************************************************************************************************************************/

void RTC_DS3234::batteryBackedTemperatureConv( bool status )
{
	// Perform Write-Access to the DS3234-Configuration Register
	setSystemRegister( batteryBackedAddr,(uint8_t) status );
}


/****************************************************************************************************************************************//**
 * @brief
 *  The method calculates the Register-Value of a DS3234 system-register, that holds timing-values, like sec/min/hour/day
 *
 *  @param[in]
 *   uint8_t: Decimal-Representation of the sec/min/hour/day
 *
 *  @return
 *   uint8_t: Register-Value
 *
 * @note
 *  !!! Internally used !!!
 *
 *******************************************************************************************************************************************/

uint8_t RTC_DS3234::calculateTimeReg( uint8_t time )
{
	// The user-defined time and date information is transfered form a Binary/Hex-Format into a Decimal
	// Data-Format which is used by the DS3234 for representation of Time and Date.
	uint8_t temp = time % 10;
	time = ( temp | ( ( time / 10 ) << 4 ) );

	return time;
}


/****************************************************************************************************************************************//**
 * @brief
 *  The method calculates the decimal representation of a Register-Value read from an DS3234 system-register
 *
 *  @param[in]
 *   uint8_t: Register-Value
 *
 *  @return
 *   uint8_t: Decimal-Representation of the sec/min/hour/day
 *
 * @note
 *  !!! Internally used !!!
 *
 *******************************************************************************************************************************************/

uint8_t RTC_DS3234::calculateInverseTimeReg( uint8_t timeReg )
{
	uint8_t temp = ( timeReg & 0x0F );

	timeReg >>= 4;
	return ( temp + ( timeReg * 10) );
}


/****************************************************************************************************************************************//**
 * @brief
 *  The method reads a DS3234 System-Register
 *
 *  @param[in]
 *   uint8_t: Register-Address
 *
 *  @return
 *   uint8_t: Data read from the Register-Address
 *
 * @note
 *  !!! Internally used !!!
 *
 *******************************************************************************************************************************************/

inline uint8_t RTC_DS3234::getSystemRegister( uint8_t address )
{
	if( !( GPIO_PinInGet( _RTC_VDD_PIN ) ) )
	{
		GPIO_PinOutSet( _RTC_VDD_PIN );
		for( volatile uint32_t i = 0; i < 4000; i++ );
	}

	USART_TxDouble( rtcUSART,( address << 8 ) );
	while( !GPIO_PinInGet( _RTC_SPI_CS_PIN ) );


	return (uint8_t)USART_RxDouble( rtcUSART );
}


/****************************************************************************************************************************************//**
 * @brief
 *  The method sets a DS3234 System-Register.
 *
 *  @param[in]
 *   uint8_t: Register-Address
 *
 *  @param[in]
 *   uint8_t: Configuration-Data
 *
 * @note
 *  !!! Internally used !!!
 *******************************************************************************************************************************************/

inline void RTC_DS3234::setSystemRegister( uint8_t address, uint8_t data )
{
	// Make sure the RTC supply is Enabled, when the SPI-Interface is accessed.
	// In case the RTC-VDD
	if( !( GPIO_PinInGet( _RTC_VDD_PIN ) ) )
	{
		GPIO_PinOutSet( _RTC_VDD_PIN );
		for( volatile uint32_t i = 0; i < 4000; i++ );
	}

	// Transfer the 16-Bit Address and Data-Information to the SPI-Interface
	// Set the Shift the Address-Byte including the Read/Write-Information into the higher 8-Bit of an uint16_t variable
	// and then combine the result with the data to be written to the DS3234.
	USART_TxDouble( rtcUSART, ( ( address | 0x80 ) << 8 ) | data );

	// Wait for the Chip select to go high again, to avoid interferences between two Write-Commands to the DS3234
	// Make the pin selection dynamic to allow different Locations of the USART interface
	while( !GPIO_PinInGet( _RTC_SPI_CS_PIN ) );



	USART_RxDouble( rtcUSART );
}
