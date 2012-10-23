/*
 * SHT1X_Humid_Temp.cpp
 *
 *  Created on: Apr 11, 2011
 *      Author: matthias
 */
#include <math.h>
#include "SHT1X_Sensirion.h"



/****************************************************************************************************************************************//**
 * @brief
 *  The EFM32 IO-Pins which are used to connect the Sensor. Clock(SCK) and Data Pin are defined in SystemConfig.h
 *
 *******************************************************************************************************************************************/

void SHT1X_Sensirion::initializeInterface()
{
	GPIO_PinModeSet( SHT_DataPin, gpioModeWiredAndPullUpFilter, 0);
	GPIO_PinModeSet( SHT_SCK_Pin, gpioModePushPull, 0);
}

void SHT1X_Sensirion::setLowPowerMode()
{
	GPIO_PinModeSet( SHT_DataPin, gpioModeDisabled, 0);
	GPIO_PinModeSet( SHT_SCK_Pin, gpioModeDisabled, 0);
}

/****************************************************************************************************************************************//**
 * @brief
 *  Dependent on the clock-frequency, defined in SystemConfig.h,CPU waits a certain amount of clock-cycles. The delay is in the range of us.
 *
 *******************************************************************************************************************************************/

volatile void SHT1X_Sensirion::delayDriver()
{
	volatile uint8_t test = 0;
#if SentioEM_CPU_Clock_MHZ > 28
	test++;
	test++;
	test++;
	test++;
	test++;
#endif

#if SentioEM_CPU_Clock_MHZ > 14
	test++;
#endif

#if SentioEM_CPU_Clock_MHZ > 2
	test++;
#endif
}


/****************************************************************************************************************************************//**
 * @brief
 *  writes a byte on the Sensibus and checks the acknowledge
 *
 *******************************************************************************************************************************************/

uint8_t SHT1X_Sensirion::s_write_byte( uint8_t value )
{
	uint8_t i, error = 0;

	for ( i=0x80; i > 0; i /= 2 )           //shift bit for masking
	{
		if ( i & value )
			GPIO_PinOutSet( SHT_DataPin );  //masking value with i , write to SENSI-BUS
		else
			GPIO_PinOutClear( SHT_DataPin );

		delayDriver();                   //observe setup time
		GPIO_PinOutSet( SHT_SCK_Pin );   //clk for SENSI-BUS
		delayDriver(); delayDriver();    //pulswith approx. 5 us
		GPIO_PinOutClear( SHT_SCK_Pin );
		delayDriver();                        			//observe hold time
	}

	GPIO_PinOutSet( SHT_DataPin );        //release DATA-line

	delayDriver();                          //observe setup time
	GPIO_PinOutSet( SHT_SCK_Pin );                            //clk #9 for ack
	error = GPIO_PinInGet( SHT_DataPin );                       //check ack (DATA will be pulled down by SHT11)
	GPIO_PinOutClear( SHT_SCK_Pin );

	return error;                     //error=1 in case of no acknowledge
}


/****************************************************************************************************************************************//**
 * @brief
 *  reads a byte form the Sensibus and gives an acknowledge in case of "ack=1"
 *
 *******************************************************************************************************************************************/

uint8_t SHT1X_Sensirion::s_read_byte( uint8_t ack )
{
	uint8_t i, val = 0;

	GPIO_PinOutSet( SHT_DataPin );          //release DATA-line

	for ( i = 0x80; i > 0; i /= 2 )         //shift bit for masking
	{
		GPIO_PinOutSet( SHT_SCK_Pin );      //clk for SENSI-BUS
		 delayDriver();
		if ( GPIO_PinInGet( SHT_DataPin ) ) //read bit
			val = ( val | i );

		GPIO_PinOutClear( SHT_SCK_Pin );

		delayDriver();

	}

	//DATA = !ack;                    //in case of "ack==1" pull down DATA-Line

	if(ack)
		GPIO_PinOutClear( SHT_DataPin );

	delayDriver();                   //observe setup time

	GPIO_PinOutSet( SHT_SCK_Pin );   //clk #9 for ack

	delayDriver(); delayDriver();    //pulswith approx. 5 us

	GPIO_PinOutClear( SHT_SCK_Pin );

	delayDriver();                   //observe hold time

	GPIO_PinOutSet( SHT_DataPin );   //release DATA-line

	return val;
}




/****************************************************************************************************************************************//**
 * @brief
 *  Internal driver function, used to generate transmission start pattern
 *
 * @details
 * a transmission start:
 *       _____         ________
 * DATA:      |_______|
 *           ___     ___
 * SCK : ___|   |___|   |______
 *
 *******************************************************************************************************************************************/
void SHT1X_Sensirion::s_transstart()
{
	// Initial state
	GPIO_PinOutSet( SHT_DataPin );
	GPIO_PinOutClear( SHT_SCK_Pin );
	delayDriver();

	// Generate the transmission start pattern
	GPIO_PinOutSet( SHT_SCK_Pin );
	delayDriver();
	GPIO_PinOutClear( SHT_DataPin );
	delayDriver();
	GPIO_PinOutClear( SHT_SCK_Pin );
	delayDriver();delayDriver();delayDriver();delayDriver();
	GPIO_PinOutSet( SHT_SCK_Pin );
	delayDriver();
	GPIO_PinOutSet( SHT_DataPin );
	delayDriver();
	GPIO_PinOutClear( SHT_SCK_Pin );
}


/****************************************************************************************************************************************//**
 * @brief
 *   Internal driver function, used to generate communication reset pattern
 *
 * @details
 * communication reset: DATA-line=1 and at least 9 SCK cycles followed by transmission-start, call transstart
 *       _____________________________________________________         ________
 * DATA:                                                      |_______|
 *          _    _    _    _    _    _    _    _    _        ___     ___
 * SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
 *
 *******************************************************************************************************************************************/

void SHT1X_Sensirion::s_connectionreset()
{
	//Initial state
	GPIO_PinOutSet( SHT_DataPin );

	GPIO_PinOutClear( SHT_SCK_Pin );

	for( volatile uint8_t i = 0; i < 18; i++)
	{
		GPIO_PinOutToggle( SHT_SCK_Pin );
		delayDriver();
	}

	delayDriver();

	s_transstart();
}


/****************************************************************************************************************************************//**
 * @brief
 *  resets the sensor by a software-reset
 *
 *******************************************************************************************************************************************/

uint8_t SHT1X_Sensirion::s_softreset()
{
	uint8_t error = 0;

	s_connectionreset();              //reset communication

	error += s_write_byte( RESET );   //send RESET-command to sensor

	return error;                     //error=1 in case of no response form the sensor
}


/****************************************************************************************************************************************//**
 * @brief
 *  reads the status register with checksum (8-bit) read-out
 *
 *******************************************************************************************************************************************/

uint8_t SHT1X_Sensirion::s_read_statusreg( uint8_t *p_value, uint8_t *p_checksum )
{
	uint8_t error=0;

	s_transstart();                       //transmission start
	error = s_write_byte( STATUS_REG_R ); //send command to sensor

	*p_value = s_read_byte( ACK );        //read status register (8-bit)
	*p_checksum = s_read_byte( noACK );   //read checksum (8-bit)

	return error;                         //error=1 in case of no response form the sensor
}


/****************************************************************************************************************************************//**
 * @brief
 *  writes the status register with checksum (8-bit) read-out
 *
 *******************************************************************************************************************************************/

uint8_t SHT1X_Sensirion::s_write_statusreg( uint8_t *p_value )
{
  uint8_t error = 0;

  s_transstart();                       //transmission start

  error += s_write_byte( STATUS_REG_W );//send command to sensor

  error += s_write_byte( *p_value );    //send value of status register

  return error;                         //error>=1 in case of no response form the sensor
}


/****************************************************************************************************************************************//**
 * @brief
 * makes a measurement (humidity/temperature) with checksum read-out
 *
 *******************************************************************************************************************************************/

uint8_t SHT1X_Sensirion::s_measure(uint8_t *p_value, uint8_t *p_checksum, uint8_t mode)
{
	uint8_t error=0;
	uint32_t i;

	s_transstart(); //transmission start

	switch( mode )
	{              //send command to sensor
	case TEMP:
		error += s_write_byte( MEASURE_TEMP );
		break;

    case HUMI:
    	error += s_write_byte( MEASURE_HUMI );
    	break;

    default:
    	break;
	}

	for (i=0; i < 15000000; i++)				//wait until sensor has finished the measurement
	{										// or timeout is reached
		if( !( GPIO_PinInGet( SHT_DataPin ) ) )
			break;
	}

	if(GPIO_PinInGet( SHT_DataPin ) )
		error += 1;

	*(p_value +1) = s_read_byte(ACK);  //read the first byte (MSB)
	*(p_value)    = s_read_byte(ACK);  //read the second byte (LSB)
	*p_checksum   = s_read_byte(noACK);//read checksum

	return error;
}


//----------------------------------------------------------------------------------
// calculates temperature [�C] and humidity [%RH]
// input :  humi [Ticks] (12 bit)
//          temp [Ticks] (14 bit)
// output:  humi [%RH]
//          temp [�C]
//----------------------------------------------------------------------------------

/****************************************************************************************************************************************//**
 * @brief
 *  Basic System-Variables are set-up, which do not need to be changed during Run-Time
 *
 * @details
 * The Parameterization of the Serial-Interface and the Selection of the Serial-Interface as well as the selection of Interface is done here!
 *
 * @note
 * The User should not change the Initialization-Values for System-Variables which are not related to the Hardware-Interfaces
 *******************************************************************************************************************************************/

void SHT1X_Sensirion::calc_sth11(float *p_humidity ,float *p_temperature)
{

	const float C1 = -2.0468;           // for 12 Bit RH
	const float C2 = +0.0367;           // for 12 Bit RH
	const float C3 = -0.0000015955;     // for 12 Bit RH
	const float T1 = +0.01;             // for 12 Bit RH
	const float T2 = +0.00008;          // for 12 Bit RH

	float rh = *p_humidity;             // rh:      Humidity [Ticks] 12 Bit
	float t = *p_temperature;           // t:       Temperature [Ticks] 14 Bit
	float rh_lin;                       // rh_lin:  Humidity linear
	float rh_true;                      // rh_true: Temperature compensated humidity
	float t_C;                          // t_C   :  Temperature [�C]

	t_C=t*0.01 - 40.3;     				//calc. temperature[�C]from 14 bit temp.ticks
	rh_lin=C3*rh*rh + C2*rh + C1;     	//calc. humidity from ticks to [%RH]
	rh_true=(t_C-25)*(T1+T2*rh)+rh_lin; //calc. temperature compensated humidity [%RH]
	if(rh_true>100)rh_true=100;       	//cut if the value is outside of
	if(rh_true<0.1)rh_true=0.1;       	//the physical possible range

	*p_temperature=t_C;      //return temperature [�C]
	*p_humidity=rh_true;     //return humidity[%RH]
}

//--------------------------------------------------------------------
// calculates dew point
// input:   humidity [%RH], temperature [�C]
// output:  dew point [�C]
//--------------------------------------------------------------------

/****************************************************************************************************************************************//**
 * @brief
 *  Basic System-Variables are set-up, which do not need to be changed during Run-Time
 *
 * @details
 * The Parameterization of the Serial-Interface and the Selection of the Serial-Interface as well as the selection of Interface is done here!
 *
 * @note
 * The User should not change the Initialization-Values for System-Variables which are not related to the Hardware-Interfaces
 *******************************************************************************************************************************************/

float SHT1X_Sensirion::calc_dewpoint( float h, float t )
{ float k,dew_point ;

  k = (log10(h)-2)/0.4343 + (17.62*t)/(243.12+t);
  dew_point = 243.12*k/(17.62-k);
  return dew_point;
}


/****************************************************************************************************************************************//**
 * @brief
 *  This function reads out the temperature and humidity value,the results are available in the location handed over by two pointer to float.
 *
 *******************************************************************************************************************************************/

uint8_t SHT1X_Sensirion::getMeasurement( float &humidity, float &temperature )
{
	INPUT_VAR humid,temp;
	float bufferHumid, bufferTemp;
	unsigned char error,checksum;

	s_connectionreset();

	error = 0;
	error += s_measure(humid.in,&checksum,HUMI);  //measure humidity
	error += s_measure(temp.in, &checksum,TEMP);  //measure temperature


	if( error != 0 )
	{
	   	s_connectionreset();        //in case of an error: connection reset
	}

	else
	{
		bufferHumid = (float) humid.out;
		bufferTemp  = (float) temp.out;

	    calc_sth11(&bufferHumid,&bufferTemp);            //calculate humidity, temperature
	}

	humidity = bufferHumid;
	temperature= bufferTemp;

	return error;
}


uint8_t SHT1X_Sensirion::getMeasurement( uint16_t &humidity, uint16_t &temperature )
{
	INPUT_VAR humid,temp;
	float bufferHumid, bufferTemp;
	unsigned char error,checksum;

	s_connectionreset();

	error = 0;
	error += s_measure(humid.in,&checksum,HUMI);  //measure humidity
	error += s_measure(temp.in, &checksum,TEMP);  //measure temperature


	if( error != 0 )
	{
	   	s_connectionreset();        //in case of an error: connection reset
	}

	else
	{
		humidity = (float) humid.out;
		temperature  = (float) temp.out;

	    calc_sth11(&bufferHumid,&bufferTemp);            //calculate humidity, temperature
	}

	return error;
}

/****************************************************************************************************************************************//**
 * @brief
 *  This function reads out the temperature and humidity value, calculates and returns the dewpoint-value as float.
 *
 *******************************************************************************************************************************************/

float SHT1X_Sensirion::getDewpoint()
{
	float h,t;

	getMeasurement( h, t );

	return calc_dewpoint( h, t );
}
