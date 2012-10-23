/*
 * time.cpp
 *
 *  Created on: Nov 26, 2011
 *      Author: matthias
 */

#include "time.h"

time::time(uint8_t second, uint8_t minute, uint8_t hour, uint8_t day, uint8_t date, uint8_t month, uint8_t year )
{

	var.config.second = second;
	var.config.minute = minute;
	var.config.hour   = hour;

	var.config.day    = day;
	var.config.date   = date;
	var.config.month  = month;
	var.config.year   = year;
}

time::time( uint32_t seconds )
{
	var.config.day    = 1;
	var.config.date   = 1;
	var.config.month  = 1;
	var.config.year   = 0;

	var.config.hour   = seconds/3600;
	seconds           = seconds%3600;

	var.config.minute = seconds/60;

	var.config.second = seconds%60;
}


uint8_t*time::getArrayAccess()
{
	return var.arrayAccess;
}


uint32_t time::getInSeconds()
{
	return ( var.config.hour  * 60 + var.config.minute ) * 60 + var.config.second;
}

uint8_t time::getSecond()
{
	return var.config.second;
}

uint8_t time::getMinute()
{
	return var.config.minute;
}

uint8_t time::getHour()
{
	return var.config.hour;
}

time time::operator+ ( const time &summand )
{
	uint32_t temp = ( ( var.config.hour + summand.var.config.hour ) * 60 + ( var.config.minute + summand.var.config.minute ) )\
					* 60 + var.config.second + summand.var.config.second;

	uint8_t tempHour;
	uint8_t tempMin;
	uint8_t tempSec;

	tempHour = temp/3600;
	temp     = temp%3600;

	tempMin  = temp/60;
	tempSec  = temp%60;

	return time( tempSec, tempMin, tempHour );
}



time time::operator- ( const time &subtrahend )
{
	uint32_t temp = ( var.config.hour  * 60 + var.config.minute ) * 60 + var.config.second;
	uint32_t sub  = ( subtrahend.var.config.hour  * 60 + subtrahend.var.config.minute ) * 60 + subtrahend.var.config.second;

	uint8_t tempHour;
	uint8_t tempMin;
	uint8_t tempSec;

	if( sub >= temp )
	{
		tempHour = 0;
		tempMin  = 0;
		tempSec  = 0;
	}

	else
	{
		temp -= sub;

		tempHour = temp / 3600;
		temp     = temp % 3600;
		tempMin  = temp / 60;
		tempSec  = temp % 60;
	}

	return time( tempSec, tempMin, tempHour );
}


uint32_t time::operator/ ( const time &divisor )
{
	uint32_t temp = ( var.config.hour  * 60 + var.config.minute ) * 60 + var.config.second;
	uint32_t div  = ( divisor.var.config.hour  * 60 + divisor.var.config.minute ) * 60 + divisor.var.config.second;

	if( div > temp )
	{
		return 0;
	}

	else
	{
		return temp / div;
	}
}

time time::operator/ ( uint32_t divisor )
{
	uint32_t temp = ( var.config.hour  * 60 + var.config.minute ) * 60 + var.config.second;

	uint8_t tempHour;
	uint8_t tempMin;
	uint8_t tempSec;

	if( divisor > temp )
	{
		tempHour = 0;
		tempMin  = 0;
		tempSec  = 0;
	}

	else
	{
		temp /= divisor;

		tempHour = temp / 3600;
		temp     = temp % 3600;
		tempMin  = temp / 60;
		tempSec  = temp % 60;
	}

	return time( tempSec, tempMin, tempHour );
}

time time::operator* ( uint32_t multiplicant )
{
	uint32_t temp = ( var.config.hour  * 60 + var.config.minute ) * 60 + var.config.second;

	temp *= multiplicant;

	uint8_t tempHour;
	uint8_t tempMin;
	uint8_t tempSec;

	tempHour = temp / 3600;
	temp     = temp % 3600;
	tempMin  = temp / 60;
	tempSec  = temp % 60;

	return time( tempSec, tempMin, tempHour );
}

time time::operator= ( const time &source )
{
	var.config.second = source.var.config.second;
	var.config.minute = source.var.config.minute;
	var.config.hour   = source.var.config.hour;

	var.config.day    = source.var.config.day;
	var.config.date   = source.var.config.date;
	var.config.month  = source.var.config.month;
	var.config.year   = source.var.config.year;
}

bool time::operator< ( const time input )
{
	uint32_t temp = ( var.config.hour  * 60 + var.config.minute ) * 60 + var.config.second;
	uint32_t in  = ( input.var.config.hour  * 60 + input.var.config.minute ) * 60 + input.var.config.second;

	if( temp < in )
		return true;
	else
		return false;
}

bool time::operator> ( const time input )
{
	uint32_t temp = ( var.config.hour  * 60 + var.config.minute ) * 60 + var.config.second;
	uint32_t in  = ( input.var.config.hour  * 60 + input.var.config.minute ) * 60 + input.var.config.second;

	if( temp > in )
		return true;
	else
		return false;
}

bool time::operator<= ( const time input )
{
	uint32_t temp = ( var.config.hour  * 60 + var.config.minute ) * 60 + var.config.second;
	uint32_t in  = ( input.var.config.hour  * 60 + input.var.config.minute ) * 60 + input.var.config.second;

	if( temp <= in )
		return true;
	else
		return false;
}

bool time::operator>= ( const time input )
{
	uint32_t temp = ( var.config.hour  * 60 + var.config.minute ) * 60 + var.config.second;
	uint32_t in  = ( input.var.config.hour  * 60 + input.var.config.minute ) * 60 + input.var.config.second;

	if( temp >= in )
		return true;
	else
		return false;
}

bool time::operator== ( const time input )
{
	uint32_t temp = ( var.config.hour  * 60 + var.config.minute ) * 60 + var.config.second;
	uint32_t in  = ( input.var.config.hour  * 60 + input.var.config.minute ) * 60 + input.var.config.second;

	if( temp == in)
		return true;
	else
		return false;
}

bool time::operator!= ( const time input )
{
	uint32_t temp = ( var.config.hour  * 60 + var.config.minute ) * 60 + var.config.second;
	uint32_t in  = ( input.var.config.hour  * 60 + input.var.config.minute ) * 60 + input.var.config.second;

	if( temp != in)
		return true;
	else
		return false;
}
