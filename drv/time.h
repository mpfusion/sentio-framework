/*
 * time.h
 *
 *  Created on: Nov 26, 2011
 *      Author: matthias
 */

#ifndef TIME_H_
#define TIME_H_

#include "efm32.h"

class time {

union SYSTEMTIME {
	uint8_t arrayAccess[7];
	struct{
		uint8_t second;
		uint8_t minute;
		uint8_t hour;
		uint8_t day;
		uint8_t date;
		uint8_t month;
		uint8_t year;
	}config;
};

private:
	SYSTEMTIME var;

public:
	time( uint8_t second, uint8_t minute, uint8_t hour, uint8_t day = 1, uint8_t date = 1, uint8_t month = 1, uint8_t year = 0 );
	time( uint32_t seconds );
	time(){}
	~time(){}

	uint32_t getInSeconds();
	uint8_t* getArrayAccess();
	uint8_t  getSecond();
	uint8_t  getMinute();
	uint8_t  getHour();

	time 	 operator+ ( const time &summand );
	time	 operator- ( const time &subtrahend );
	uint32_t operator/ ( const time &divisor );
	time 	 operator/ ( const uint32_t divisor );
	time 	 operator* ( uint32_t multiplicant );
	time     operator= ( const time &source );

	bool     operator> ( const time input );
	bool     operator< ( const time input );
	bool     operator<=( const time input );
	bool     operator>=( const time input );
	bool	 operator==( const time input );
	bool 	 operator!=( const time input );
};
#endif /* TIME_H_ */
