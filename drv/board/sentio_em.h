/*
 * sentioem.h
 *
 *  Created on: Aug 2, 2013
 *      Author: Matthias Krämer
 */

#ifndef SENTIO_EM_H_
#define SENTIO_EM_H_

#include "em_gpio.h"

/****************************************************************************************************************************************//**
 * @brief
 *  Declaration of the "system"-Class.
 *
 *******************************************************************************************************************************************/

class SENTIO_EM
{
public:
	SENTIO_EM() {}
	~SENTIO_EM() {}

	void init();
	
	void setLED( int port, unsigned int pin );
	void tglLED( GPIO_Port_TypeDef port, unsigned int pin );
	void clrLED( GPIO_Port_TypeDef port, unsigned int pin );

};


#endif /* SENTIO_EM_H_ */
