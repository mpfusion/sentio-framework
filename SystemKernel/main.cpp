/*
 * main.cpp
 *
 *  Created on: May 15, 2011
 *      Author: Matthias Krämer
 */

#include "efm32.h"
#include "efm32_chip.h"
#include "SystemConfig.h"

#include USERCODE_FILENAME
USERCODE_CLASSNAME    application;


/****************************************************************************************************************************************//**
 * @brief Main Application-Code
 * Initialize the Main-System and setup/execute the Application
 *******************************************************************************************************************************************/

int main(void)
{
	// Initialize chip, apply workaround and bug-fixes which are dependent on the chip-revision
	CHIP_Init();

	application.sentio.initializeSentioEM();


	application.setupApplication();
	application.executeApplication();

	return 0;
}
