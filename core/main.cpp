/*
 * main.cpp
 *
 *  Created on: May 15, 2011
 *      Author: Matthias Krämer
 */

#include "application.h"

APPLICATION    application;

/****************************************************************************************************************************************//**
 * @brief Main Application-Code
 * Initialize the Main-System and setup/execute the Application
 *******************************************************************************************************************************************/

int main(void)
{
	application.init();
	application.run();

	return 0;
}
