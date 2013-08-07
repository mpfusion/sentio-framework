/*
 * Memory.cpp
 *
 *  Created on: Nov 23, 2011
 *      Author: matthias
 */

#include "Memory.h"

MEMORY_ELEMENT Memory::queue[_maxNumberOfElements];
void* 		   Memory::test;

Memory::Memory() {
	// TODO Auto-generated constructor stub

}

Memory::~Memory() {
	// TODO Auto-generated destructor stub
}


void Memory::initializeElement( void *element, uint8_t bufferLength, QUEUE varName )
{
	//test = element;

	queue[(uint8_t)varName].element      = element;
	queue[(uint8_t)varName].bufferLength = bufferLength;
}


bool Memory::getAccessToElement( void **element, uint8_t *bufferLength, QUEUE varName )
{
	*element = queue[(uint8_t)varName].element;
	*bufferLength = queue[(uint8_t)varName].bufferLength;

	return true;
}
