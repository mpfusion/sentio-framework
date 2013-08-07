#include "sentio_io.h"

#define digi1   gpioPortA, 0
#define digi2	gpioPortA, 2
#define digi3   gpioPortA, 1
#define digi4   gpioPortA, 4
#define digi5   gpioPortA, 3
#define digi6	gpioPortF, 4
#define digi7   gpioPortF, 5
#define digi8   gpioPortF, 3
#define digi9   gpioPortC, 15
#define digi10  gpioPortC, 14
#define digi11  gpioPortC, 13

#define maskInterruptButton1		0x0800
#define maskInterruptButton2		0x1000
#define maskInterruptButton3		0x2000
#define maskInterruptButton4		0x4000