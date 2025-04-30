#ifndef __DELAY_H__
#define __DELAY_H__

#include <stdint.h>
#include "TM4C123GH6PM.h"

// System clock frequency
#define SYSTEM_CLOCK 16000000 // Assuming 16MHz clock

// Function prototypes
void delay_ms(uint32_t ms);

#endif // __DELAY_H__