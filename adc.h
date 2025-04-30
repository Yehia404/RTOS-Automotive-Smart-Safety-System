#ifndef __ADC_H__
#define __ADC_H__

#include <stdint.h>
#include "TM4C123GH6PM.h"

// Function prototypes
void ADC0_Init(void);
uint32_t ADC0_ReadChannel(uint8_t channel);

#endif // __ADC_H__