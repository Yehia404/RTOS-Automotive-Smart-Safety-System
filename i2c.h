#ifndef __I2C_H__
#define __I2C_H__

#include <stdint.h>
#include <stdbool.h>
#include "TM4C123GH6PM.h"

// Function prototypes
void I2C0_Init(void);
bool I2C0_SendByte(uint8_t slave_addr, uint8_t data);

#endif // __I2C_H__