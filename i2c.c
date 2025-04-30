#include "i2c.h"

void I2C0_Init(void)
{
    // Enable clock for I2C0 and GPIOB
    SYSCTL->RCGCI2C |= (1 << 0);  // Enable I2C0 clock
    SYSCTL->RCGCGPIO |= (1 << 1); // Enable GPIOB clock

    // Wait for clock stabilization
    while ((SYSCTL->PRGPIO & (1 << 1)) == 0)
    {
    }
    while ((SYSCTL->PRI2C & (1 << 0)) == 0)
    {
    }

    // Configure PB2 (SCL) and PB3 (SDA) for I2C
    GPIOB->AFSEL |= (1 << 2) | (1 << 3);                   // Enable alternate function
    GPIOB->ODR |= (1 << 3);                                // SDA (PB3) open drain
    GPIOB->DEN |= (1 << 2) | (1 << 3);                     // Enable digital function
    GPIOB->PCTL = (GPIOB->PCTL & 0xFFFF00FF) | 0x00003300; // PMCx value = 3 for I2C

    // Configure I2C0
    I2C0->MCR = 0x00000010; // Initialize I2C0 master

    // Set I2C clock frequency
    // TPR = (System Clock / (2 * (SCL_LP + SCL_HP) * I2C_CLK)) - 1
    // For 100kHz, TPR = (16MHz / (2 * 10 * 100kHz)) - 1 = 7
    I2C0->MTPR = 7;
}

bool I2C0_SendByte(uint8_t slave_addr, uint8_t data)
{
    // Wait until I2C bus is not busy
    while (I2C0->MCS & 0x00000001)
    {
    }

    // Set slave address
    I2C0->MSA = slave_addr;

    // Load data to be transmitted
    I2C0->MDR = data;

    // Generate START, transmit data, and generate STOP (S+data+P)
    I2C0->MCS = 0x00000007;

    // Wait until transmission completes
    while (I2C0->MCS & 0x00000001)
    {
    }

    // Check for errors
    if (I2C0->MCS & 0x00000002)
    {
        return false; // Error occurred
    }

    return true; // Success
}