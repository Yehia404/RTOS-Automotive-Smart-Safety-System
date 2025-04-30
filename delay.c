#include "delay.h"

void delay_ms(uint32_t ms)
{
    uint32_t i, j;
    for (i = 0; i < ms; i++)
    {
        for (j = 0; j < (SYSTEM_CLOCK / 10000); j++)
        {
            __NOP(); // No operation to create delay
        }
    }
}