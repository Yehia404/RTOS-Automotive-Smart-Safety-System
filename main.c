#include <stdint.h>
#include <stdbool.h>
#include "TM4C123GH6PM.h"
#include "adc.h"
#include "lcd.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tasks.h"

int main(void)
{
    // Initialize system
    SystemInit();

    // Initialize peripherals
    ADC0_Init();
    I2C0_Init();
    LCD_Init();

    // Create and start all tasks
    tasks_init();

    // Start the scheduler
    vTaskStartScheduler();

    // Should never get here
    while (1)
    {
    }
}