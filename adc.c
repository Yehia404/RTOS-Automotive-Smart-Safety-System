#include "adc.h"

void ADC0_Init(void)
{
    // Enable clock for ADC0 and GPIO pin (PE3 for potentiometer)
    SYSCTL->RCGCADC |= (1 << 0);  // Enable ADC0 clock
    SYSCTL->RCGCGPIO |= (1 << 4); // Enable GPIOE clock

    // Wait for clocks to stabilize
    while ((SYSCTL->PRADC & (1 << 0)) == 0)
    {
    }
    while ((SYSCTL->PRGPIO & (1 << 4)) == 0)
    {
    }

    // Configure PE3 for analog input
    GPIOE->AFSEL |= (1 << 3); // Enable alternate function
    GPIOE->DEN &= ~(1 << 3);  // Disable digital function
    GPIOE->AMSEL |= (1 << 3); // Enable analog function

    // Configure ADC0
    ADC0->ACTSS &= ~(1 << 3); // Disable SS3 during configuration
    ADC0->EMUX &= ~0xF000;    // Configure for software trigger
    ADC0->SSMUX3 = 0;         // Select channel 0 (AIN0 = PE3)
    ADC0->SSCTL3 = 0x06;      // Set IE0 and END0 bits (single sample, interrupt generation)
    ADC0->PC = 0x3;           // Configure for 250ksps
    ADC0->ACTSS |= (1 << 3);  // Enable SS3
}

uint32_t ADC0_ReadChannel(uint8_t channel)
{
    // Set the channel to sample (only relevant for more complex ADC sequences)
    ADC0->SSMUX3 = channel;

    // Start sampling
    ADC0->PSSI |= (1 << 3);

    // Wait for conversion to complete
    while ((ADC0->RIS & (1 << 3)) == 0)
    {
    }

    // Read the conversion result
    uint32_t result = ADC0->SSFIFO3;

    // Clear the interrupt flag
    ADC0->ISC = (1 << 3);

    return result;
}