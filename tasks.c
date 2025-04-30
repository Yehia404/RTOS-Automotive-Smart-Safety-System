#include "tasks.h"
#include "adc.h"
#include "lcd.h"
#include "TM4C123GH6PM.h"
#include <stdio.h>
#include <string.h>

// Task handles
TaskHandle_t speedSensingHandle = NULL;
TaskHandle_t displayUpdateHandle = NULL;
TaskHandle_t switchMonitorHandle = NULL;

// Semaphore/mutex for LCD access
SemaphoreHandle_t lcdMutex = NULL;

// Queue for speed updates
QueueHandle_t speedQueue = NULL;

// System state
SystemState_t systemState = {
    .currentSpeed = 0,
    .doorsLocked = false,
    .ignitionOn = false,
    .manualLockOverride = false,
    .gearPosition = GEAR_PARK  // Initialize to PARK
};

// GPIO initialization for switches
void switches_init(void) {
    // Enable clock for GPIO ports
    SYSCTL->RCGCGPIO |= (1 << 5);  // Enable GPIOF clock for ignition button (PF4)
    SYSCTL->RCGCGPIO |= (1 << 1);  // Enable GPIOB clock for lock/unlock buttons (PB0, PB1)
    SYSCTL->RCGCGPIO |= (1 << 4);  // Enable GPIOE clock for gear shifter and potentiometer
    
    // Wait for clocks to stabilize
    while((SYSCTL->PRGPIO & (1 << 5)) == 0) {}
    while((SYSCTL->PRGPIO & (1 << 1)) == 0) {}
    while((SYSCTL->PRGPIO & (1 << 4)) == 0) {}
    
    // Configure PF4 as input with pull-up for ignition switch (push button)
    GPIOF->LOCK = 0x4C4F434B;         // Unlock GPIO registers
    GPIOF->CR = 0x1F;                 // Allow changes to PF4-0
    GPIOF->DIR &= ~IGNITION_PIN;      // Set as input
    GPIOF->PUR |= IGNITION_PIN;       // Enable pull-up resistor
    GPIOF->DEN |= IGNITION_PIN;       // Enable digital function
    
    // Configure PB0 (Lock button) and PB1 (Unlock button) as inputs with pull-up
    // Note: PB2 and PB3 are used for I2C LCD and configured in I2C initialization
    GPIOB->DIR &= ~(LOCK_BTN_PIN | UNLOCK_BTN_PIN); // Set as inputs
    GPIOB->PUR |= (LOCK_BTN_PIN | UNLOCK_BTN_PIN);  // Enable pull-up resistors
    GPIOB->DEN |= (LOCK_BTN_PIN | UNLOCK_BTN_PIN);  // Enable digital function
    
    // Configure PE0, PE1, PE2 as inputs with pull-ups for gear shifter
    // Note: PE3 is used for potentiometer and configured in ADC initialization
    GPIOE->DIR &= ~(GEAR_PARK_PIN | GEAR_DRIVE_PIN | GEAR_REVERSE_PIN);  // Set as inputs
    GPIOE->PUR |= (GEAR_PARK_PIN | GEAR_DRIVE_PIN | GEAR_REVERSE_PIN);   // Enable pull-up resistors
    GPIOE->DEN |= (GEAR_PARK_PIN | GEAR_DRIVE_PIN | GEAR_REVERSE_PIN);   // Enable digital function
}

// Task initialization function
void tasks_init(void) {
    // Initialize GPIO for switches
    switches_init();
    
    // Create mutex for LCD access
    lcdMutex = xSemaphoreCreateMutex();
    
    // Create queue for speed updates
    speedQueue = xQueueCreate(1, sizeof(uint32_t));
    
    // Create tasks
    if (lcdMutex != NULL && speedQueue != NULL) {
        xTaskCreate(SpeedSensingTask, "SpeedSensing", configMINIMAL_STACK_SIZE, NULL, 2, &speedSensingHandle);
        xTaskCreate(DisplayUpdateTask, "DisplayUpdate", configMINIMAL_STACK_SIZE, NULL, 1, &displayUpdateHandle);
        xTaskCreate(SwitchMonitorTask, "SwitchMonitor", configMINIMAL_STACK_SIZE, NULL, 3, &switchMonitorHandle);
    } else {
        // Handle resource creation failure
        while(1) {}
    }
}

void SpeedSensingTask(void *pvParameters) {
    uint32_t adcValue;
    uint32_t speedValue;
    
    // Define the expected ADC range based on your potentiometer behavior
    const uint32_t ADC_MIN = 450;      // Value when pot is at minimum
    const uint32_t ADC_MAX = 3800;     // Value when pot is at maximum
    const uint32_t SPEED_MIN = 0;
    const uint32_t SPEED_MAX = 180;
    
    while(1) {
        // Only measure speed if ignition is ON and gear is in Drive or Reverse
        if (systemState.ignitionOn && 
            (systemState.gearPosition == GEAR_DRIVE || systemState.gearPosition == GEAR_REVERSE)) {
            
            // Read potentiometer value from ADC
            adcValue = ADC0_ReadChannel(0);
            
            // Map ADC value to speed using the calibrated range
            if (adcValue < ADC_MIN) {
                // Below minimum - set to minimum speed
                speedValue = SPEED_MIN;
            } 
            else if (adcValue > ADC_MAX) {
                // Above maximum - set to maximum speed
                speedValue = SPEED_MAX;
            } 
            else {
                // Within range - linear mapping
                speedValue = SPEED_MIN + ((adcValue - ADC_MIN) * (SPEED_MAX - SPEED_MIN)) / (ADC_MAX - ADC_MIN);
            }
            
            // Auto-lock doors if speed exceeds threshold and no manual override is active
            if (speedValue >= 10 && !systemState.doorsLocked && !systemState.manualLockOverride) {
                systemState.doorsLocked = true;
            }
        } else {
            // Ignition is OFF or gear is in Park, set speed to 0
            speedValue = 0;
        }
        
        // Update system state with current speed
        systemState.currentSpeed = speedValue;
        
        // Send speed value to queue for display task
        xQueueOverwrite(speedQueue, &speedValue);
        
        // Task delay (100ms)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void SwitchMonitorTask(void *pvParameters) {
    // Button states - High when not pressed (pull-up)
    bool prevIgnitionBtn = true;
    bool prevLockBtn = true;
    bool prevUnlockBtn = true;
    
    bool currIgnitionBtn;
    bool currLockBtn;
    bool currUnlockBtn;
    uint8_t gearSwitches;
    
    const TickType_t debounceDelay = pdMS_TO_TICKS(50);  // 50ms debounce delay
    
    while(1) {
        // Read all button states (active low with pull-up resistors)
        currIgnitionBtn = (GPIOF->DATA & IGNITION_PIN) ? true : false;     // PF4 for ignition
        currLockBtn = (GPIOB->DATA & LOCK_BTN_PIN) ? true : false;         // PB0 for lock
        currUnlockBtn = (GPIOB->DATA & UNLOCK_BTN_PIN) ? true : false;     // PB1 for unlock
        
        // Read gear position from switches (active low with pull-ups)
        // Read only the relevant bits (PE0-PE2)
        gearSwitches = (~GPIOE->DATA) & (GEAR_PARK_PIN | GEAR_DRIVE_PIN | GEAR_REVERSE_PIN);
        
        // Update gear position based on switch setting
        // This assumes that only one switch is active at a time
        if (gearSwitches & GEAR_PARK_PIN) {
            systemState.gearPosition = GEAR_PARK;
        } else if (gearSwitches & GEAR_DRIVE_PIN) {
            systemState.gearPosition = GEAR_DRIVE;
        } else if (gearSwitches & GEAR_REVERSE_PIN) {
            systemState.gearPosition = GEAR_REVERSE;
        } else {
            // Default to PARK if no switch is active or multiple switches are active
            systemState.gearPosition = GEAR_PARK;
        }
        
        // Check for ignition button press (HIGH to LOW transition)
        if (prevIgnitionBtn == true && currIgnitionBtn == false) {
            // Debounce delay
            vTaskDelay(debounceDelay);
            
            // Read button state again to confirm press
            currIgnitionBtn = (GPIOF->DATA & IGNITION_PIN) ? true : false;
            
            if (currIgnitionBtn == false) {
                // Toggle ignition state
                systemState.ignitionOn = !systemState.ignitionOn;
                
                // If turning ignition off, always unlock doors - override priority
                if (!systemState.ignitionOn) {
                    systemState.doorsLocked = false;
                    // Manual override continues to apply for lock but not for unlock when ignition goes off
                }
            }
        }
        
        // Check for lock button press (HIGH to LOW transition)
        if (prevLockBtn == true && currLockBtn == false) {
            // Debounce delay
            vTaskDelay(debounceDelay);
            
            // Read button state again to confirm press
            currLockBtn = (GPIOB->DATA & LOCK_BTN_PIN) ? true : false;
            
            if (currLockBtn == false) {
                // Manually lock doors
                systemState.doorsLocked = true;
                // Set manual override flag to prevent auto lock/unlock based on speed
                systemState.manualLockOverride = true;
            }
        }
        
        // Check for unlock button press (HIGH to LOW transition)
        if (prevUnlockBtn == true && currUnlockBtn == false) {
            // Debounce delay
            vTaskDelay(debounceDelay);
            
            // Read button state again to confirm press
            currUnlockBtn = (GPIOB->DATA & UNLOCK_BTN_PIN) ? true : false;
            
            if (currUnlockBtn == false) {
                // Manually unlock doors
                systemState.doorsLocked = false;
                // Set manual override flag to prevent auto lock when speed increases
                systemState.manualLockOverride = true;
                
                // Reset override after 5 seconds (50 * 100ms) to allow auto logic to resume
                for (int i = 0; i < 50; i++) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    
                    // Exit early if another button was pressed
                    if ((GPIOB->DATA & LOCK_BTN_PIN) == 0 || (GPIOB->DATA & UNLOCK_BTN_PIN) == 0) {
                        break;
                    }
                }
                
                // Reset manual override flag after delay (unless another button press occurred)
                if ((GPIOB->DATA & LOCK_BTN_PIN) && (GPIOB->DATA & UNLOCK_BTN_PIN)) {
                    systemState.manualLockOverride = false;
                }
            }
        }
        
        // Update previous states
        prevIgnitionBtn = currIgnitionBtn;
        prevLockBtn = currLockBtn;
        prevUnlockBtn = currUnlockBtn;
        
        // Task delay (20ms - fast enough to detect button presses)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void DisplayUpdateTask(void *pvParameters) {
    char speedStr[16];
    char statusStr[16];
    char gearStr[8];
    uint32_t currentSpeed = 0;
    
    while(1) {
        // Receive latest speed value from queue with a timeout
        if (xQueueReceive(speedQueue, &currentSpeed, pdMS_TO_TICKS(10)) != pdTRUE) {
            // Queue receive timed out, use last known value
        }
        
        // Take mutex to access LCD
        if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Clear display
            LCD_Clear();
            
            // Display speed on first line
            LCD_Set_Cursor(0, 0);
            LCD_Print("Speed:");
            
            // Convert speed to string and display
            sprintf(speedStr, "%3lu km/h", currentSpeed);
            LCD_Set_Cursor(0, 7);
            LCD_Print(speedStr);
            
            // Display gear, ignition, and door status on second line
            LCD_Set_Cursor(1, 0);
            
            // Get gear string
            switch(systemState.gearPosition) {
                case GEAR_PARK:
                    strcpy(gearStr, "P ");
                    break;
                case GEAR_REVERSE:
                    strcpy(gearStr, "R ");
                    break;
                case GEAR_DRIVE:
                    strcpy(gearStr, "D ");
                    break;
                default:
                    strcpy(gearStr, "? ");
                    break;
            }
            
            // Format status string with gear position
            if (systemState.ignitionOn) {
                sprintf(statusStr, "%sON ", gearStr);
            } else {
                sprintf(statusStr, "%sOFF ", gearStr);
            }
            
            if (systemState.doorsLocked) {
                strcat(statusStr, "LOCK");
                // If manual override, add indicator
                if (systemState.manualLockOverride) {
                    strcat(statusStr, "*");
                }
            } else {
                strcat(statusStr, "UNLK");
                // If manual override, add indicator
                if (systemState.manualLockOverride) {
                    strcat(statusStr, "*");
                }
            }
            
            LCD_Print(statusStr);
            
            // Release the mutex
            xSemaphoreGive(lcdMutex);
        }
        
        // Task delay (200ms)
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}