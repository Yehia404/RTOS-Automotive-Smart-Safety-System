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
TaskHandle_t ultrasonicTaskHandle = NULL;
TaskHandle_t alertHandlerHandle = NULL;

// Semaphores/mutexes
SemaphoreHandle_t lcdMutex = NULL;
SemaphoreHandle_t rgbLedMutex = NULL;
SemaphoreHandle_t buzzerMutex = NULL;

// Queues
QueueHandle_t speedQueue = NULL;
QueueHandle_t distanceQueue = NULL;

// System state
SystemState_t systemState = {
    .currentSpeed = 0,
    .doorsLocked = false,
    .ignitionOn = false,
    .manualLockOverride = false,
    .gearPosition = GEAR_PARK,  // Initialize to PARK
    .parkingAssistActive = false
};

// GPIO initialization for switches
void switches_init(void) {
    // Enable clock for GPIO ports
    SYSCTL->RCGCGPIO |= (1 << 5);  // Enable GPIOF clock for ignition button (PF4)
    SYSCTL->RCGCGPIO |= (1 << 1);  // Enable GPIOB clock for lock/unlock buttons (PB0, PB1)
    SYSCTL->RCGCGPIO |= (1 << 4);  // Enable GPIOE clock for gear shifter and potentiometer
    SYSCTL->RCGCGPIO |= (1 << 0);  // Enable GPIOA clock for door switch (if not already enabled for ultrasonic)
    
    // Wait for clocks to stabilize
    while((SYSCTL->PRGPIO & (1 << 5)) == 0) {}
    while((SYSCTL->PRGPIO & (1 << 1)) == 0) {}
    while((SYSCTL->PRGPIO & (1 << 4)) == 0) {}
    while((SYSCTL->PRGPIO & (1 << 0)) == 0) {}
    
    // Configure PF4 as input with pull-up for ignition switch
    GPIOF->LOCK = 0x4C4F434B;         // Unlock GPIO registers
    GPIOF->CR = 0x1F;                 // Allow changes to PF4-0
    GPIOF->DIR &= ~IGNITION_PIN;      // Set as input
    GPIOF->PUR |= IGNITION_PIN;       // Enable pull-up resistor
    GPIOF->DEN |= IGNITION_PIN;       // Enable digital function
    
    // Configure PB0 (Lock button) and PB1 (Unlock button) as inputs with pull-up
    GPIOB->DIR &= ~(LOCK_BTN_PIN | UNLOCK_BTN_PIN); // Set as inputs
    GPIOB->PUR |= (LOCK_BTN_PIN | UNLOCK_BTN_PIN);  // Enable pull-up resistors
    GPIOB->DEN |= (LOCK_BTN_PIN | UNLOCK_BTN_PIN);  // Enable digital function
    
    // Configure PE0, PE1, PE2 as inputs with pull-ups for gear shifter
    GPIOE->DIR &= ~(GEAR_PARK_PIN | GEAR_DRIVE_PIN | GEAR_REVERSE_PIN);  // Set as inputs
    GPIOE->PUR |= (GEAR_PARK_PIN | GEAR_DRIVE_PIN | GEAR_REVERSE_PIN);   // Enable pull-up resistors
    GPIOE->DEN |= (GEAR_PARK_PIN | GEAR_DRIVE_PIN | GEAR_REVERSE_PIN);   // Enable digital function
    
    // Configure PA5 as input with pull-up for door switch
    GPIOA->DIR &= ~DOOR_SWITCH_PIN;   // Set as input
    GPIOA->PUR |= DOOR_SWITCH_PIN;    // Enable pull-up resistor
    GPIOA->DEN |= DOOR_SWITCH_PIN;    // Enable digital function
}

// Initialize ultrasonic sensor, RGB LED, and buzzer
void ultrasonic_system_init(void) {
    // Enable clocks for required ports
    SYSCTL->RCGCGPIO |= (1 << 0);  // Enable GPIOA clock for ultrasonic sensor and buzzer
    
    // Wait for clocks to stabilize
    while((SYSCTL->PRGPIO & (1 << 0)) == 0) {}
    
    // Configure PA2 (Trigger) as output and PA3 (Echo) as input
    GPIOA->DIR |= TRIG_PIN;        // Set Trigger as output
    GPIOA->DIR &= ~ECHO_PIN;       // Set Echo as input
    GPIOA->DEN |= (TRIG_PIN | ECHO_PIN); // Enable digital function
    
    // Configure PA4 as output for buzzer with enhanced drive capability
    GPIOA->DIR |= BUZZER_PIN;      // Set as output
    GPIOA->DEN |= BUZZER_PIN;      // Enable digital function
    GPIOA->DR8R |= BUZZER_PIN;     // Enable 8mA drive capability for buzzer
    GPIOA->DATA &= ~BUZZER_PIN;    // Ensure buzzer is off initially
    
    // Configure PA5 as input with pull-up for door switch
    GPIOA->DIR &= ~DOOR_SWITCH_PIN;// Set as input
    GPIOA->PUR |= DOOR_SWITCH_PIN; // Enable pull-up
    GPIOA->DEN |= DOOR_SWITCH_PIN; // Enable digital function
    
    // Configure PF1 (Red), PF2 (Blue), PF3 (Green) for RGB LED
    GPIOF->LOCK = 0x4C4F434B;      // Unlock GPIO registers
    GPIOF->CR = 0x1F;              // Allow changes to PF0-PF4
    GPIOF->DIR |= (RED_PIN | BLUE_PIN | GREEN_PIN); // Set as outputs
    GPIOF->DEN |= (RED_PIN | BLUE_PIN | GREEN_PIN); // Enable digital function
    
    // Initialize all outputs to LOW (off)
    GPIOA->DATA &= ~TRIG_PIN;
    GPIOA->DATA &= ~BUZZER_PIN;
    GPIOF->DATA &= ~(RED_PIN | BLUE_PIN | GREEN_PIN);
}

// Task initialization function
void tasks_init(void) {
    // Initialize GPIO for switches
    switches_init();
    
    // Initialize ultrasonic system
    ultrasonic_system_init();
    
    // Create semaphores/mutexes
    lcdMutex = xSemaphoreCreateMutex();
    rgbLedMutex = xSemaphoreCreateMutex();
    buzzerMutex = xSemaphoreCreateMutex();
    
    // Create queues
    speedQueue = xQueueCreate(1, sizeof(uint32_t));
    distanceQueue = xQueueCreate(1, sizeof(uint32_t));
    
    // Create tasks if resources were created successfully
    if (lcdMutex != NULL && rgbLedMutex != NULL && buzzerMutex != NULL && 
        speedQueue != NULL && distanceQueue != NULL) {
        
        xTaskCreate(SpeedSensingTask, "SpeedSensing", configMINIMAL_STACK_SIZE, NULL, 2, &speedSensingHandle);
        xTaskCreate(DisplayUpdateTask, "DisplayUpdate", configMINIMAL_STACK_SIZE, NULL, 1, &displayUpdateHandle);
        xTaskCreate(SwitchMonitorTask, "SwitchMonitor", configMINIMAL_STACK_SIZE, NULL, 3, &switchMonitorHandle);
        xTaskCreate(UltrasonicTask, "Ultrasonic", configMINIMAL_STACK_SIZE, NULL, 2, &ultrasonicTaskHandle);
        xTaskCreate(AlertHandlerTask, "AlertHandler", configMINIMAL_STACK_SIZE, NULL, 2, &alertHandlerHandle);
    } else {
        // Handle resource creation failure
        while(1) {}
    }
}

void SpeedSensingTask(void *pvParameters) {
    uint32_t adcValue;
    uint32_t speedValue;
    
    // Define the expected ADC range based on your potentiometer behavior
    const uint32_t ADC_MIN = 300;      // Value when pot is at minimum
    const uint32_t ADC_MAX = 3900;     // Value when pot is at maximum
    const uint32_t SPEED_MIN = 0;
    const uint32_t SPEED_MAX = 220;
    
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
            if (speedValue >= 60 && !systemState.doorsLocked && !systemState.manualLockOverride) {
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
    bool prevDoorSwitch = true;  // Previous state of door switch (HIGH = closed)
    
    bool currIgnitionBtn;
    bool currLockBtn;
    bool currUnlockBtn;
    bool currDoorSwitch;         // Current state of door switch
    uint8_t gearSwitches;
    
    const TickType_t debounceDelay = pdMS_TO_TICKS(50);  // 50ms debounce delay
    
    while(1) {
        // Read all button states (active low with pull-up resistors)
        currIgnitionBtn = (GPIOF->DATA & IGNITION_PIN) ? true : false;      // PF4 for ignition
        currLockBtn = (GPIOB->DATA & LOCK_BTN_PIN) ? true : false;          // PB0 for lock
        currUnlockBtn = (GPIOB->DATA & UNLOCK_BTN_PIN) ? true : false;      // PB1 for unlock
        currDoorSwitch = (GPIOA->DATA & DOOR_SWITCH_PIN) ? true : false;    // PA5 for door switch
        
        // Read gear position from switches (active low with pull-ups)
        // Read only the relevant bits (PE0-PE2)
        gearSwitches = (~GPIOE->DATA) & (GEAR_PARK_PIN | GEAR_DRIVE_PIN | GEAR_REVERSE_PIN);
        
        // Update gear position based on switch setting and ignition state
        if (systemState.ignitionOn) {
            // Only allow gear changes when ignition is ON
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
        }
        // When ignition is OFF, the gear position remains unchanged regardless of switch position
        
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
                
                // If door is open when trying to lock, close it automatically (in reality, this would be a warning)
                if (systemState.driverDoorOpen) {
                    systemState.driverDoorOpen = false;
                }
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
        
        // Check for door switch state change
        if (prevDoorSwitch != currDoorSwitch) {
            // Debounce delay
            vTaskDelay(debounceDelay);
            
            // Read switch state again to confirm change
            currDoorSwitch = (GPIOA->DATA & DOOR_SWITCH_PIN) ? true : false;
            
            if (prevDoorSwitch != currDoorSwitch) {
                // Attempt to open door (switch LOW)
                if (!currDoorSwitch) {
                    // Check if doors are locked
                    if (systemState.doorsLocked) {
                        // Door is locked, prevent opening - ignore switch press
                        // Here you could add code to provide feedback (e.g., flash LED)
                        
                        // Flash the RGB LED red briefly to indicate door can't be opened
                        if (xSemaphoreTake(rgbLedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                            // Flash red LED
                            GPIOF->DATA |= RED_PIN;
                            GPIOF->DATA &= ~(BLUE_PIN | GREEN_PIN);
                            xSemaphoreGive(rgbLedMutex);
                            
                            vTaskDelay(pdMS_TO_TICKS(200)); // Flash for 200ms
                            
                            // Turn off if no other alerts are active
                            if (xSemaphoreTake(rgbLedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                                // Only turn off if we're not in parking assist mode
                                if (!systemState.parkingAssistActive) {
                                    GPIOF->DATA &= ~(RED_PIN | BLUE_PIN | GREEN_PIN);
                                }
                                xSemaphoreGive(rgbLedMutex);
                            }
                        }
                        
                        // Door state remains unchanged (closed)
                    } else {
                        // Door is unlocked, allow opening
                        systemState.driverDoorOpen = true;
                    }
                } else {
                    // Door is being closed (switch HIGH)
                    systemState.driverDoorOpen = false;
                }
            }
        }
        
        // Update previous states
        prevIgnitionBtn = currIgnitionBtn;
        prevLockBtn = currLockBtn;
        prevUnlockBtn = currUnlockBtn;
        prevDoorSwitch = currDoorSwitch;
        
        // Task delay (20ms - fast enough to detect button presses)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void UltrasonicTask(void *pvParameters) {
    uint32_t distance;
    uint32_t pulseWidth;
    uint32_t startTime, endTime;
    TickType_t lastWakeTime;
    const TickType_t measurementPeriod = pdMS_TO_TICKS(100); // 100ms measurement period
    
    // Initialize the last wake time
    lastWakeTime = xTaskGetTickCount();
    
    while(1) {
        // Check if we should activate parking assist (in reverse gear)
        systemState.parkingAssistActive = (systemState.ignitionOn && 
                                          systemState.gearPosition == GEAR_REVERSE);
        
        if (systemState.parkingAssistActive) {
            // 1. Clear the trigger pin
            GPIOA->DATA &= ~TRIG_PIN;
            vTaskDelay(pdMS_TO_TICKS(2)); // Short delay
            
            // 2. Send 10us pulse to trigger
            GPIOA->DATA |= TRIG_PIN;
            vTaskDelay(pdMS_TO_TICKS(1)); // Wait for at least 10us (minimum 1 tick)
            GPIOA->DATA &= ~TRIG_PIN;
            
            // 3. Wait for echo pin to go high
            while((GPIOA->DATA & ECHO_PIN) == 0) {
                // Add timeout check to prevent getting stuck
                if (xTaskGetTickCount() - lastWakeTime > pdMS_TO_TICKS(100)) {
                    break;
                }
            }
            
            // 4. Measure pulse width (time between rising and falling edge)
            startTime = xTaskGetTickCount();
            
            // Wait for echo pin to go low
            while((GPIOA->DATA & ECHO_PIN) != 0) {
                // Add timeout check to prevent getting stuck
                if (xTaskGetTickCount() - startTime > pdMS_TO_TICKS(100)) {
                    break;
                }
            }
            
            endTime = xTaskGetTickCount();
            
            // 5. Calculate pulse width in milliseconds
            pulseWidth = (endTime - startTime);
            
            // 6. Convert pulse width to distance (speed of sound is 343 m/s)
            // Distance = (time * speed of sound) / 2 (divide by 2 because sound travels to object and back)
            // For HC-SR04: Distance (cm) = pulse width (µs) / 58
            // Adjust calculation based on tick rate and units
            distance = (pulseWidth * 343 * 100) / (2 * configTICK_RATE_HZ);
            
            // 7. Send distance measurement to queue (overwrite previous value if not consumed)
            xQueueOverwrite(distanceQueue, &distance);
        } else {
            // If not in reverse, send a large value to indicate "no obstacle"
            distance = 0xFFFFFFFF;
            xQueueOverwrite(distanceQueue, &distance);
        }
        
        // Wait for the next measurement cycle
        vTaskDelayUntil(&lastWakeTime, measurementPeriod);
    }
}

void AlertHandlerTask(void *pvParameters) {
    uint32_t currentDistance;
    TickType_t buzzerDelay = pdMS_TO_TICKS(200); // Default delay
    TickType_t lastWakeTime;
    bool buzzerState = false;
    uint32_t currentSpeed = 0;
    
    // Initialize last wake time
    lastWakeTime = xTaskGetTickCount();
    
    // One-time direct test of buzzer to check hardware
    GPIOA->DATA |= BUZZER_PIN;  // Turn buzzer on
    vTaskDelay(pdMS_TO_TICKS(500)); // Keep on for 500ms
    GPIOA->DATA &= ~BUZZER_PIN; // Turn buzzer off
    
    while(1) {
        // Try to receive latest distance from queue (non-blocking)
        if (xQueuePeek(distanceQueue, &currentDistance, 0) != pdTRUE) {
            // If no new distance data, use a very large value
            currentDistance = 0xFFFFFFFF;
        }
        
        // Get current speed
        if (xQueuePeek(speedQueue, &currentSpeed, 0) != pdTRUE) {
            // If no speed data, assume stopped
            currentSpeed = 0;
        }
        
        // Check for door open warning condition (door open while moving)
        bool doorOpenWarning = systemState.driverDoorOpen && 
                              systemState.ignitionOn && 
                              currentSpeed > 0;
        
        // Determine action based on active warnings
        if (doorOpenWarning) {
            // Door open while moving warning - highest priority
            buzzerDelay = pdMS_TO_TICKS(200); // Fast beeping for door warning
            
            // Toggle RGB LED - avoid using red only as it might conflict with onboard LED
            if (xSemaphoreTake(rgbLedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                // Use a different color combination to avoid red-only conflicts
                // Alternating between blue+green and all off
                if (buzzerState) {
                    GPIOF->DATA |= (BLUE_PIN | GREEN_PIN);  // Blue+Green on
                    GPIOF->DATA &= ~RED_PIN;                // Red off
                } else {
                    GPIOF->DATA &= ~(RED_PIN | BLUE_PIN | GREEN_PIN); // All LEDs off
                }
                xSemaphoreGive(rgbLedMutex);
            }
            
            // Direct control of buzzer without mutex for testing
            buzzerState = !buzzerState;
            if (buzzerState) {
                // Try direct register write to control buzzer
                GPIOA->DATA = GPIOA->DATA | BUZZER_PIN;  // Turn buzzer on
            } else {
                GPIOA->DATA = GPIOA->DATA & ~BUZZER_PIN; // Turn buzzer off
            }
        } 
        else if (systemState.parkingAssistActive && currentDistance != 0xFFFFFFFF) {
            // Parking assist active - handle proximity alerts
            if (xSemaphoreTake(rgbLedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                // Set RGB LED based on distance
                if (currentDistance < DANGER_ZONE) {
                    // Red - Danger zone
                    GPIOF->DATA = (GPIOF->DATA & ~(BLUE_PIN | GREEN_PIN)) | RED_PIN;
                    buzzerDelay = pdMS_TO_TICKS(100); // Fast beeping (100ms)
                } else if (currentDistance < CAUTION_ZONE) {
                    // Yellow - Caution zone (Red + Green)
                    GPIOF->DATA = (GPIOF->DATA & ~BLUE_PIN) | (RED_PIN | GREEN_PIN);
                    buzzerDelay = pdMS_TO_TICKS(300); // Medium beeping (300ms)
                } else {
                    // Green - Safe zone
                    GPIOF->DATA = (GPIOF->DATA & ~(RED_PIN | BLUE_PIN)) | GREEN_PIN;
                    buzzerDelay = pdMS_TO_TICKS(1000); // Slow beeping (1000ms)
                }
                // Release RGB LED mutex
                xSemaphoreGive(rgbLedMutex);
            }
            
            // Handle buzzer (toggle state based on frequency)
            if (xSemaphoreTake(buzzerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                buzzerState = !buzzerState;
                if (buzzerState) {
                    GPIOA->DATA |= BUZZER_PIN;  // Turn buzzer on
                } else {
                    GPIOA->DATA &= ~BUZZER_PIN; // Turn buzzer off
                }
                xSemaphoreGive(buzzerMutex);
            }
        } 
        else {
            // No active warnings - turn off all indicators
            if (xSemaphoreTake(rgbLedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                GPIOF->DATA &= ~(RED_PIN | BLUE_PIN | GREEN_PIN);
                xSemaphoreGive(rgbLedMutex);
            }
            
            if (xSemaphoreTake(buzzerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                GPIOA->DATA &= ~BUZZER_PIN;
                xSemaphoreGive(buzzerMutex);
                buzzerState = false;
            }
            
            // Standard delay when idle
            buzzerDelay = pdMS_TO_TICKS(200);
        }
        
        // Wait for the next alert cycle
        vTaskDelayUntil(&lastWakeTime, buzzerDelay);
    }
}

void DisplayUpdateTask(void *pvParameters) {
    char speedStr[16];
    char statusStr[16];
    char gearStr[8];
    char distanceStr[16];
    uint32_t currentSpeed = 0;
    uint32_t currentDistance = 0xFFFFFFFF; // Default to a large value
    bool prevIgnitionState = false;
    
    // Initialize previous ignition state
    prevIgnitionState = systemState.ignitionOn;
    
    // Initial LCD state - ensure it's off if ignition is off at startup
    if (!systemState.ignitionOn) {
        if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            LCD_Power_Off();
            xSemaphoreGive(lcdMutex);
        }
    } else {
        // Make sure LCD is initialized if ignition is on at startup
        if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            LCD_Power_On();
            xSemaphoreGive(lcdMutex);
        }
    }
    
    while(1) {
        // Check for ignition state change
        if (prevIgnitionState != systemState.ignitionOn) {
            if (systemState.ignitionOn) {
                // Ignition turned ON - power up LCD
                if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    LCD_Power_On();
                    xSemaphoreGive(lcdMutex);
                }
            } else {
                // Ignition turned OFF - power down LCD
                if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    LCD_Power_Off();
                    xSemaphoreGive(lcdMutex);
                }
            }
            prevIgnitionState = systemState.ignitionOn;
        }
        
        // Only update LCD if ignition is ON
        if (systemState.ignitionOn) {
            // Receive latest speed value from queue
            if (xQueuePeek(speedQueue, &currentSpeed, pdMS_TO_TICKS(10)) != pdTRUE) {
                // Queue receive timed out, use last known value
            }
            
            // Receive latest distance value from queue
            if (xQueuePeek(distanceQueue, &currentDistance, pdMS_TO_TICKS(10)) != pdTRUE) {
                // Queue receive timed out, use last known value or default
            }
            
            // Door open warning condition - door open while moving
            bool doorOpenWarning = systemState.driverDoorOpen && 
                                  systemState.ignitionOn && 
                                  currentSpeed > 0;
            
            // Take mutex to access LCD
            if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (LCD_Is_Powered()) {  // Double-check power state before updating
                    // Clear display
                    LCD_Clear();
                    
                    // If door is open while moving, show warning message on first line
                    if (doorOpenWarning) {
                        LCD_Set_Cursor(0, 0);
                        LCD_Print("DOOR OPEN!");
                        
                        // Still show speed on same line
                        sprintf(speedStr, "%3lu km/h", currentSpeed);
                        LCD_Set_Cursor(0, 10);
                        LCD_Print(speedStr);
                    } else {
                        // Normal speed display
                        LCD_Set_Cursor(0, 0);
                        LCD_Print("Speed:");
                        
                        // Convert speed to string and display
                        sprintf(speedStr, "%3lu km/h", currentSpeed);
                        LCD_Set_Cursor(0, 7);
                        LCD_Print(speedStr);
                    }
                    
                    // Display gear, ignition, and door/distance status on second line
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
                    
                    // Start with gear and ignition status
                    sprintf(statusStr, "%sON ", gearStr);
                    
                    // Add appropriate status based on current condition
                    if (systemState.parkingAssistActive && currentDistance != 0xFFFFFFFF) {
                        // Show distance when in reverse with parking assist
                        if (currentDistance < DANGER_ZONE) {
                            sprintf(distanceStr, "%lucm!", currentDistance);
                        } else if (currentDistance < CAUTION_ZONE) {
                            sprintf(distanceStr, "%lucm", currentDistance);
                        } else {
                            sprintf(distanceStr, "%lucm", currentDistance);
                        }
                        strcat(statusStr, distanceStr);
                    } 
                    else if (systemState.driverDoorOpen) {
                        // Show door open status
                        strcat(statusStr, "DOOR");
                    }
                    else if (systemState.doorsLocked) {
                        // Show locked status when door is closed and locked
                        strcat(statusStr, "LOCK");
                        // If manual override, add indicator
                        if (systemState.manualLockOverride) {
                            strcat(statusStr, "*");
                        }
                    } else {
                        // Show unlocked status when door is closed but unlocked
                        strcat(statusStr, "UNLK");
                        // If manual override, add indicator
                        if (systemState.manualLockOverride) {
                            strcat(statusStr, "*");
                        }
                    }
                    
                    LCD_Print(statusStr);
                }
                
                // Release the mutex
                xSemaphoreGive(lcdMutex);
            }
        }
        
        // Task delay (200ms)
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}