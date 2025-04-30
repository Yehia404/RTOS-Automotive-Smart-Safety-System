#ifndef __TASKS_H__
#define __TASKS_H__

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Task handle declarations
extern TaskHandle_t speedSensingHandle;
extern TaskHandle_t displayUpdateHandle;
extern TaskHandle_t switchMonitorHandle;
extern TaskHandle_t ultrasonicTaskHandle;

// Semaphore/mutex for LCD access
extern SemaphoreHandle_t lcdMutex;

// Queue for speed updates
extern QueueHandle_t speedQueue;

// Gear position enumeration
typedef enum
{
    GEAR_PARK = 0,
    GEAR_DRIVE,
    GEAR_REVERSE,
    GEAR_UNKNOWN
} GearPosition_t;

// Pin definitions
#define IGNITION_PIN (1 << 4)     // PF4 for ignition switch
#define LOCK_BTN_PIN (1 << 0)     // PB0 for lock button
#define UNLOCK_BTN_PIN (1 << 1)   // PB1 for unlock button
#define GEAR_PARK_PIN (1 << 0)    // PE0 for Park gear switch
#define GEAR_DRIVE_PIN (1 << 1)   // PE1 for Drive gear switch
#define GEAR_REVERSE_PIN (1 << 2) // PE2 for Reverse gear switch

// Pin definitions for ultrasonic sensor (updated)
#define TRIG_PIN (1 << 2) // PA2 for Trigger pin
#define ECHO_PIN (1 << 3) // PA3 for Echo pin

// Pin definition for buzzer (updated)
#define BUZZER_PIN (1 << 4) // PA4 for Buzzer

// Pin definitions for RGB LED
#define RED_PIN (1 << 1)   // PF1 for Red LED
#define BLUE_PIN (1 << 2)  // PF2 for Blue LED
#define GREEN_PIN (1 << 3) // PF3 for Green LED

// Distance zones (in cm)
#define DANGER_ZONE 30
#define CAUTION_ZONE 100

// Shared system state
typedef struct
{
    uint32_t currentSpeed;
    bool doorsLocked;
    bool ignitionOn;
    bool manualLockOverride;
    GearPosition_t gearPosition; // Added for gear shifter
    uint32_t distanceCm;         // Distance measurement in cm
    bool parkingAssistActive;    // Whether parking assist is active
} SystemState_t;

extern SystemState_t systemState;

// Task initialization function
void tasks_init(void);

// GPIO initialization for switches
void switches_init(void);

// GPIO initialization for ultrasonic, RGB LED, and buzzer
void ultrasonic_system_init(void);

// Task functions
void SpeedSensingTask(void *pvParameters);
void DisplayUpdateTask(void *pvParameters);
void SwitchMonitorTask(void *pvParameters);
void UltrasonicTask(void *pvParameters);

#endif // __TASKS_H__