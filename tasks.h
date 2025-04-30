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

// Shared system state
typedef struct
{
    uint32_t currentSpeed;
    bool doorsLocked;
    bool ignitionOn;
    bool manualLockOverride;
    GearPosition_t gearPosition; // Added for gear shifter
} SystemState_t;

extern SystemState_t systemState;

// Task initialization function
void tasks_init(void);

// GPIO initialization for switches
void switches_init(void);

// Task functions
void SpeedSensingTask(void *pvParameters);
void DisplayUpdateTask(void *pvParameters);
void SwitchMonitorTask(void *pvParameters);

#endif // __TASKS_H__