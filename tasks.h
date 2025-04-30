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
extern TaskHandle_t alertHandlerHandle;

// Semaphore/mutex declarations
extern SemaphoreHandle_t lcdMutex;
extern SemaphoreHandle_t rgbLedMutex;
extern SemaphoreHandle_t buzzerMutex;

// Queue declarations
extern QueueHandle_t speedQueue;
extern QueueHandle_t distanceQueue;

// Gear position enumeration
typedef enum
{
    GEAR_PARK = 0,
    GEAR_DRIVE,
    GEAR_REVERSE,
    GEAR_UNKNOWN
} GearPosition_t;

// Pin definitions for switches and buttons
#define IGNITION_PIN (1 << 4)     // PF4 for ignition switch
#define LOCK_BTN_PIN (1 << 0)     // PB0 for lock button
#define UNLOCK_BTN_PIN (1 << 1)   // PB1 for unlock button
#define GEAR_PARK_PIN (1 << 0)    // PE0 for Park gear switch
#define GEAR_DRIVE_PIN (1 << 1)   // PE1 for Drive gear switch
#define GEAR_REVERSE_PIN (1 << 2) // PE2 for Reverse gear switch
#define DOOR_SWITCH_PIN (1 << 5)  // PA5 for driver door switch (endstop limit switch)

// Pin definitions for ultrasonic sensor
#define TRIG_PIN (1 << 2) // PA2 for Trigger pin
#define ECHO_PIN (1 << 3) // PA3 for Echo pin

// Pin definition for buzzer
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
    uint32_t currentSpeed;       // Current vehicle speed
    bool doorsLocked;            // Door lock status
    bool ignitionOn;             // Ignition state
    bool manualLockOverride;     // Manual door lock override flag
    GearPosition_t gearPosition; // Current gear position
    bool parkingAssistActive;    // Whether parking assist is active
    bool driverDoorOpen;         // Driver door state (open/closed)
} SystemState_t;

extern SystemState_t systemState;

// Task initialization function
void tasks_init(void);

// GPIO initialization for switches and inputs
void switches_init(void);

// GPIO initialization for ultrasonic, RGB LED, and buzzer
void ultrasonic_system_init(void);

// Task functions
void SpeedSensingTask(void *pvParameters);
void DisplayUpdateTask(void *pvParameters);
void SwitchMonitorTask(void *pvParameters);
void UltrasonicTask(void *pvParameters);
void AlertHandlerTask(void *pvParameters);

#endif // __TASKS_H__