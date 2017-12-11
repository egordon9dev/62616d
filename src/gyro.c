/*
 * PROS Analog Gyro Library for VEX Yaw-Rate 1000dps LY3100ALH gyro
 *
 * Copyright (c) 2011-2016, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "periph.h"
#include "task.h"
// Default gyro multiplier
// Calculation: 1.1 mV/dps = 1.365 quid/dps = 0.0007326007... dpms/quid ~= 196>>18
#define GYRO_MULTIPLIER_DEFAULT 196
// Integration interval in milliseconds
/*
turn 360 cw then 360 ccw
dt      dz      err
1       2       7
1       4       6

drive straight 3 ft, then back
1       4       2,3
2       4       1,2
*/
#define DT 1  // 2
// Rate noise elimination threshold
#define RATE_NOISE_LIMIT 2  // 4
#define SCALE_FACTOR 0.01
// Sensor status storage
Analog_TypeDef _analogState[BOARD_NR_ADC_PINS];
MyAnalog_TypeDef _myAnalogState[BOARD_NR_ADC_PINS];
static volatile unsigned int gyroCount = 0;

void myAnalogCalibrate(unsigned char channel);
double myAnalogReadCalibrated(unsigned char channel);

// Gets the current gyro angle in degrees
int gyroGet(Gyro g) {
    Analog_TypeDef *gyro = (Analog_TypeDef *)g;
    if (gyro) return (int)(gyro->value + 0x80) >> 8;
    return 0;
}
// Gets the current gyro angle in degrees
int myGyroGet(Gyro g) {
    if (g) return (int)(((MyAnalog_TypeDef *)g)->value + 0x80) >> 8;
    return 0;
}
// Gyro integration routine
static INLINE void gyroIntegrate(uint32_t port) {
    Analog_TypeDef *gyro = &_analogState[port];
    // LSLed by 4 as offset
    int32_t reading = (int32_t)analogReadCalibratedHR(port + 1);
    // Multiplier is (0.0007...<<18) dpms * DT ms * (reading<<4) quid = degrees<<22
    // So we need to get from LSL22 to LSL8 = LSR14
    int32_t d = ((int32_t)(gyro->lastValue) * DT * reading + 0x2000) >> 14;
    if (d < -RATE_NOISE_LIMIT || d > RATE_NOISE_LIMIT) gyro->value += d;
}
static INLINE void myGyroIntegrate(uint32_t port) {
    double d = (int32_t)(DT * ((int)myAnalogReadCalibrated(port + 1) << 4) * 196 + 0x2000) >> 14;  // SCALE_FACTOR;
    if (d < -RATE_NOISE_LIMIT || d > RATE_NOISE_LIMIT) _myAnalogState[port].value += d;
}

// Gyro integration task which integrates all gyros on robot
static void gyroIntegrateTask(void *ignore) {
    clock_t now = timeLowRes();
    while (gyroCount > 0) {
        for (uint32_t i = 0; i < BOARD_NR_ADC_PINS; i++)
            // If active, integrate it
            if (_analogState[i].flags & (uint8_t)0x02) gyroIntegrate(i);
        taskDelayUntil(&now, DT);
    }
}
// Gyro integration task which integrates all gyros on robot
static void myGyroIntegrateTask(void *ignore) {
    clock_t now = timeLowRes();
    while (gyroCount > 0) {
        for (uint32_t i = 0; i < BOARD_NR_ADC_PINS; i++)
            // If active, integrate it
            if (_myAnalogState[i].flags & (uint8_t)0x02) myGyroIntegrate(i);
        taskDelayUntil(&now, DT);
    }
}
// Initialize the gyro - call in initialize()
Gyro gyroInit(unsigned char port, unsigned short multiplier) {
    // Initialize gyro
    if (multiplier == 0) multiplier = GYRO_MULTIPLIER_DEFAULT;
    port--;
    if (port < 8) {
        unsigned int gc = gyroCount;
        // Calibrate the port using standard PROS API
        analogCalibrate(port + 1);
        // Check to ensure that the gyro is properly plugged in
        /*if (_analogState[port].calibValue > 512) {*/
        __disable_irq();
        // Mark in use, non reversed (gyros cannot be reversed)
        _analogState[port].flags = (uint8_t)0x02;
        _analogState[port].value = 0;
        // Store multiplier (can be independent per gyro to account for differences)
        _analogState[port].lastValue = multiplier;
        gyroCount = gc + 1;
        __enable_irq();
        if (gc == 0)
            // Start integrator
            taskCreate(gyroIntegrateTask, TASK_MINIMAL_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT + 1);
        return (Gyro)(&_analogState[port]);
        /*}*/
    }
    return NULL;
}
// Initialize the gyro - call in initialize()
Gyro myGyroInit(unsigned char port) {
    // Initialize gyro
    port--;
    if (port < 8) {
        unsigned int gc = gyroCount;
        // Calibrate the port using standard PROS API
        myAnalogCalibrate(port + 1);
        // Check to ensure that the gyro is properly plugged in
        /*if (_analogState[port].calibValue > 512) {*/
        __disable_irq();
        // Mark in use, non reversed (gyros cannot be reversed)
        _myAnalogState[port].flags = (uint8_t)0x02;
        _myAnalogState[port].value = 0.0;
        gyroCount = gc + 1;
        __enable_irq();
        if (gc == 0)
            // Start integrator
            taskCreate(myGyroIntegrateTask, TASK_MINIMAL_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT + 1);
        return (Gyro)(&_myAnalogState[port]);
        /*}*/
    }
    return NULL;
}

// Resets the gyro angle to zero
void gyroReset(Gyro g) {
    Analog_TypeDef *gyro = (Analog_TypeDef *)g;
    if (gyro) gyro->value = 0;
}

// Stop gyro and freeze the value
void gyroShutdown(Gyro g) {
    Analog_TypeDef *gyro = (Analog_TypeDef *)g;
    if (gyro) {
        __disable_irq();
        // Mark inactive
        gyro->flags = (uint8_t)0x00;
        gyroCount--;
        __enable_irq();
    }
}
