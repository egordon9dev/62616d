/** @file init.c
 * @brief File for initialization code
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */
#include "main.h"

#define CHAIN_ENC_TOP 11
#define CHAIN_ENC_BOT 12
#define ARM_ENC_TOP 4
#define ARM_ENC_BOT 3

static Encoder eArm, eChain;
int eArmGet() {
     //90 - 34 = 56
     return 56 - encoderGet(eArm);
}
int eChainGet() {
     return encoderGet(eChain) + 180;
}
/*
 * The purpose of this function is solely to set the default pin modes (pinMode()) and port
 * states (digitalWrite()) of limit switches, push buttons, and solenoids. It can also safely
 * configure a UART port (usartOpen()) but cannot set up an LCD (lcdInit()).
 */
void initializeIO() {
     pinMode(2, OUTPUT);
     digitalWrite(2, LOW);
}
/*
 * Runs user initialization code. This function will be started in its own task with the default
 * priority and stack size once when the robot is starting up. It is possible that the VEXnet
 * communication link may not be fully established at this time, so reading from the VEX
 * Joystick may fail.
 *
 * This function should initialize most sensors (gyro, encoders, ultrasonics), LCDs, global
 * variables, and IMEs.
 *
 * This function must exit relatively promptly, or the operatorControl() and autonomous() tasks
 * will not start. An autonomous mode selection menu like the pre_auton() in other environments
 * can be implemented in this task if desired.
 */
void initialize() {
     eArm = encoderInit(ARM_ENC_TOP, ARM_ENC_BOT, false);
     eChain = encoderInit(CHAIN_ENC_TOP, CHAIN_ENC_BOT, false);
     encoderReset(eArm);
     encoderReset(eChain);
}
