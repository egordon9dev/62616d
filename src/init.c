/** @file init.c
 * @brief File for initialization code
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */
#include "main.h"

#include "ports.h"

static Encoder eArm, eCB, eDL, eDR;
int eArmGet() {
     //90 - 34 = 56
     //56 + (90-77)  = 56 + 13 = 69
     return 69 - encoderGet(eArm);
}
int eCBGet() {
     // 315 to 79
     return encoderGet(eCB) + 291;
}
int eDLGet() {
     return encoderGet(eDL);
}
int eDRGet() {
     return encoderGet(eDR);
}
void resetDriveEnc() {
     encoderReset(eDL);
     encoderReset(eDR);
}
void resetMotors() {
     for(int i = 1; i <= 10; i++) {
          motorSet(i, 0);
     }
}
/*
 * The purpose of this function is solely to set the default pin modes (pinMode()) and port
 * states (digitalWrite()) of limit switches, push buttons, and solenoids. It can also safely
 * configure a UART port (usartOpen()) but cannot set up an LCD (lcdInit()).
 */
void initializeIO() {
     pinMode(MGL_LIM, INPUT);
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
     eCB = encoderInit(CHAIN_ENC_TOP, CHAIN_ENC_BOT, false);
     eDL = encoderInit(DRIVE_L_ENC_TOP, DRIVE_L_ENC_BOT, false);
     eDR = encoderInit(DRIVE_R_ENC_TOP, DRIVE_R_ENC_BOT, false);
     encoderReset(eArm);
     encoderReset(eCB);
     encoderReset(eDL);
     encoderReset(eDR);
}
