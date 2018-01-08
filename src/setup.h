#include "pid.h"

#ifndef SETUP_H
#define SETUP_H

#define LCD uart1

//------- Digital -------
#define DRIVE_L_ENC_TOP 1
#define DRIVE_L_ENC_BOT 2
#define DRIVE_R_ENC_BOT 3
#define DRIVE_R_ENC_TOP 4

//------- Analog -------

#define POW_EXP 1
#define MGL_POT 2
#define DRFB_POT 3
#define FB_POT 4
#define GYRO 5

//------- MOTOR -------
// CORTEX
#define M1_2 2
#define M3 3
#define M11 4

// PWR EX
#define M6_7 5
#define M8_9 6
#define M10 7

// CORTEX
#define M0 8
#define M4_5 9

#define FB_MAX 140                                         // 153
#define FB_MIN 0                                           // -13
#define FB_MIN_CUT (drfbGet() <= 6 ? 26 - drfbGet() : 20)  // <------ keep these parentheses!
#define DRFB_MAX_CUT 999
#define DRFB_MAX 98  // 121
#define DRFB_MIN 30
#define MGL_MAX 105  // 120
#define MGL_MIN 10   // 1

// motors
void limMotorVal(int *n);
int getLimMotorVal(int n);
void setDL(int n);
void setDR(int n);
void setDRFB(int n);
void setFB(int n);
void setRollers(int n);
void setMGL(int n);
void resetMotors();

int yawGet();
void setupSens();
int drfbGet();
int fbGet();
int mglGet();
int eDLGet();
int eDRGet();

// zeroes drive encoders
void resetDriveEnc();

// resets drive in preparation for using PID with drive
void resetDrive();
void resetMGL();
void resetFB(bool auton);
void resetDRFB(bool auton);

void printEnc();
void printEnc_pidDrive();
void printEnc_pidDRFBFB();
void setupLCD();

// auton
#define nAutons 5
#define nSkills 1
extern int autonMode;
extern bool progSkills;
void autoSelect();

#endif  // SETUP_H
