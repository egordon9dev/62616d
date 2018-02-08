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

#define POW_EXP 2
#define MGL_POT 1
#define DRFB_POT 3
#define FB_POT 4

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

#define FB_MAX 130                                                // 153
#define FB_MIN 0                                                  // -13
#define FB_MIN_HOLD_ANGLE (drfbGet() <= 6 ? 26 - drfbGet() : 20)  // <------ keep these parentheses!
#define FB_MID_POS 45
#define FB_UP_POS (120 + drfbGet() * drfbGet() * 0.0002)
#define DRFB_MAX_HOLD_ANGLE 999
#define DRFB_MAX1 95
#define DRFB_MAX2 102  // 124, 105
#define DRFB_MIN 30
#define MGL_MAX 114  // 120
#define MGL_MIN 4    // 1
#define MGL_DOWN_POS 121
#define MGL_MID_POS 88
#define MGL_UP_POS 1

#define DRIVE_TURN_MAX 85
#define DRIVE_DRIVE_MAX 110
#define DRFB_MAX 100  // 85

#define DRIVE_TICKS_PER_IN 27.6067
#define DRIVE_TICKS_PER_DEG 3.58

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

void setupSens();
int drfbGet();
int fbGet();
int mglGet();
int eDLGet();
int eDRGet();

// zeroes drive encoders
void resetDriveEnc();

void printEnc();
void printEnc_pidDrive();
void printEnc_pidDRFBFB();
void printEnc_all();
void setupLCD();

extern int autonMode, autonModeLen;
void autoSelect();

#define DM 2
/*

-----   DM 2: "Rahul, Erik"   -----
            joy1                joy2
j4:         ----                ----
j3:         drive               ----
j1:         turn                ----
j2:         ----                drfb
btn7L:      ----                return
btn7U:      ----                fbManUp
btn7R:      ----                stopRollers
btn7D:      ----                fbManDown
btn8L:      mglAutoMid          ----
btn8U:      mglManualUp         ----
btn8R:      mglAutoUp           ----
btn8D:      mglManualDown       ----
btn5U:      mglAutoMid          fbAutoUp
btn5D:      mglAutoMid          fbAutoDown
btn6U:      mglAutoUp           rollerIn
btn6D:      mglAutoDown         rollerOut
*/
#endif  // SETUP_H
