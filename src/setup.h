#include <math.h>
#include "pid.h"

#ifndef SETUP_H
#define SETUP_H

#define LCD uart1

//------- Digital -------
#define DRIVE_L_ENC_TOP 1
#define DRIVE_L_ENC_BOT 2
#define DRIVE_R_ENC_TOP 3
#define DRIVE_R_ENC_BOT 4

//------- Analog -------
#define MGL_POT 1
#define DRFB_POT 2
#define FB_POT 3
#define LT0 4
#define LT1 5
#define LT2 6
#define LT3 7
#define LT4 8

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

#define FB_MAX 130                                         // 153
#define FB_MIN 0                                           // -13
#define FB_MIN_HOLD_ANGLE (drfbGet() <= 7 ? 20 : -999999)  // <------ keep these parentheses!
extern double fbUpP;
#define FB_UP_P0 120.0
#define FB_UP_POS (fbUpP + drfbGet() * 0.1 + (drfbGet() > 108 ? 5 : 0))
#define FB_MID_POS 43
#define DRFB_MAX_HOLD_ANGLE 999
#define DRFB_MAX1 112
#define DRFB_MAX2 119  // 124, 105
#define DRFB_MIN 20
#define MGL_MAX 114  // 120
#define MGL_MIN 6    // 1
#define MGL_DOWN_POS 121
#define MGL_MID_POS 88
#define MGL_UP_POS 1
#define LT_LIGHT -1000

#define DRIVE_TURN_MAX 100
#define DRIVE_DRIVE_MAX 110
#define DRFB_MAX 127

#define DRIVE_TICKS_PER_IN 27.6067
#define DRIVE_TICKS_PER_DEG 3.58

// motors
void limMotorVal(int *n);
int getLimMotorVal(int n);
int limInt(int n, int min, int max);
double limDouble(double n, double min, double max);
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
void opctrlDrive();

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
j3:         drive               fb
j1:         turn                ----
j2:         ----                drfb
btn7L:      ----                ----
btn7U:      ----                incFbUp
btn7R:      ----                ----
btn7D:      setDownStack        decFbUp
btn8L:      mglAutoMid          stopRollers
btn8U:      mglManualUp         stopRollers
btn8R:      mglAutoUp           stopRollers
btn8D:      mglManualDown       stopRollers
btn5U:      mglAutoMid          fbAutoUp
btn5D:      setDownStack        fbAutoMid
btn6U:      mglAutoUp           rollerIn
btn6D:      mglAutoDown         rollerOut
*/
#endif  // SETUP_H
