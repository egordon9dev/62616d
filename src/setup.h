#include <math.h>
#include "pid.h"

#ifndef SETUP_H
#define SETUP_H

#define LCD uart1

//------- Digital -------
#define DRIVE_L_ENC_T 1
#define DRIVE_L_ENC_B 2
#define DRIVE_R_ENC_B 3
#define DRIVE_R_ENC_T 4
#define US_IN 7
#define US_OUT 8

//------- Analog -------
#define MGL_POT 1
#define DRFB_POT 2
#define FB_POT 3
#define LT1 4
#define LT2 5

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

// fb angles
#define FB_MAX 145  // 152
#define FB_MIN 15   // -13
//#define FB_MIN_HOLD_ANGLE (drfbGet() <= 7 ? 20 : -999999)  // <------ keep these parentheses!
extern double fbUpP;
#define FB_UP_P0 146  // 154
#define FB_HALF_UP_POS (fbUpP - 38)
#define FB_UP_POS (limDouble(fbUpP + drfbGet() * 0.055, 0.0, 154.0))
#define FB_MID_POS 65
#define FB_FLIP_POS 78
#define FB_CLEAR_OF_STACK 105

// drfb angles
#define DRFB_FLIP_POS 5
#define DRFB_HORIZONTAL 55
#define DRFB_MAX_HOLD_ANGLE 999
#define DRFB_ENDPT_UP 124
#define DRFB_ENDPT_DOWN 0
#define DRFB_MAX1 119
#define DRFB_MAX2 121.5  // 124, 105
#define DRFB_MIN 10
#define DRFB_LDR_UP 66
#define DRFB_LDR_DOWN 38
#define DRFB_MGL_ACTIVE 19
#define DRFB_MGL_ACTIVE2 16

// mgl angles
#define MGL_MAX 109  // 120
#define MGL_MIN 6    // 1
#define MGL_DOWN_POS 113
#define MGL_MID_POS 75
#define MGL_UP_POS 0
#define MGL_VERT 41
#define MGL_ACTIVE2 47
// mgl max put down s0ft = 123.5
#define LT_LIGHT -1000
#define US_CONE_GRAB_DIST 13
#define US_MGL 9
/*
MG_MID - 10 -->  fb:    91
MG_MID      -->         69
*/

#define JOY_THRESHOLD 22

// height when stacking from loader when drive can be stationary
#define AUTO_STACK_STATIONARY 8

extern int DRIVE_DRIVE_MAX, DRIVE_TURN_MAX;
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
void setDRFBUnlim(int n);
void setFB(int n);
void setMGL(int n);
void stopMGL();
void resetMotors();

void setupSens();
void shutdownSens();
double drfbGet();
double fbGet();
double mglGet();
int eDLGet();
int eDRGet();
int lt1Get();
int lt2Get();
extern int prevDrv;
void opctrlDrive();
extern unsigned long pipeDriveT0;
bool pipeDrive();
extern bool curSetDownStack, autoStacking;
bool stackConeQ(int q);
bool liftConeQ(int q);
bool autoStack(int start, int end);

// zeroes drive encoders
void resetDriveEnc();

void printEnc();
void printEnc_pidDrive();
void printEnc_pidDriveShort();
void printEnc_pidDRFBFB();
void printEnc_all();
void setupLCD();
double usPredict();

double myAsin(double d);

void autoSelect();
typedef struct AutoSel {
    int stackH, zone, nAuton;
    bool leftSide, loaderSide;
} AutoSel;
extern AutoSel autoSel;
extern int ldrGrabI, ldrStackI;
extern int drfba[][2];
extern int drfbDownA[];
/*          Rahul               Erik
            joy1                joy2
j4:         ----                ----
j3:         drive               fb
j1:         turn                ----
j2:         ----                drfb
btn7L:      ----                canclSetDwnStk
btn7U:      ----                incFbUp
btn7R:      ----                resetFbUp
btn7D:      pipeDrive           decFbUp
btn8L:      mglAutoMid          canclSetDwnStk
btn8U:      mglManualUp         boostDrive
btn8R:      mglAutoUp           ----
btn8D:      mglManualDown       limitDrive
btn5U:      stopMglPid          fbAutoUp
btn5D:      setDownStack        fbAutoMid
btn6U:      mglAutoUp           ----
btn6D:      mglAutoDown         ----
*/
#endif  // SETUP_H
