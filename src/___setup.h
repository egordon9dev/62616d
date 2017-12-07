#include "pid.h"

#ifndef SETUP_H
#define SETUP_H

#define LCD uart1

//------- Digital -------
#define DRIVE_L_ENC_TOP 1
#define DRIVE_L_ENC_BOT 2
#define DRIVE_R_ENC_BOT 3
#define DRIVE_R_ENC_TOP 4

#define MGL_LIM1 8
#define MGL_LIM2 9

//------- Analog -------

#define POW_EXP 1
#define ARML_POT 5
#define DRFB_POT 3
#define FB_POT 4

//------- MOTOR -------
// CORTEX
#define M7_8 2
#define M0 3

// PWR EX
#define M1_2 4
#define M6 6
#define M4_5 7
#define M11 5

// CORTEX
#define M3 8
#define M9_10 9

#define FB_MAX 315
#define FB_MIN 79

// motors
void limMotorVal(int *n);
int getLimMotorVal(int n);
void setDL(int n);
void setDR(int n);
void setDRFB(int n);
void setFB(int n);
void setClaw(int n);
void setMGL(int n);
void resetMotors();

// encoders
void setupEnc();
double drfbGet();
double fbGet();
int eDLGet();
int eDRGet();

// zeroes drive encoders
void resetDriveEnc();

// resets drive in preparation for using PID with drive
void resetDrive(PidVars *DL_pid, PidVars *DR_pid, PidVars *DLturn_pid, PidVars *DRturn_pid);

void printEnc();
void printEnc_pidDrive(PidVars *DL_pid, PidVars *DR_pid, PidVars *DLturn_pid, PidVars *DRturn_pid);
void printEnc_PidDRFBFB(PidVars *drfb_pid, PidVars *fb_pid);
bool mglBut();
void setupLCD();

// auton
#define nAutons 5
#define nSkills 1
extern int autonMode;
void autoSelect();

#endif  // SETUP_H