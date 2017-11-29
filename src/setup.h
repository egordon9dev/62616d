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
#define ARMR_POT 3
#define CB_POT 4

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

#define CB_MAX 315
#define CB_MIN 79

// motors
void limMotorVal(int *n);
int getLimMotorVal(int n);
void setDL(int n);
void setDR(int n);
void setArm(int n);
void setCB(int n);
void setClaw(int n);
void setMGL(int n);
void resetMotors();

// encoders
void setupEnc();
double armGet();
double cbGet();
int eDLGet();
int eDRGet();

// zeroes drive encoders
void resetDriveEnc();

// resets drive in preparation for using PID with drive
void resetDrive(PidVars *DL_pid, PidVars *DR_pid, PidVars *DLturn_pid, PidVars *DRturn_pid);

void printEnc();
void printEnc_pidDrive(PidVars *DL_pid, PidVars *DR_pid, PidVars *DLturn_pid, PidVars *DRturn_pid);
void printEnc_pidArmCB(PidVars *arm_pid, PidVars *cb_pid);
bool mglBut();
void setupLCD();

// auton
#define nAutons 5
#define nSkills 1
extern int autonMode;
void autoSelect();

#endif  // SETUP_H
