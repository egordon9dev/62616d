#include "pid.h"

#ifndef SETUP_H
#define SETUP_H

//------- Digital -------
#define DRIVE_R_ENC_BOT 12
#define DRIVE_R_ENC_TOP 11
#define CHAIN_ENC_TOP 7
#define CHAIN_ENC_BOT 8
#define ARM_ENC_TOP 6
#define ARM_ENC_BOT 5
#define DRIVE_L_ENC_TOP 4
#define DRIVE_L_ENC_BOT 2

#define MGL_LIM 1

//------- MOTOR -------
//CORTEX
#define M7_8 2
#define M0_1 3

//PWR EX
#define M2 4 // A
#define M11 5 // B
#define M6 6 // C
#define M5 7 // D

//CORTEX
#define M3_4 8
#define M9_10 9

#define CB_MAX 315
#define CB_MIN 79

//motors
void limMotorVal(int* n);
int getLimMotorVal(int n);
void setDL(int n);
void setDR(int n);
void setArm(int n);
void setCB(int n);
void setClaw(int n);
void setMGL(int n);
void resetMotors();


//encoders
void setupEnc();
extern int stackAngles[][2];
extern int returnAngle[];
extern const int ARM, CB;
int eArmGet();
int eCBGet();
int eDLGet();
int eDRGet();

//zeroes drive encoders
void resetDriveEnc();

//resets drive in preparation for using PID with drive
void resetDrive(PidVars* DL_pid, PidVars* DR_pid, PidVars* DLturn_pid, PidVars* DRturn_pid);

#endif //SETUP_H
