#ifndef PORTS_H
#define PORTS_H

//------- Digital -------
#define DRIVE_L_ENC_BOT 12
#define DRIVE_L_ENC_TOP 11
#define CHAIN_ENC_TOP 7
#define CHAIN_ENC_BOT 8
#define ARM_ENC_TOP 6
#define ARM_ENC_BOT 5
#define DRIVE_R_ENC_BOT 3
#define DRIVE_R_ENC_TOP 4

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

int eArmGet();
int eCBGet();
int eDRGet();
int eDLGet();

#endif
