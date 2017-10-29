#ifndef PID_H
#define PID_H
#include <stdbool.h>
//----- variables to be entered into P, PD, or PID function -----//
typedef struct PidVars {
    double INTEGRAL_ACTIVE_ZONE, DERIVATIVE_ACTIVE_ZONE, DONE_ZONE, maxIntegral, target, sensVal, prevErr, errTot, kp, ki, kd;
    unsigned long prevTime, doneTime;
} PidVars;

extern PidVars pidDef, arm_pid, cb_pid, DL_pid, DR_pid, DLturn_pid, DRturn_pid;
#define LONG_MAX 2147483647
#define DBL_MAX 999999999.999999
void resetDone(PidVars *pidVars);

//----- proportional control feedback -----//
double updateP(PidVars *pidVars);

//----- proportional + integral control feedback -----//

double updatePI(PidVars *pidVars);

//----- proportional + derivative control feedback -----//
double updatePD(PidVars *pidVars);

//----- proportional + integral + derivative control feedback -----//
double updatePID(PidVars *pidVars);

/*
    returns 1 if pid (p) is closer than distance (d) and slower than speed (s)
    otherwise returns 0
*/
int killPID(int d, int s, PidVars *p);

// set chain-bar angle with PID
void pidCB(PidVars *cb_pid, double a);

// set arm angle with PID
void pidArm(PidVars *arm_pid, double a);

// set chain bar and arm with PID to stack given cone
int stack(PidVars *arm_pid, PidVars *cb_pid, int cone);
int getArm(int cone);
int getCB(int cone);
// return lift to pick up cones
void returnLift(PidVars *arm_pid, PidVars *cb_pid);

// if turning, dist is in degrees
// if not turning, dist is in inches
void pidDrive(double dist, PidVars *left, PidVars *right, bool turning);

#endif
