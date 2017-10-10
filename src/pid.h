#ifndef PID_H
#define PID_H
#include <float.h>

//----- variables to be entered into P, PD, or PID function -----//
typedef struct PidVars {
     double INTEGRAL_ACTIVE_ZONE, DERIVATIVE_ACTIVE_ZONE, DONE_ZONE, maxIntegral, target, sensVal, prevErr, errTot, kp, ki, kd;
     unsigned long prevTime, doneTime;
} PidVars;

extern PidVars pidDef, arm_pid, cb_pid, DL_pid, DR_pid, DLturn_pid, DRturn_pid;
#define MASSIVE 2000111000
//----- proportional control feedback -----//
double updateP(PidVars* pidVars);

//----- proportional + integral control feedback -----//

double updatePI(PidVars* pidVars);

//----- proportional + derivative control feedback -----//
double updatePD(PidVars* pidVars);

//----- proportional + integral + derivative control feedback -----//
double updatePID(PidVars* pidVars);

#endif
