#ifndef PID_H
#define PID_H
#include <float.h>

//----- variables to be entered into P, PD, or PID function -----//
typedef struct PidVars {
     double INTEGRAL_ACTIVE_ZONE, maxIntegral, target, sensVal, prevErr, errTot, kp, ki, kd;
     unsigned long prevTime;
} PidVars;

//----- default initialized PID values -----//
extern PidVars pidDef;

//----- proportional control -----//
double updateP(PidVars pidVars);

//----- proportional + integral control -----//
double updatePI(PidVars pidVars);

//----- proportional + derivative control -----//
double updatePD(PidVars pidVars);

//----- proportional + integral + derivative control -----//
double updatePID(PidVars pidVars);

#endif
