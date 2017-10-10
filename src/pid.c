#include "API.h"
#include "pid.h"

/*
     -------- PidVars --------
     must be set by user:
     kp, ki, kd, INTEGRAL_ACTIVE_ZONE, DERIVATIVE_ACTIVE_ZONE, maxIntegral, target, sensVal

     automatically handled by functions:
     prevError, errTot, prevTime
     -------------------------
*/
#define MASSIVE 2000111000
PidVars pidDef = {
     .doneTime = 0, .DONE_ZONE = 10,
     .DERIVATIVE_ACTIVE_ZONE = DBL_MAX,
     .INTEGRAL_ACTIVE_ZONE = DBL_MAX, .maxIntegral = DBL_MAX,
     .target = 0.0,
     .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0,
     .kp = 0.0, .ki = 0.0, .kd = 0.0,
     .prevTime = MASSIVE
};
PidVars arm_pid = {
     .doneTime = 0, .DONE_ZONE = 10,
     .DERIVATIVE_ACTIVE_ZONE = 20,
     .INTEGRAL_ACTIVE_ZONE = 20, .maxIntegral = 50,
     .target = 0.0,
     .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0,
     .kp = 7.0, .ki = 0.01, .kd = 300.0,
     .prevTime = MASSIVE
};
PidVars cb_pid = {
     .doneTime = 0, .DONE_ZONE = 10,
     .DERIVATIVE_ACTIVE_ZONE = 60,
     .INTEGRAL_ACTIVE_ZONE = 20, .maxIntegral = 50,
     .target = 0.0,
     .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0,
     .kp = 2.55, .ki = 0.0, .kd = 220.0,
     .prevTime = MASSIVE
};
PidVars DL_pid = {
     .doneTime = 0, .DONE_ZONE = 75,
     .DERIVATIVE_ACTIVE_ZONE = 300,
     .INTEGRAL_ACTIVE_ZONE = 1000, .maxIntegral = 50,
     .target = 0.0,
     .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0,
     .kp = 0.35, .ki = 0.0, .kd = 50.0,
     .prevTime = MASSIVE
};
PidVars DR_pid = {
     .doneTime = 0, .DONE_ZONE = 75,
     .DERIVATIVE_ACTIVE_ZONE = 300,
     .INTEGRAL_ACTIVE_ZONE = 1000, .maxIntegral = 50,
     .target = 0.0,
     .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0,
     .kp = 0.35, .ki = 0.0, .kd = 50.0,
     .prevTime = MASSIVE
};
PidVars DLturn_pid = {
     .doneTime = 0, .DONE_ZONE = 45,
     .DERIVATIVE_ACTIVE_ZONE = 200,
     .INTEGRAL_ACTIVE_ZONE = 400, .maxIntegral = 50,
     .target = 0.0,
     .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0,
     .kp = 1.24, .ki = 0.0, .kd = 90.0,
     .prevTime = MASSIVE
};
PidVars DRturn_pid = {
     .doneTime = 0, .DONE_ZONE = 45,
     .DERIVATIVE_ACTIVE_ZONE = 200,
     .INTEGRAL_ACTIVE_ZONE = 400, .maxIntegral = 50,
     .target = 0.0,
     .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0,
     .kp = 1.24, .ki = 0.0, .kd = 90.0,
     .prevTime = MASSIVE
};
//proportional control feedback
double updateP(PidVars* pidVars) {
     double err = pidVars->target - pidVars->sensVal;
     if(err < pidVars->DONE_ZONE && pidVars->doneTime < millis() - 500) {
          pidVars->doneTime = millis();
     }
     return err * pidVars->kp;
}
//get integral component
double getI(PidVars* pidVars, double dt) {
     double err = pidVars->target - pidVars->sensVal;
     if(abs(err) < pidVars->INTEGRAL_ACTIVE_ZONE) {
          pidVars->errTot += err;
     } else {
          pidVars->errTot = 0;
     }
     double i = pidVars->errTot * dt * pidVars->ki;
     if(i > pidVars->maxIntegral) {
          i = pidVars->maxIntegral;
     }
     return i;
}
//get derivative component
double getD(PidVars* pidVars, double dt) {
     double err = pidVars->target - pidVars->sensVal;
     if(abs(err) > pidVars->DERIVATIVE_ACTIVE_ZONE) {
          return 0.0;
     }
     double d = ((err - pidVars->prevErr) * pidVars->kd) / dt;
     pidVars->prevErr = err;
     return d;
}
//proportional + integral control feedback
double updatePI(PidVars* pidVars) {
     double dt = millis() - pidVars->prevTime;
     pidVars->prevTime = millis();
     return updateP(pidVars) + getI(pidVars, dt);
}
//proportional + derivative control feedback
double updatePD(PidVars* pidVars) {
     double dt = millis() - pidVars->prevTime;
     pidVars->prevTime = millis();
     return updateP(pidVars) + getD(pidVars, dt);
}
//proportional + integral + derivative control feedback
double updatePID(PidVars* pidVars) {
     double dt = millis() - pidVars->prevTime;
     pidVars->prevTime = millis();
     return updateP(pidVars) + getI(pidVars, dt) + getD(pidVars, dt);
}
