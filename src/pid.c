#include "API.h"
#include "pid.h"

/*
     -------- PidVars --------
     must be set by user:
     kp, ki, kd, INTEGRAL_ACTIVE_ZONE, maxIntegral, target, sensVal

     automatically handled by functions:
     prevError, errTot, prevTime
     -------------------------
*/
PidVars pidDef = {
     .INTEGRAL_ACTIVE_ZONE = DBL_MAX, .maxIntegral = DBL_MAX,
     .target = 0.0,
     .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0,
     .kp = 0.0, .ki = 0.0, .kd = 0.0,
     .prevTime = 0
};
//proportional control feedback
double updateP(PidVars pidVars) {
     return (pidVars.target - pidVars.sensVal) * pidVars.kp;
}
//get integral component
double getI(PidVars pidVars, double dt) {
     double err = pidVars.target - pidVars.sensVal;
     if(abs(err) < pidVars.INTEGRAL_ACTIVE_ZONE) {
          pidVars.errTot += err;
     } else {
          pidVars.errTot = 0;
     }
     double i = pidVars.errTot * dt * pidVars.ki;
     if(i > pidVars.maxIntegral) {
          i = pidVars.maxIntegral;
     }
     return i;
}
//get derivative component
double getD(PidVars pidVars, double dt) {
     double err = pidVars.target - pidVars.sensVal;
     double d = ((err - pidVars.prevErr) * pidVars.kd) / dt;
     pidVars.prevErr = err;
     return d;
}
//proportional + integral control feedback
double updatePI(PidVars pidVars) {
     double dt = (millis() - pidVars.prevTime) / 1000.0;
     pidVars.prevTime = millis();
     return updateP(pidVars) + getI(pidVars, dt);
}
//proportional + derivative control feedback
double updatePD(PidVars pidVars) {
     double dt = (millis() - pidVars.prevTime) / 1000.0;
     pidVars.prevTime = millis();
     return updateP(pidVars) + getD(pidVars, dt);
}
//proportional + integral + derivative control feedback
double updatePID(PidVars pidVars) {
     double dt = (millis() - pidVars.prevTime) / 1000.0;
     pidVars.prevTime = millis();
     return updateP(pidVars) + getI(pidVars, dt) + getD(pidVars, dt);
}
