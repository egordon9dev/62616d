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
//proportional control
double updateP(PidVars pidVars) {
     return (pidVars.target - pidVars.sensVal) * pidVars.kp;
}
//proportional + integral control
double updatePI(PidVars pidVars) {
     double err = pidVars.target - pidVars.sensVal;
     if(abs(err) < pidVars.INTEGRAL_ACTIVE_ZONE) {
          pidVars.errTot += err;
     } else {
          pidVars.errTot = 0;
     }
     double dt = (millis() - pidVars.prevTime) / 1000.0;
     double i = pidVars.errTot * pidVars.ki * dt;
     if(i > pidVars.maxIntegral) {
          i = pidVars.maxIntegral;
     }
     return (err * pidVars.kp) + i;
}
//proportional + derivative control
double updatePD(PidVars pidVars) {
     double err = pidVars.target - pidVars.sensVal;
     pidVars.prevErr = err;
     double dt = (millis() - pidVars.prevTime) / 1000.0;
     pidVars.prevTime = millis();
     return (err * pidVars.kp) + ((err - pidVars.prevErr) * pidVars.kd)/dt;
}
//proportional + integral + derivative control
double updatePID(PidVars pidVars) {
     double err = pidVars.target - pidVars.sensVal;
     if(abs(err) < pidVars.INTEGRAL_ACTIVE_ZONE) {
          pidVars.errTot += err;
     } else {
          pidVars.errTot = 0;
     }
     double dt = (millis() - pidVars.prevTime) / 1000.0;
     pidVars.prevTime = millis();
     double i = pidVars.errTot * pidVars.ki * dt;
     if(i > pidVars.maxIntegral) {
          i = pidVars.maxIntegral;
     }
     double d = ((err - pidVars.prevErr) * pidVars.kd) / dt;
     pidVars.prevErr = err;
     return (err * pidVars.kp) + i + d;
}
