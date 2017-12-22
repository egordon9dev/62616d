#include "pid.h"
#include <math.h>
#include "API.h"
#include "setup.h"

/*
     -------- PidVars --------
     must be set by user:
     kp, ki, kd, INTEGRAL_ACTIVE_ZONE, DERIVATIVE_ACTIVE_ZONE, maxIntegral,
   target, sensVal

     automatically handled by functions:
     prevError, errTot, prevTime
     -------------------------
*/
#define A 0.7
LPF fb_lpf = {.a = A, .out = 0.0};
LPF drfb_lpf = {.a = A, .out = 0.0};
LPF mgl_lpf = {.a = A, .out = 0.0};
LPF DL_lpf = {.a = A, .out = 0.0};
LPF DR_lpf = {.a = A, .out = 0.0};
LPF DL_lpf_auto = {.a = 0.95, .out = 0.0};
LPF DR_lpf_auto = {.a = 0.95, .out = 0.0};
PidVars pidDef = {.doneTime = LONG_MAX, .DONE_ZONE = 10, .maxIntegral = DBL_MAX, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 0.0, .ki = 0.0, .kd = 0.0, .prevTime = 0, .unwind = 0};
PidVars drfb_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 3, .maxIntegral = 50, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 0.95, .ki = 0.035, .kd = 400, .prevTime = 0, .unwind = 0};
PidVars fb_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 3, .maxIntegral = 35, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 0.9, .ki = 0.025, .kd = 420.0, .prevTime = 0, .unwind = 0};
PidVars mgl_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 3, .maxIntegral = 15, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 5.0, .ki = 0.0, .kd = 400.0, .prevTime = 0, .unwind = 0};
#define dkp 0.55
#define dki 0.006
#define dkd 200.0
PidVars DL_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 15, .maxIntegral = 30, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = dkp, .ki = dki, .kd = dkd, .prevTime = 0, .unwind = 0};
PidVars DR_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 15, .maxIntegral = 30, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = dkp, .ki = dki, .kd = dkd, .prevTime = 0, .unwind = 0};
PidVars turn_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 1, .maxIntegral = 40, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 11.0, .ki = 0.0175, .kd = 2700.0, .prevTime = 0, .unwind = 0};
void resetDone(PidVars *pidVars) { pidVars->doneTime = LONG_MAX; }

double updateLPF(LPF *lpf, double in) {
    lpf->out = lpf->out * lpf->a + in * (1 - lpf->a);
    return lpf->out;
}
// proportional + integral + derivative control feedback
double updatePID(PidVars *pidVars) {
    unsigned long dt = millis() - pidVars->prevTime;
    pidVars->prevTime = millis();
    // PROPORTIONAL
    double err = pidVars->target - pidVars->sensVal;
    double p = err * pidVars->kp;
    // INTEGRAL
    pidVars->errTot += err * dt;
    double maxErrTot = pidVars->maxIntegral / pidVars->ki;
    if (pidVars->errTot > maxErrTot) pidVars->errTot = maxErrTot;
    if (pidVars->errTot < -maxErrTot) pidVars->errTot = -maxErrTot;
    if (((err > 0.0 && pidVars->errTot < 0.0) || (err < 0.0 && pidVars->errTot > 0.0)) && abs(err) > 0.001) {
        if (fabs(err) >= pidVars->unwind) {
            pidVars->errTot = 0.0;
            printf("UNWIND\n");
        }
    }
    double i = pidVars->errTot * pidVars->ki;
    // DERIVATIVE
    double d = 0.0;
    if (dt != 0) {
        d = ((pidVars->prevSensVal - pidVars->sensVal) * pidVars->kd) / dt;
    }
    // done zone
    if (fabs(err) < pidVars->DONE_ZONE && pidVars->doneTime > millis() && (int)(d * 10) == 0) {
        pidVars->doneTime = millis();
        printf("DONE\n");
    }
    // derivative action: slowing down
    if (fabs(d) > fabs(p) * 20.0) {
        pidVars->errTot = 0.0;
    }
    pidVars->prevErr = err;
    pidVars->prevSensVal = pidVars->sensVal;
    printf("p: %lf, i: %lf, d: %lf\t", p, i, d);
    // OUTPUT
    return p + i + d;
}
int killPID(int d, int s, PidVars *p) {
    double err = p->target - p->sensVal;
    if (fabs(err) < d && fabs(err - p->prevErr) < s) {
        return 1;
    }
    return 0;
}
int pidFB(double a, int wait) {  // set chain-bar angle with PID
    fb_pid.target = a;
    fb_pid.sensVal = fbGet();
    setFB(updatePID(&fb_pid));
    if (fb_pid.doneTime + wait < millis()) {
        fb_pid.doneTime = LONG_MAX;
        return 1;
    }
    return 0;
}
int pidDRFB(double a, int wait) {  // set arm angle with PID
    drfb_pid.target = a;
    drfb_pid.sensVal = drfbGet();
    setDRFB(updatePID(&drfb_pid));
    if (drfb_pid.doneTime + wait < millis()) {
        drfb_pid.doneTime = LONG_MAX;
        return 1;
    }
    return 0;
}
int pidMGL(double a, int wait) {  // set chain-bar angle with PID
    mgl_pid.target = a;
    mgl_pid.sensVal = mglGet();
    setMGL(updatePID(&mgl_pid));
    if (mgl_pid.doneTime + wait < millis()) {
        mgl_pid.doneTime = LONG_MAX;
        return 1;
    }
    return 0;
}
// angle settings for autonomous cone stacking
const int DRFB = 0, FB = 1;
int stackAngles[][2] = {
    //	  ARM | CB
    {75, 110},
    {75, 120},
    {75, 130},
};
int returnAngle[] = {0, FB_MIN_CUT};
// set chain bar and arm with PID to stack given cone
int stack(int cone) {
    pidFB(stackAngles[cone][FB], 0);
    if (fb_pid.doneTime < millis()) {
        pidDRFB(stackAngles[cone][DRFB], 0);
    }
    int wait = 200;
    if (drfb_pid.doneTime + wait < millis() && fb_pid.doneTime + wait < millis()) {
        drfb_pid.doneTime = LONG_MAX;
        fb_pid.doneTime = LONG_MAX;
        return 1;
    }
    return 0;
}
int getDRFB(int cone) { return stackAngles[cone][DRFB]; }
int getFB(int cone) { return stackAngles[cone][FB]; }
// return lift to pick up cones
void returnLift() {
    if (pidFB(40, 0)) {
        if (pidDRFB(returnAngle[DRFB], 0)) {
            pidFB(returnAngle[FB], 0);
        }
    }
}
// dist is in inches
void pidDrive(double dist) {
    DL_pid.sensVal = eDLGet();
    DR_pid.sensVal = eDRGet();
    // 89 inches = 2457 ticks : 2457.0/89.0 = 27.6067
    DL_pid.target = dist * 27.6067;
    DR_pid.target = dist * 27.6067;
    setDL(updatePID(&DL_pid));
    setDR(updatePID(&DR_pid));
}
void pidTurn(double angle) {
    turn_pid.sensVal = yawGet();
    turn_pid.target = angle;
    int n = updatePID(&turn_pid);
    setDL(-n);
    setDR(n);
}
