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
#define A 0.7  // motor value change per ms
Slew fb_slew = {.a = A, .out = 0.0, .prevTime = 0};
Slew drfb_slew = {.a = A, .out = 0.0, .prevTime = 0};
Slew mgl_slew = {.a = A, .out = 0.0, .prevTime = 0};
Slew DL_slew = {.a = A, .out = 0.0, .prevTime = 0};
Slew DR_slew = {.a = A, .out = 0.0, .prevTime = 0}; /*
 Slew DL_slew_auto = {.a = 1.0, .out = 0.0, .prevTime = 0};
 Slew DR_slew_auto = {.a = 1.0, .out = 0.0, .prevTime = 0};*/
PidVars pidDef = {.doneTime = LONG_MAX, .DONE_ZONE = 10, .maxIntegral = DBL_MAX, .iActiveZone = 0.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 0.0, .ki = 0.0, .kd = 0.0, .prevTime = 0, .unwind = 0};
// weaker PID for opcontrol
PidVars drfb_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 5, .maxIntegral = 25, .iActiveZone = 10.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 3.0, .ki = 0.005, .kd = 250, .prevTime = 0, .unwind = 1};
PidVars fb_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 8, .maxIntegral = 35, .iActiveZone = 30.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 1.1, .ki = 0.005, .kd = 180.0, .prevTime = 0, .unwind = 1};  // .9, .025, 420
PidVars mgl_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 3, .maxIntegral = 15, .iActiveZone = 8.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 3.5, .ki = 0.0, .kd = 200.0, .prevTime = 0, .unwind = 0};
// agressive PID mostly for autonomous
PidVars drfb_pid_auto = {.doneTime = LONG_MAX, .DONE_ZONE = 5, .maxIntegral = 25, .iActiveZone = 10.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 5, .ki = 0.01, .kd = 400, .prevTime = 0, .unwind = 1};
PidVars fb_pid_auto = {.doneTime = LONG_MAX, .DONE_ZONE = 8, .maxIntegral = 35, .iActiveZone = 10.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 1.8, .ki = 0.01, .kd = 175.0, .prevTime = 0, .unwind = 5};
// Drive
#define DIA 40
#define DKP 0.48  // .32, .002, 185
#define DKI 0.0015
#define DKD 50.0
PidVars DL_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 50, .maxIntegral = 30, .iActiveZone = DIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = DKP, .ki = DKI, .kd = DKD, .prevTime = 0, .unwind = 0};
PidVars DR_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 50, .maxIntegral = 30, .iActiveZone = DIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = DKP, .ki = DKI, .kd = DKD, .prevTime = 0, .unwind = 0};
PidVars turn_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 1, .maxIntegral = 60, .iActiveZone = 9.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 4.5, .ki = 0.013, .kd = 370.0, .prevTime = 0, .unwind = 0};

void resetDone(PidVars *pidVars) { pidVars->doneTime = LONG_MAX; }

double updateSlew(Slew *slew, double in) {
    unsigned long dt = millis() - slew->prevTime;
    if (dt > 1000) dt = 0;
    slew->prevTime = millis();
    double maxInc = slew->a * dt;
    double vel = (double)(in - slew->out) / (double)dt;
    if (fabs(vel) < slew->a) {
        slew->out = in;
    } else {
        if (vel > 0) {
            slew->out += maxInc;
        } else {
            slew->out -= maxInc;
        }
    }
    return slew->out;
}
// proportional + integral + derivative control feedback
double updatePID(PidVars *pidVars) {
    unsigned long dt = millis() - pidVars->prevTime;
    if (dt > 1000) dt = 0;
    pidVars->prevTime = millis();
    // PROPORTIONAL
    double err = pidVars->target - pidVars->sensVal;
    double p = err * pidVars->kp;
    // INTEGRAL
    pidVars->errTot += err * dt;
    if (fabs(err) > pidVars->iActiveZone) pidVars->errTot = 0;
    double maxErrTot = pidVars->maxIntegral / pidVars->ki;
    if (pidVars->errTot > maxErrTot) pidVars->errTot = maxErrTot;
    if (pidVars->errTot < -maxErrTot) pidVars->errTot = -maxErrTot;
    if (((err > 0.0 && pidVars->errTot < 0.0) || (err < 0.0 && pidVars->errTot > 0.0)) && abs(err) > 0.001) {
        if (fabs(err) >= pidVars->unwind) {
            pidVars->errTot = 0.0;
            printf("UNWIND\n");
        }
    }
    if (fabs(pidVars->unwind) < 0.001 && fabs(err) < 0.001) {
        pidVars->errTot = 0.0;
        printf("UNWIND\n");
    }
    double i = pidVars->errTot * pidVars->ki;
    // DERIVATIVE
    double d = 0.0;
    if (dt != 0) { d = ((pidVars->prevSensVal - pidVars->sensVal) * pidVars->kd) / dt; }
    // done zone
    if (fabs(err) <= pidVars->DONE_ZONE && pidVars->doneTime > millis() && (int)(d * 10) == 0) {
        pidVars->doneTime = millis();
        printf("DONE\n");
    }
    // derivative action: slowing down
    /*if (fabs(d) > (fabs(p)) * 20.0) {
        pidVars->errTot = 0.0;
    }*/
    pidVars->prevErr = err;
    pidVars->prevSensVal = pidVars->sensVal;
    printf("p: %lf, i: %lf, d: %lf\t", p, i, d);
    // OUTPUT
    return p + i + d;
}
bool killPID(int d, int s, PidVars *p) {
    double err = p->target - p->sensVal;
    if (fabs(err) < d && fabs(err - p->prevErr) < s) { return true; }
    return false;
}
bool pidFB(double a, unsigned long wait, bool auton) {  // set chain-bar angle with PID
    PidVars *pid = auton ? &fb_pid_auto : &fb_pid;
    if (pid->doneTime + wait < millis()) {
        setFB(0);
        return true;
    }
    pid->target = a;
    pid->sensVal = fbGet();
    setFB(updatePID(pid));
    return false;
}
bool pidDRFB(double a, unsigned long wait, bool auton) {  // set arm angle with PID
    PidVars *pid = auton ? &drfb_pid_auto : &drfb_pid;
    if (pid->doneTime + wait < millis()) {
        setDRFB(0);
        return true;
    }
    pid->target = a;
    pid->sensVal = drfbGet();
    setDRFB(updatePID(pid));
    return false;
}
bool pidMGL(double a, unsigned long wait) {  // set chain-bar angle with PID
    if (mgl_pid.doneTime + wait < millis()) {
        setMGL(0);
        return true;
    }
    mgl_pid.target = a;
    mgl_pid.sensVal = mglGet();
    setMGL(updatePID(&mgl_pid));
    return false;
}
// angle settings for autonomous cone stacking
const int DRFB = 0, FB = 1;
int stackAngles[3][2] = {
    //	  ARM | CB
    {75, 110},
    {75, 120},
    {75, 130},
};
int returnAngle[] = {15, 40};

int getDRFB(int cone) { return stackAngles[cone][DRFB]; }
int getFB(int cone) { return stackAngles[cone][FB]; }
// return lift to pick up cones
int retStep = 0;
void startReturnLift(bool auton) {
    retStep = 0;
    resetFB(auton);
    resetDRFB(auton);
}
bool contReturnLift(bool auton, unsigned long wait) {
    printf("retStep: %d\t", retStep);
    switch (retStep) {
        case 0:
            if (pidFB(40, 100, auton)) {
                retStep++;
                resetFB(auton);
            }
            break;
        case 1:
            if (pidDRFB(returnAngle[DRFB], 100, auton)) {
                retStep++;
                resetDRFB(auton);
            }
            break;
        case 2:
            if (pidFB(returnAngle[FB], wait, auton) && pidDRFB(returnAngle[DRFB], wait, auton)) {
                retStep++;
                resetFB(auton);
                resetDRFB(auton);
            }
            break;
        default: return true;
    }
    return false;
}
// dist is in inches
bool pidDrive(double dist, unsigned long wait) {
    DL_pid.sensVal = eDLGet();
    DR_pid.sensVal = eDRGet();
    // 89 inches = 2457 ticks : 2457.0/89.0 = 27.6067
    DL_pid.target = dist * 27.6067;
    DR_pid.target = dist * 27.6067;
    setDL(updatePID(&DL_pid));
    setDR(updatePID(&DR_pid));
    if (DL_pid.doneTime + wait < millis() && DR_pid.doneTime + wait < millis()) {
        DL_pid.doneTime = LONG_MAX;
        DR_pid.doneTime = LONG_MAX;
        return true;
    }
    return false;
}
bool pidTurn(double angle, unsigned long wait) {
    turn_pid.sensVal = yawGet();
    turn_pid.target = angle;
    int n = updatePID(&turn_pid);
    setDL(-n);
    setDR(n);
    if (turn_pid.doneTime + wait < millis()) {
        turn_pid.doneTime = LONG_MAX;
        return true;
    }
    return false;
}
