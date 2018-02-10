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
#define A 1  // motor value change per ms           0.7
Slew fb_slew = {.a = A, .out = 0.0, .prevTime = 0};
Slew drfb_slew = {.a = A, .out = 0.0, .prevTime = 0};
Slew mgl_slew = {.a = A, .out = 0.0, .prevTime = 0};
Slew DL_slew = {.a = A, .out = 0.0, .prevTime = 0};
Slew DR_slew = {.a = A, .out = 0.0, .prevTime = 0};
Slew roller_slew = {.a = A, .out = 0.0, .prevTime = 0}; /*
 Slew DL_slew_auto = {.a = 1.0, .out = 0.0, .prevTime = 0};
 Slew DR_slew_auto = {.a = 1.0, .out = 0.0, .prevTime = 0};*/
PidVars pidDef = {.doneTime = LONG_MAX, .DONE_ZONE = 10, .maxIntegral = DBL_MAX, .iActiveZone = 0.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 0.0, .ki = 0.0, .kd = 0.0, .prevTime = 0, .unwind = 0};
// weaker PID for opcontrol
PidVars drfb_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 5, .maxIntegral = 25, .iActiveZone = 10.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 3.0, .ki = 0.005, .kd = 250, .prevTime = 0, .unwind = 1};
PidVars mgl_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 3, .maxIntegral = 15, .iActiveZone = 8.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 4.0, .ki = 0.0, .kd = 200.0, .prevTime = 0, .unwind = 0};
// agressive PID mostly for autonomous
PidVars drfb_pid_auto = {.doneTime = LONG_MAX, .DONE_ZONE = 5, .maxIntegral = 25, .iActiveZone = 10.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 5.3, .ki = 0.005, .kd = 280, .prevTime = 0, .unwind = 1};
PidVars fb_pid_auto = {.doneTime = LONG_MAX, .DONE_ZONE = 8, .maxIntegral = 35, .iActiveZone = 10.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 4.0, .ki = 0.01, .kd = 250.0, .prevTime = 0, .unwind = 5};  // 1.8, 0.01, 175.0
// Drive
#define DIA 50
#define DDZ 15
#define DKP 0.44    // .44
#define DKI 0.0007  //.0007, .0015
#define DKD 40.0
PidVars DL_pid = {.doneTime = LONG_MAX, .DONE_ZONE = DDZ, .maxIntegral = 40, .iActiveZone = DIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = DKP, .ki = DKI, .kd = DKD, .prevTime = 0, .unwind = 0};
PidVars DR_pid = {.doneTime = LONG_MAX, .DONE_ZONE = DDZ, .maxIntegral = 40, .iActiveZone = DIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = DKP, .ki = DKI, .kd = DKD, .prevTime = 0, .unwind = 0};
PidVars driveCurve_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 4, .maxIntegral = 40, .iActiveZone = DIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 3, .ki = 0.003, .kd = 100.0, .prevTime = 0, .unwind = 0};
#define TIA 40
#define TDZ 10
#define TKP 1.65   // 1.35, 1.2
#define TKI 0.002  //.002, .003
#define TKD 120.0
PidVars DLturn_pid = {.doneTime = LONG_MAX, .DONE_ZONE = TDZ, .maxIntegral = 45, .iActiveZone = TIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = TKP, .ki = TKI, .kd = TKD, .prevTime = 0, .unwind = 0};
PidVars DRturn_pid = {.doneTime = LONG_MAX, .DONE_ZONE = TDZ, .maxIntegral = 45, .iActiveZone = TIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = TKP, .ki = TKI, .kd = TKD, .prevTime = 0, .unwind = 0};
PidVars turnCurve_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 2, .maxIntegral = 20, .iActiveZone = TIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 3.5, .ki = 0.0035, .kd = 160.0, .prevTime = 0, .unwind = 0};
// 7.2,.013,560

double updateSlew(Slew *slew, double in) {
    unsigned long dt = millis() - slew->prevTime;
    if (dt > 1000) dt = 0;
    slew->prevTime = millis();
    double maxInc = slew->a * dt;
    double vel = (double)(in - slew->out) / (double)dt;
    if (fabs(vel) < slew->a) {
        slew->out = in;
    } else if (in >= 0 && slew->out < 0) {
        slew->out = 0;
    } else if (in <= 0 && slew->out > 0) {
        slew->out = 0;
    } else if (vel > 0) {
        slew->out += maxInc;
    } else {
        slew->out -= maxInc;
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
    // DERIVATIVE
    double d = 0.0;
    if (dt != 0) { d = ((pidVars->prevSensVal - pidVars->sensVal) * pidVars->kd) / dt; }
    // INTEGRAL
    pidVars->errTot += err * dt;
    if (fabs(err) > pidVars->iActiveZone) pidVars->errTot = 0;
    if (fabs(d) > 10) {
        double maxErrTot = pidVars->maxIntegral / pidVars->ki;
        if (pidVars->errTot > maxErrTot) pidVars->errTot = maxErrTot;
        if (pidVars->errTot < -maxErrTot) pidVars->errTot = -maxErrTot;
    }
    if ((err > 0.0 && pidVars->errTot < 0.0) || (err < 0.0 && pidVars->errTot > 0.0) || abs(err) < 0.001) {
        if (fabs(err) - pidVars->unwind > -0.001) {
            pidVars->errTot = 0.0;
            // printf("UNWIND\n");
        }
    }
    if (fabs(pidVars->unwind) < 0.001 && fabs(err) < 0.001) {
        pidVars->errTot = 0.0;
        // printf("UNWIND\n");
    }
    double i = pidVars->errTot * pidVars->ki;
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
    // printf("p: %lf, i: %lf, d: %lf\t", p, i, d);
    // OUTPUT
    return p + i + d;
}
bool killPID(int d, int s, PidVars *p) {
    double err = p->target - p->sensVal;
    if (fabs(err) < d && fabs(err - p->prevErr) < s) { return true; }
    return false;
}
bool pidFB(double a, unsigned long wait, bool auton) {  // set fb angle with PID
    if (fb_pid_auto.doneTime + wait < millis()) {
        setFB(0);
        return true;
    }
    fb_pid_auto.target = a;
    fb_pid_auto.sensVal = fbGet();
    setFB(updatePID(&fb_pid_auto));
    return false;
}
bool pidDRFB(double a, unsigned long wait, bool auton) {  // set drfb angle with PID
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
bool pidMGL(double a, unsigned long wait) {  // set mgl angle with PID
    if (mgl_pid.doneTime + wait < millis()) {
        setMGL(0);
        return true;
    }
    mgl_pid.target = a;
    mgl_pid.sensVal = mglGet();
    setMGL(updatePID(&mgl_pid));
    return false;
}

// dist is in inches
bool pidDrive(double dist, unsigned long wait) {
    if (DL_pid.doneTime + wait < millis() && DR_pid.doneTime + wait < millis()) {
        setDL(0);
        setDR(0);
        return true;
    }
    DL_pid.sensVal = eDLGet();
    DR_pid.sensVal = eDRGet();
    // 89 inches = 2457 ticks : 2457.0/89.0 = 27.6067
    DL_pid.target = dist * DRIVE_TICKS_PER_IN;
    DR_pid.target = dist * DRIVE_TICKS_PER_IN;

    driveCurve_pid.sensVal = eDRGet() - eDLGet();
    driveCurve_pid.target = 0.0;
    double curve = updatePID(&driveCurve_pid) * 0.5 * (fabs((DL_pid.target - DL_pid.sensVal) / DL_pid.target) + fabs((DR_pid.target - DR_pid.sensVal) / DR_pid.target));
    setDL(updatePID(&DL_pid) - curve);  // *
    setDR(updatePID(&DR_pid) + curve);  // /
    return false;
}
bool pidTurn(double angle, unsigned long wait) {
    if (DLturn_pid.doneTime + wait < millis() && DRturn_pid.doneTime + wait < millis()) {
        setDL(0);
        setDR(0);
        return true;
    }
    turnCurve_pid.sensVal = eDRGet() + eDLGet();
    turnCurve_pid.target = 0.0;
    double curve = updatePID(&turnCurve_pid) * 0.5 * (fabs((DLturn_pid.target - DLturn_pid.sensVal) / DLturn_pid.target) + fabs((DRturn_pid.target - DRturn_pid.sensVal) / DRturn_pid.target));

    DLturn_pid.sensVal = eDLGet();
    DLturn_pid.target = -angle * DRIVE_TICKS_PER_DEG;
    DRturn_pid.sensVal = eDRGet();
    DRturn_pid.target = angle * DRIVE_TICKS_PER_DEG;
    setDL(updatePID(&DLturn_pid) + curve);
    setDR(updatePID(&DRturn_pid) + curve);
    return false;
}
