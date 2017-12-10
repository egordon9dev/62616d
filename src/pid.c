#include "pid.h"
#include "API.h"
#include "setup.h"
#include <math.h>

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
LPF fb_lpf = { .a = A,.out = 0.0 };
LPF drfb_lpf = { .a = A,.out = 0.0 };
LPF mgl_lpf = { .a = A,.out = 0.0 };
LPF DL_lpf = { .a = A,.out = 0.0 };
LPF DR_lpf = { .a = A,.out = 0.0 };
LPF DL_lpf_auto = { .a = 0.98,.out = 0.0 };
LPF DR_lpf_auto = { .a = 0.98,.out = 0.0 };
PidVars pidDef = {.doneTime = LONG_MAX, .DONE_ZONE = 10, .maxIntegral = DBL_MAX, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 0.0, .ki = 0.0, .kd = 0.0, .prevTime = 0};
PidVars drfb_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 10, .maxIntegral = 50, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 7.0, .ki = 0.02, .kd = 600.0, .prevTime = 0};
PidVars fb_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 10, .maxIntegral = 50, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 2.55, .ki = 0.0, .kd = 220.0, .prevTime = 0};
PidVars mgl_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 4, .maxIntegral = 15, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 5.0, .ki = 0.0, .kd = 400.0, .prevTime = 0};
#define dkp 0.7
#define dki 0.003
#define dkd 75.0
PidVars DL_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 50, .maxIntegral = 50, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = dkp, .ki = dki, .kd = dkd, .prevTime = 0};
PidVars DR_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 50, .maxIntegral = 50, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = dkp, .ki = dki, .kd = dkd, .prevTime = 0};
#define dtkp 0.62
#define dtki 0.0025  // 0.0025
#define dtkd 98.0
PidVars DLturn_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 15, .maxIntegral = 50, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = dtkp, .ki = dtki, .kd = dtkd, .prevTime = 0};
PidVars DRturn_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 15, .maxIntegral = 50, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = dtkp, .ki = dtki, .kd = dtkd, .prevTime = 0};
void resetDone(PidVars *pidVars) { pidVars->doneTime = LONG_MAX; }

double updateLPF(LPF* lpf, double in) {
	lpf->out = lpf->out * lpf->a + in * (1 - lpf->a);
	return lpf->out;
}
// proportional + integral + derivative control feedback
double updatePID(PidVars *pidVars) {
	unsigned long dt = millis() - pidVars->prevTime;
	pidVars->prevTime = millis();
	//PROPORTIONAL
	double err = pidVars->target - pidVars->sensVal;
	double p = err * pidVars->kp;
	if (fabs(err) < pidVars->DONE_ZONE && pidVars->doneTime > millis()) {
		pidVars->doneTime = millis();
		printf("DONE\n");
	}
	//INTEGRAL
	pidVars->errTot += err * dt;
	double maxErrTot = pidVars->maxIntegral / pidVars->ki;
	if (pidVars->errTot > maxErrTot) pidVars->errTot = maxErrTot;
	if (pidVars->errTot < -maxErrTot) pidVars->errTot = -maxErrTot;
	if ((err > 0 && pidVars->errTot < 0) || (err < 0 && pidVars->errTot > 0)) {
		if (fabs(err) >= pidVars->unwind) {
			pidVars->errTot = 0;
		}
	}
	double i = pidVars->errTot * pidVars->ki;
	//DERIVATIVE
	double d = 0.0;
	if (dt != 0) {
		d = ((pidVars->prevSensVal - pidVars->sensVal) * pidVars->kd) / dt;
	}
	pidVars->prevErr = err;
	pidVars->prevSensVal = pidVars->sensVal;
	//OUTPUT
	return p + i + d;
}
int killPID(int d, int s, PidVars *p) {
    double err = p->target - p->sensVal;
    if (fabs(err) < d && fabs(err - p->prevErr) < s) {
        return 1;
    }
    return 0;
}
void pidFB(PidVars *fb_pid, double a) {  // set chain-bar angle with PID
    fb_pid->target = a;
    fb_pid->sensVal = fbGet();
    setFB(updatePID(fb_pid));
}
void pidDRFB(PidVars *drfb_pid, double a) {  // set arm angle with PID
    drfb_pid->target = a;
    drfb_pid->sensVal = drfbGet();
    setDRFB(updatePID(drfb_pid));
}
void pidMGL(PidVars *mgl_pid, double a) {  // set chain-bar angle with PID
	mgl_pid->target = a;
	mgl_pid->sensVal = mglGet();
	setMGL(updatePID(mgl_pid));
}
// angle settings for autonomous cone stacking
const int ARM = 0, CB = 1;
int stackAngles[][2] = {
    //	  ARM | CB
    {75, 110},
    {75, 120},
    {75, 130},
};
int returnAngle[] = {73, 305};
// set chain bar and arm with PID to stack given cone
int stack(PidVars *drfb_pid, PidVars *fb_pid, int cone) {
    pidFB(fb_pid, stackAngles[cone][CB]);
    if (fb_pid->doneTime < millis()) {
        pidDRFB(drfb_pid, stackAngles[cone][ARM]);
    }
    int wait = 200;
    if (drfb_pid->doneTime + wait < millis() && fb_pid->doneTime + wait < millis()) {
        drfb_pid->doneTime = LONG_MAX;
        fb_pid->doneTime = LONG_MAX;
        return 1;
    }
    return 0;
}
int getDRFB(int cone) { return stackAngles[cone][ARM]; }
int getFB(int cone) { return stackAngles[cone][CB]; }
// return lift to pick up cones
void returnLift(PidVars *drfb_pid, PidVars *fb_pid) {
    pidFB(fb_pid, returnAngle[CB]);
    pidDRFB(drfb_pid, returnAngle[ARM]);
}
// if turning, dist is in degrees
// if not turning, dist is in inches
void pidDrive(double dist, PidVars *left, PidVars *right, bool turning) {
    left->sensVal = eDLGet();
    right->sensVal = eDRGet();
    if (turning) {
        left->target = dist * -3.5610;
        int leftPow = updatePID(left);
        limMotorVal(&leftPow);
        setDL(leftPow);
        right->target = dist * 3.5610;
        int rightPow = updatePID(right);
        limMotorVal(&rightPow);
        setDR(rightPow);
    } else {
        // 89 inches = 2457 ticks : 2457.0/89.0 = 27.6067
        left->target = dist * 27.6067;
        int leftPow = updatePID(left);
        limMotorVal(&leftPow);
        setDL(leftPow);
        right->target = dist * 27.6067;
        int rightPow = updatePID(right);
        limMotorVal(&rightPow);
        setDR(rightPow);
    }
}
