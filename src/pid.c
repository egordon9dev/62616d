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
Slew fb_slew = {.a = 2 * A, .out = 0.0, .prevTime = 0};
Slew drfb_slew = {.a = A, .out = 0.0, .prevTime = 0};
Slew mgl_slew = {.a = 2 * A, .out = 0.0, .prevTime = 0};
Slew DL_slew = {.a = 2 * A, .out = 0.0, .prevTime = 0};
Slew DR_slew = {.a = 2 * A, .out = 0.0, .prevTime = 0};
Slew roller_slew = {.a = A, .out = 0.0, .prevTime = 0}; /*
 Slew DL_slew_auto = {.a = 1.0, .out = 0.0, .prevTime = 0};
 Slew DR_slew_auto = {.a = 1.0, .out = 0.0, .prevTime = 0};*/
PidVars pidDef = {.doneTime = LONG_MAX, .DONE_ZONE = 10, .maxIntegral = DBL_MAX, .iActiveZone = 0.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 0.0, .ki = 0.0, .kd = 0.0, .prevTime = 0, .unwind = 0};
// weaker PID for opcontrol
PidVars drfb_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 5, .maxIntegral = 25, .iActiveZone = 10.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 3.0, .ki = 0.005, .kd = 250, .prevTime = 0, .unwind = 1};
PidVars fb_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 8, .maxIntegral = 35, .iActiveZone = 10.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 2.5, .ki = 0.003, .kd = 170.0, .prevTime = 0, .unwind = 3};
PidVars mgl_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 3, .maxIntegral = 15, .iActiveZone = 8.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 5.0, .ki = 0.0, .kd = 70.0, .prevTime = 0, .unwind = 0};
// agressive PID mostly for autonomous
PidVars drfb_pid_auto = {.doneTime = LONG_MAX, .DONE_ZONE = 5, .maxIntegral = 30, .iActiveZone = 10.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 7.0, .ki = 0.007, .kd = 600, .prevTime = 0, .unwind = 3};
PidVars fb_pid_auto = {.doneTime = LONG_MAX, .DONE_ZONE = 8, .maxIntegral = 30, .iActiveZone = 10.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 5.5, .ki = 0.000, .kd = 500.0, .prevTime = 0, .unwind = 0};  // 1.8, 0.01, 175.0

// Drive
#define DIA 50
#define DDZ 12
#define DKP 0.52   // .44
#define DKI 0.004  //.0007, .0015
#define DKD 55.0   // 40
PidVars DL_pid = {.doneTime = LONG_MAX, .DONE_ZONE = DDZ, .maxIntegral = 40, .iActiveZone = DIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = DKP, .ki = DKI, .kd = DKD, .prevTime = 0, .unwind = 0};
PidVars DR_pid = {.doneTime = LONG_MAX, .DONE_ZONE = DDZ, .maxIntegral = 40, .iActiveZone = DIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = DKP, .ki = DKI, .kd = DKD, .prevTime = 0, .unwind = 0};
PidVars driveCurve_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 4, .maxIntegral = 40, .iActiveZone = DIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 6, .ki = 0.006, .kd = 0.0, .prevTime = 0, .unwind = 0};  // 7, 0, 0
#define TIA 40
#define TDZ 7
#define TKP 1.45   // 1.35, 1.2
#define TKI 0.006  //.002, .003
#define TKD 160.0
PidVars DLturn_pid = {.doneTime = LONG_MAX, .DONE_ZONE = TDZ, .maxIntegral = 45, .iActiveZone = TIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = TKP, .ki = TKI, .kd = TKD, .prevTime = 0, .unwind = 0};
PidVars DRturn_pid = {.doneTime = LONG_MAX, .DONE_ZONE = TDZ, .maxIntegral = 45, .iActiveZone = TIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = TKP, .ki = TKI, .kd = TKD, .prevTime = 0, .unwind = 0};
PidVars turnCurve_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 2, .maxIntegral = 20, .iActiveZone = TIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 1.0, .ki = 0.001, .kd = 00.0, .prevTime = 0, .unwind = 0};  // 2, .002, 200
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
    if (fabs(err) <= pidVars->DONE_ZONE && pidVars->doneTime > millis()) {
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
    PidVars *pid = auton ? &fb_pid_auto : &fb_pid;
    if (pid->doneTime + wait < millis()) {
        setFB(0);
        return true;
    }
    pid->target = a;
    pid->sensVal = fbGet();
    int pwr = updatePID(pid);
    if (pid->sensVal > FB_UP_POS) pwr -= 25;
    setFB(pwr);
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
double ltdKi = 0.5, ltdKp = 12.0, ltdInt = 0.0;
// dist is in inches
void setDownStack() {
    int i = 0;
    mgl_pid.doneTime = LONG_MAX;
    double drfba0 = drfbGet();
    int t0 = millis();
    while (true) {
        int j = 0;
        pidDRFB(drfba0, 0, true);
        if (i == j++) {
            pidFB(FB_MID_POS, 999999, true);
            if (pidMGL(MGL_DOWN_POS, 0) || millis() - t0 > 2000) {
                t0 = millis();
                i++;
            }
        } else if (i == j++) {
            setRollers(-50);
            pidFB(FB_UP_POS, 999999, true);
            pidMGL(MGL_DOWN_POS, 999999);
            if (fbGet() > FB_UP_POS - 20 || millis() - t0 > 1000) {
                i++;
                setRollers(0);
            }
        } else if (i == j++) {
            break;
        }
        opctrlDrive();
        delay(20);
    }
    setFB(0);
    setDRFB(0);
}
bool pidDrive(double dist, unsigned long wait, bool lineTrack) {
    if (DL_pid.doneTime + wait < millis() && DR_pid.doneTime + wait < millis()) {
        setDL(0);
        setDR(0);
        return true;
    }
    if (lineTrack) {
        double eAvg = (eDLGet() + eDRGet()) * 0.5;
        DL_pid.sensVal = eAvg;
        DR_pid.sensVal = eAvg;
    } else {
        DL_pid.sensVal = eDLGet();
        DR_pid.sensVal = eDRGet();
    }
    // 89 inches = 2457 ticks : 2457.0/89.0 = 27.6067
    DL_pid.target = dist * DRIVE_TICKS_PER_IN;
    DR_pid.target = dist * DRIVE_TICKS_PER_IN;
    int powerL = updatePID(&DL_pid);
    int powerR = updatePID(&DR_pid);
    limMotorVal(&powerL);
    limMotorVal(&powerR);
    if (lineTrack) {
        bool b0 = analogReadCalibrated(LT0) < LT_LIGHT;
        bool b1 = analogReadCalibrated(LT1) < LT_LIGHT;
        bool b2 = analogReadCalibrated(LT2) < LT_LIGHT;
        bool b3 = analogReadCalibrated(LT3) < LT_LIGHT;
        bool b4 = analogReadCalibrated(LT4) < LT_LIGHT;
        if (b2) {
            if (b1 && !b3) {
                if (powerR + powerL > 0) {
                    ltdInt += 0.5;
                } else {
                    ltdInt -= 0.5;
                }
                powerL *= 0.5 / ltdKp;
            } else if (b3 && !b1) {
                if (powerR + powerL > 0) {
                    ltdInt -= 0.5;
                } else {
                    ltdInt += 0.5;
                }
                powerR *= 0.5 / ltdKp;
            }
        } else {
            if (b0) {
                if (powerR + powerL > 0) {
                    ltdInt += 2.0;
                } else {
                    ltdInt -= 2.0;
                }
                powerL = 0;
            } else if (b1) {
                if (powerL + powerR > 0) {
                    ltdInt += 1.0;
                } else {
                    ltdInt -= 1.0;
                }
                powerL *= 0.25 / ltdKp;
            } else if (b4) {
                if (powerL + powerR > 0) {
                    ltdInt -= 2.0;
                } else {
                    ltdInt += 2.0;
                }
                powerR = 0;
            } else if (b3) {
                if (powerL + powerR > 0) {
                    ltdInt -= 1.0;
                } else {
                    ltdInt += 1.0;
                }
                powerR *= 0.25 / ltdKp;
            }
        }

        if (!b0 && !b1 && !b2 && !b3 && !b4) lineTrack = false;
    }
    if (!lineTrack) {
        driveCurve_pid.sensVal = eDRGet() - eDLGet();
        driveCurve_pid.target = 0.0;
        int curve = updatePID(&driveCurve_pid) * 0.5 * (fabs((DL_pid.target - DL_pid.sensVal) / DL_pid.target) + fabs((DR_pid.target - DR_pid.sensVal) / DR_pid.target));
        double curveInfluence = 0.5;
        if (abs(powerR) > abs(powerL)) {
            if (powerL > 0) {
                curve = limInt(curve, -powerL * curveInfluence, powerL * curveInfluence);
            } else {
                curve = limInt(curve, powerL * curveInfluence, -powerL * curveInfluence);
            }
        } else {
            if (powerR > 0) {
                curve = limInt(curve, -powerR * curveInfluence, powerR * curveInfluence);
            } else {
                curve = limInt(curve, powerR * curveInfluence, -powerR * curveInfluence);
            }
        }
        powerL -= curve;
        powerR += curve;
    }
    limMotorVal(&powerL);
    limMotorVal(&powerR);
    setDL(powerL);
    setDR(powerR);
    return false;
}
bool pidDumbDrive(double dist, unsigned long wait) {
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
    int powerL = updatePID(&DL_pid);
    int powerR = updatePID(&DR_pid);
    limMotorVal(&powerL);
    limMotorVal(&powerR);
    setDL(powerL);
    setDR(powerR);
    return false;
}
bool pidTurn(double angle, unsigned long wait) {
    if (DLturn_pid.doneTime + wait < millis() && DRturn_pid.doneTime + wait < millis()) {
        setDL(0);
        setDR(0);
        printf("DONE(pidTurn)\n");
        return true;
    }

    DLturn_pid.sensVal = eDLGet();
    DLturn_pid.target = -angle * DRIVE_TICKS_PER_DEG;
    DRturn_pid.sensVal = eDRGet();
    DRturn_pid.target = angle * DRIVE_TICKS_PER_DEG;
    int powerL = updatePID(&DLturn_pid);
    int powerR = updatePID(&DRturn_pid);
    powerL = limInt(powerL, -127, 127);
    powerR = limInt(powerR, -127, 127);

    turnCurve_pid.sensVal = eDRGet() + eDLGet();
    turnCurve_pid.target = 0.0;
    double curve = updatePID(&turnCurve_pid) * 0.5 * (fabs((DLturn_pid.target - DLturn_pid.sensVal) / DLturn_pid.target) + fabs((DRturn_pid.target - DRturn_pid.sensVal) / DRturn_pid.target));
    double curveInfluence = 0.5;
    if (abs(powerR) > abs(powerL)) {
        if (powerL > 0) {
            curve = limInt(curve, -powerL * curveInfluence, powerL * curveInfluence);
        } else {
            curve = limInt(curve, powerL * curveInfluence, -powerL * curveInfluence);
        }
    } else {
        if (powerR > 0) {
            curve = limInt(curve, -powerR * curveInfluence, powerR * curveInfluence);
        } else {
            curve = limInt(curve, powerR * curveInfluence, -powerR * curveInfluence);
        }
    }
    setDL(powerL + curve);
    setDR(powerR + curve);
    return false;
}

PidVars DLturn_brake, DRturn_brake;
unsigned long ltTurnDt = 0, ltTurnPrevT = 0;
double ltTurnKd = 100.0, ltTurnKi = 0.1;
int ltTurnInt = 0;
void ltTurn(bool dir) {
    bool b0 = analogReadCalibrated(LT0) < LT_LIGHT;
    bool b1 = analogReadCalibrated(LT1) < LT_LIGHT;
    bool b2 = analogReadCalibrated(LT2) < LT_LIGHT;
    bool b3 = analogReadCalibrated(LT3) < LT_LIGHT;
    bool b4 = analogReadCalibrated(LT4) < LT_LIGHT;
    DLturn_brake.sensVal = eDLGet();
    DRturn_brake.sensVal = eDRGet();
    int powerL = 0, powerR = 0;
    if (!b2) {
        if (b1) {
            powerR = -40;
            powerL = 40;
            ltTurnInt -= 1;
        } else if (b0) {
            powerR = -80;
            powerL = 80;
            ltTurnInt -= 2;
        } else if (b3) {
            powerR = 40;
            powerL = -40;
            ltTurnInt += 1;
        } else if (b4) {
            powerR = 80;
            powerL = -80;
            ltTurnInt += 2;
        } else if (dir) {
            powerR = 999;
            powerL = -999;
        } else {
            powerR = -999;
            powerL = 999;
        }
    } else {
        ltTurnInt = 0;
    }

    powerL += ((DLturn_brake.prevSensVal - DLturn_brake.sensVal) / ltTurnDt) * ltTurnKd - ltTurnInt * ltTurnKi;
    powerR += ((DRturn_brake.prevSensVal - DRturn_brake.sensVal) / ltTurnDt) * ltTurnKd + ltTurnInt * ltTurnKi;
    setDL(powerL);
    setDR(powerR);
    DLturn_brake.prevSensVal = DLturn_brake.sensVal;
    DRturn_brake.prevSensVal = DRturn_brake.sensVal;
    ltTurnDt = millis() - ltTurnPrevT;
    if (ltTurnDt > 300) ltTurnDt = 20;
    ltTurnPrevT = millis();
}
