#include "pid.h"
#include <math.h>
#include "API.h"
#include "setup.h"

/*
########  #### ########      ######  ######## ######## ##     ## ########
##     ##  ##  ##     ##    ##    ## ##          ##    ##     ## ##     ##
##     ##  ##  ##     ##    ##       ##          ##    ##     ## ##     ##
########   ##  ##     ##     ######  ######      ##    ##     ## ########
##         ##  ##     ##          ## ##          ##    ##     ## ##
##         ##  ##     ##    ##    ## ##          ##    ##     ## ##
##        #### ########      ######  ########    ##     #######  ##
*/

/*
     -------- PidVars --------
     must be set by user:
     kp, ki, kd, INTEGRAL_ACTIVE_ZONE, DERIVATIVE_ACTIVE_ZONE, maxIntegral,
   target, sensVal

     automatically handled by functions:
     prevError, errTot, prevTime
     -------------------------
*/
// motor value change per ms           0.7
Slew fb_slew = {.a = 2.5, .out = 0.0, .prevTime = 0};
Slew drfb_slew = {.a = 1.3, .out = 0.0, .prevTime = 0};
Slew mgl_slew = {.a = 2.0, .out = 0.0, .prevTime = 0};
Slew DL_slew = {.a = 1.2, .out = 0.0, .prevTime = 0};
Slew DR_slew = {.a = 1.2, .out = 0.0, .prevTime = 0}; /*
  Slew DL_slew_auto = {.a = 1.0, .out = 0.0, .prevTime = 0};
  Slew DR_slew_auto = {.a = 1.0, .out = 0.0, .prevTime = 0};*/
PidVars pidDef = {.doneTime = LONG_MAX, .DONE_ZONE = 10, .maxIntegral = DBL_MAX, .iActiveZone = 0.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 0.0, .ki = 0.0, .kd = 0.0, .prevTime = 0, .unwind = 0, .prevDUpdateTime = 0, .deriv = 0.0};
// weaker PID for opcontrol
PidVars drfb_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 5, .maxIntegral = 30, .iActiveZone = 10.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 2.2, .ki = 0.006, .kd = 100, .prevTime = 0, .unwind = 0, .prevDUpdateTime = 0, .deriv = 0.0};
PidVars mgl_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 3, .maxIntegral = 127, .iActiveZone = 8.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 6.0, .ki = 0.01, .kd = 80.0, .prevTime = 0, .unwind = 0, .prevDUpdateTime = 0, .deriv = 0.0};
// agressive PID mostly for autonomous
PidVars drfb_pid_auto = {.doneTime = LONG_MAX, .DONE_ZONE = 5, .maxIntegral = 40, .iActiveZone = 12.0, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 3.5, .ki = 0.01, .kd = 420, .prevTime = 0, .unwind = 0, .prevDUpdateTime = 0, .deriv = 0.0};
#define FBKP 3.0
#define FBKI 0.03
#define FBKD 200
#define FBDZ 8
#define FBIA 10
PidVars fb_pid_auto = {.doneTime = LONG_MAX, .DONE_ZONE = FBDZ, .maxIntegral = 40, .iActiveZone = FBIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = FBKP, .ki = FBKI, .kd = FBKD, .prevTime = 0, .unwind = 0, .prevDUpdateTime = 0, .deriv = 0.0};  // 1.8, 0.01, 175.0
PidVars fb_pid = {.doneTime = LONG_MAX, .DONE_ZONE = FBDZ, .maxIntegral = 40, .iActiveZone = FBIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = FBKP, .ki = FBKI, .kd = FBKD, .prevTime = 0, .unwind = 0, .prevDUpdateTime = 0, .deriv = 0.0};       // 1.8, 0.01, 175.0
// Drive
#define DIA 40
#define DDZ 6
#define DKP 0.58    //.58
#define DKI 0.0025  // .004
#define DKD 63      // 60
PidVars DL_pid = {.doneTime = LONG_MAX, .DONE_ZONE = DDZ, .maxIntegral = 40, .iActiveZone = DIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = DKP, .ki = DKI, .kd = DKD, .prevTime = 0, .unwind = 0, .prevDUpdateTime = 0, .deriv = 0.0};
PidVars DR_pid = {.doneTime = LONG_MAX, .DONE_ZONE = DDZ, .maxIntegral = 40, .iActiveZone = DIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = DKP, .ki = DKI, .kd = DKD, .prevTime = 0, .unwind = 0, .prevDUpdateTime = 0, .deriv = 0.0};
#define DSKP 1.5     //.65
#define DSKI 0.0035  // .0035
#define DSKD 130     // 80
PidVars DLshort_pid = {.doneTime = LONG_MAX, .DONE_ZONE = DDZ, .maxIntegral = 50, .iActiveZone = 30, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = DSKP, .ki = DSKI, .kd = DSKD, .prevTime = 0, .unwind = 0, .prevDUpdateTime = 0, .deriv = 0.0};
PidVars DRshort_pid = {.doneTime = LONG_MAX, .DONE_ZONE = DDZ, .maxIntegral = 50, .iActiveZone = 30, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = DSKP, .ki = DSKI, .kd = DSKD, .prevTime = 0, .unwind = 0, .prevDUpdateTime = 0, .deriv = 0.0};
PidVars driveCurve_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 4, .maxIntegral = 40, .iActiveZone = DIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 3, .ki = 0.0, .kd = 0.0, .prevTime = 0, .unwind = 0, .prevDUpdateTime = 0, .deriv = 0.0};  // 6, 0, 0
#define TIA 35
#define TDZ 3
#define TKP 1.575
#define TKI 0.005
#define TKD 160.8
PidVars DLturn_pid = {.doneTime = LONG_MAX, .DONE_ZONE = TDZ, .maxIntegral = 45, .iActiveZone = TIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = TKP, .ki = TKI, .kd = TKD, .prevTime = 0, .unwind = 0, .prevDUpdateTime = 0, .deriv = 0.0};
PidVars DRturn_pid = {.doneTime = LONG_MAX, .DONE_ZONE = TDZ, .maxIntegral = 45, .iActiveZone = TIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = TKP, .ki = TKI, .kd = TKD, .prevTime = 0, .unwind = 0, .prevDUpdateTime = 0, .deriv = 0.0};
PidVars turnCurve_pid = {.doneTime = LONG_MAX, .DONE_ZONE = 2, .maxIntegral = 20, .iActiveZone = TIA, .target = 0.0, .sensVal = 0.0, .prevErr = 0.0, .errTot = 0.0, .kp = 1.5, .ki = 0.0, .kd = 0.0, .prevTime = 0, .unwind = 0, .prevDUpdateTime = 0, .deriv = 0.0};  // 3
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
    double d = pidVars->deriv;  // set d to old derivative
    double derivativeDt = millis() - pidVars->prevDUpdateTime;
    if (derivativeDt > 1000) {
        pidVars->prevSensVal = pidVars->sensVal;
        pidVars->prevDUpdateTime = millis();
    } else if (derivativeDt >= 15) {
        d = ((pidVars->prevSensVal - pidVars->sensVal) * pidVars->kd) / derivativeDt;
        pidVars->prevDUpdateTime = millis();
        pidVars->deriv = d;  // save new derivative
        pidVars->prevSensVal = pidVars->sensVal;
    }
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
    // printf("p: %lf, i: %lf, d: %lf\t", p, i, d);
    // OUTPUT
    return p + i + d;
}
bool killPID(int d, int s, PidVars *p) {
    double err = p->target - p->sensVal;
    if (fabs(err) < d && fabs(err - p->prevErr) < s) { return true; }
    return false;
}
/*
########  #### ########     ##       #### ######## ########
##     ##  ##  ##     ##    ##        ##  ##          ##
##     ##  ##  ##     ##    ##        ##  ##          ##
########   ##  ##     ##    ##        ##  ######      ##
##         ##  ##     ##    ##        ##  ##          ##
##         ##  ##     ##    ##        ##  ##          ##
##        #### ########     ######## #### ##          ##
*/
// note: wrap rubber bands 2 extra times around rubber band mounts
bool pidFB(double a, unsigned long wait, bool auton) {  // set fb angle with PID
    PidVars *pid = auton ? &fb_pid_auto : &fb_pid;
    pid->target = a;
    pid->sensVal = fbGet();
    setFB(2 + updatePID(pid));
    if (pid->doneTime + wait < millis()) return true;
    return false;
}
bool pidDRFB(double a, unsigned long wait, bool auton) {  // set drfb angle with PID
    PidVars *pid = auton ? &drfb_pid_auto : &drfb_pid;
    pid->target = a;
    pid->sensVal = drfbGet();
    setDRFB(updatePID(pid));
    if (pid->doneTime + wait < millis()) return true;
    return false;
}
bool pidMGL(double a, unsigned long wait) {  // set mgl angle with PID
    mgl_pid.target = a;
    mgl_pid.sensVal = mglGet();
    setMGL(updatePID(&mgl_pid));
    if (mgl_pid.doneTime + wait < millis()) return true;
    return false;
}
double ltdKi = 0.5, ltdKp = 12.0, ltdInt = 0.0;
/*
 ######  ######## ########    ########   #######  ##      ## ##    ##     ######  ########    ###     ######  ##    ##
##    ## ##          ##       ##     ## ##     ## ##  ##  ## ###   ##    ##    ##    ##      ## ##   ##    ## ##   ##
##       ##          ##       ##     ## ##     ## ##  ##  ## ####  ##    ##          ##     ##   ##  ##       ##  ##
 ######  ######      ##       ##     ## ##     ## ##  ##  ## ## ## ##     ######     ##    ##     ## ##       #####
      ## ##          ##       ##     ## ##     ## ##  ##  ## ##  ####          ##    ##    ######### ##       ##  ##
##    ## ##          ##       ##     ## ##     ## ##  ##  ## ##   ###    ##    ##    ##    ##     ## ##    ## ##   ##
 ######  ########    ##       ########   #######   ###  ###  ##    ##     ######     ##    ##     ##  ######  ##    ##
*/
bool settingDownStack = false;
/* PRECONDITIONS:
-settingDownStack set to false before starting
-at least 3 cones stacked on MG in robot
*/
bool setDownStack() {
    if (joystickGetDigital(2, 8, JOY_LEFT)) return true;
    static int i = 0, prevI;
    static unsigned long prevT;
    static double h = 0.0;
    if (settingDownStack == false) {
        i = 0;
        settingDownStack = true;
        h = /*0.04 + */ sin((M_PI / 180.0) * (drfbGet() - DRFB_HORIZONTAL));
        if (h > 1.0) h = 1.0;
        mgl_pid.doneTime = LONG_MAX;
    }
    bool allowRepeat = true;
    while (allowRepeat) {
        allowRepeat = false;
        int j = 0;
        if (i == j++) {
            prevI = i;
            i++;
        } else if (i == j++) {
            fb_pid_auto.target = FB_MID_POS - 15;
            fb_pid_auto.sensVal = fbGet();
            setFB(limInt((int)updatePID(&fb_pid_auto), -30, 30));  // limit fb to keep claw from going ahead of cone
            double angleUp = DRFB_HORIZONTAL + (180.0 / M_PI) * asin(h);
            if (angleUp > DRFB_ENDPT_UP) angleUp = DRFB_ENDPT_UP;
            pidDRFB(angleUp, 999999, true);
            lcdPrint(LCD, 1, "up:%d", (int)angleUp);
            if (pidMGL(MGL_DOWN_POS, 0)) {
                h = -0.65 + sin((M_PI / 180.0) * (drfbGet() - DRFB_HORIZONTAL));
                if (h < -1.0) h = -1.0;
                drfb_pid_auto.doneTime = LONG_MAX;
                i++;
            }
        } else if (i == j++) {
            pidMGL(MGL_DOWN_POS, 999999);
            double angleDown = DRFB_HORIZONTAL + (180.0 / M_PI) * asin(h);
            lcdPrint(LCD, 1, "down:%d", (int)angleDown);
            if (angleDown < DRFB_ENDPT_DOWN) angleDown = DRFB_ENDPT_DOWN;
            if (drfbGet() > angleDown + 10) {
                setDRFB(-127);
            } else {
                pidDRFB(angleDown, 999999, true);
            }
            fb_pid_auto.target = FB_UP_POS;  // 22
            fb_pid_auto.sensVal = fbGet();
            setFB(limInt((int)updatePID(&fb_pid_auto), -127, 127));
            if ((fbGet() > FB_UP_POS - 21 && drfbGet() < angleDown + 5)) i++;
        } else if (i == j++) {
            return true;
        }

        if (i != prevI) {
            prevT = millis();
            allowRepeat = true;
        }
        prevI = i;
        // safety first (ptc tripped or robot got stuck)
        if (millis() - prevT > 3000) i++;
    }
    return false;
} /*
 void setDownStack() {
     while (true) {
         setDownStackAuton();
         opctrlDrive();
         printEnc_all();
         delay(5);
     }
     resetMotors();
 }*/
/*
########  #### ########     ########  ########  #### ##     ## ########
##     ##  ##  ##     ##    ##     ## ##     ##  ##  ##     ## ##
##     ##  ##  ##     ##    ##     ## ##     ##  ##  ##     ## ##
########   ##  ##     ##    ##     ## ########   ##  ##     ## ######
##         ##  ##     ##    ##     ## ##   ##    ##   ##   ##  ##
##         ##  ##     ##    ##     ## ##    ##   ##    ## ##   ##
##        #### ########     ########  ##     ## ####    ###    ########
*/

// This function is best when dist > 6
bool pidDrive(double dist, unsigned long wait) {
    DL_pid.sensVal = eDLGet();
    DR_pid.sensVal = eDRGet();

    // 89 inches = 2457 ticks : 2457.0/89.0 = 27.6067
    DL_pid.target = dist * DRIVE_TICKS_PER_IN;
    DR_pid.target = dist * DRIVE_TICKS_PER_IN;
    int powerL = updatePID(&DL_pid);
    int powerR = updatePID(&DR_pid);
    limMotorVal(&powerL);
    limMotorVal(&powerR);

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

    limMotorVal(&powerL);
    limMotorVal(&powerR);
    setDL(powerL);
    setDR(powerR);
    if (DL_pid.doneTime + wait < millis() && DR_pid.doneTime + wait < millis()) return true;
    return false;
}
bool pidTurn(double angle, unsigned long wait) {
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
    double curve = updatePID(&turnCurve_pid) * pow(0.5 * (fabs((DLturn_pid.target - DLturn_pid.sensVal) / DLturn_pid.target) + fabs((DRturn_pid.target - DRturn_pid.sensVal) / DRturn_pid.target)), 2);
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
    if (DLturn_pid.doneTime + wait < millis() && DRturn_pid.doneTime + wait < millis()) return true;
    return false;
}

// This function is best on on interval [0, 6]
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
bool pidDriveShort(double dist, unsigned long wait) {
    DLshort_pid.sensVal = eDLGet();
    DRshort_pid.sensVal = eDRGet();

    // 89 inches = 2457 ticks : 2457.0/89.0 = 27.6067
    DLshort_pid.target = dist * DRIVE_TICKS_PER_IN;
    DRshort_pid.target = dist * DRIVE_TICKS_PER_IN;
    int powerL = updatePID(&DLshort_pid);
    int powerR = updatePID(&DRshort_pid);
    limMotorVal(&powerL);
    limMotorVal(&powerR);

    driveCurve_pid.sensVal = eDRGet() - eDLGet();
    driveCurve_pid.target = 0.0;
    int curve = updatePID(&driveCurve_pid) * 0.5 * (fabs((DLshort_pid.target - DLshort_pid.sensVal) / DLshort_pid.target) + fabs((DRshort_pid.target - DRshort_pid.sensVal) / DRshort_pid.target));
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

    limMotorVal(&powerL);
    limMotorVal(&powerR);
    setDL(powerL);
    setDR(powerR);
    if (DLshort_pid.doneTime + wait < millis() && DRshort_pid.doneTime + wait < millis()) return true;
    return false;
}
