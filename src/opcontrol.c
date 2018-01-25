#include "auto.h"
#include "main.h"
#include "pid.h"
#include "setup.h"
//----- updates Arm and Chain-Bar -----//
unsigned long tFbOff = 0, tDrfbOff = 0;
double fbHoldAngle = 0, drfbHoldAngle = 0;
bool fbPidRunning = false, drfbPidRunning = false, fbUp = false;
/*
todo:
-skills / auton
-auton selection




-2 motor(HS) fb
-passive PC intake
-wheels on wheelie bar

stuff to reduce weight:
-half MGL 2 2x25
-replace front MGL 2x15 with standoff

-maybe half the 2 2x15 and 2 2x10 on the fb tower
*/

unsigned long opT0;
void updateLift() {
    static bool returning = false;
    //------ update four bar -----//
    bool moving = true;
    if (joystickGetDigital(1, 7, JOY_UP)) {
        setFB(127);
        tFbOff = millis();
        fbPidRunning = false;
        returning = false;
        fbUp = false;
    } else if (joystickGetDigital(1, 7, JOY_DOWN)) {
        setFB(-127);
        tFbOff = millis();
        fbPidRunning = false;
        returning = false;
        fbUp = false;
    } else {
        moving = false;
    }
    /*
    int j2 = joystickGetAnalog(1, 2);
    if (abs(j2) > 15) {
        setFB(j2);
        tFbOff = millis();
        fbPidRunning = false;
        returning = false;
        fbUp = false;
    } else {
        moving = false;
    }*/
    if (!moving) {
        if (joystickGetDigital(1, 5, JOY_UP)) {
            tFbOff = 0;
            fbPidRunning = true;
            returning = false;
            fbUp = true;
        } else if (joystickGetDigital(1, 5, JOY_DOWN)) {
            fbHoldAngle = FB_MID_POS;
            tFbOff = 0;
            fbPidRunning = true;
            returning = false;
            fbUp = false;
        }
        if (!returning) {
            if (millis() - tFbOff > 300) {
                if (fbUp) fbHoldAngle = FB_UP_POS;
                if (!fbPidRunning) {
                    fbHoldAngle = fbGet();
                    fbPidRunning = true;
                }
                fb_pid_auto.sensVal = fbGet();
                if (fbHoldAngle < FB_MIN_CUT) fbHoldAngle = FB_MIN_CUT;
                fb_pid_auto.target = fbHoldAngle;
                setFB(updatePID(&fb_pid_auto));
            } else {
                setFB(0);
                fbPidRunning = false;
            }
        }
    }
    const int t = 15;
    int js = joystickGetAnalog(1, 2);
    if (abs(js) > t) {
        tDrfbOff = millis();
        drfbPidRunning = false;
        returning = false;
        setDRFB(js);
    } else if (!returning) {
        if (millis() - tDrfbOff > 300) {
            if (!drfbPidRunning) {
                drfbHoldAngle = drfbGet();
                drfbPidRunning = true;
            }
            drfb_pid.sensVal = drfbGet();
            if (drfbHoldAngle > DRFB_MAX_CUT) drfbHoldAngle = DRFB_MAX_CUT;
            drfb_pid.target = drfbHoldAngle;
            setDRFB(updatePID(&drfb_pid));
        } else {
            setDRFB(0);
            drfbPidRunning = false;
        }
    }
}
void test(int n) {
    switch (n) {
        case 0:
            while (!pidDrive(40, 20000)) {
                printEnc_pidDrive();
                delay(20);
            }
            break;
        case 1:
            while (!pidTurn(-90, 20000)) {
                printEnc_pidDrive();
                delay(20);
            }
            break;
        case 2:
            while (true) {
                printEnc_pidDRFBFB();
                fb_pid_auto.target = 40;
                fb_pid_auto.sensVal = fbGet();
                int p = updatePID(&fb_pid_auto);
                setFB(p);
                printf("power: %d\t", p);
                delay(20);
            }
            break;
        case 3:
            while (true) {
                printEnc_pidDRFBFB();
                drfb_pid_auto.target = 20;
                drfb_pid_auto.sensVal = drfbGet();
                int p = updatePID(&drfb_pid_auto);
                setDRFB(p);
                printf("power: %d\t", p);
                delay(20);
            }
            break;
        case 4:
            while (true) {
                printEnc();
                mgl_pid.target = MGL_DOWN_POS;
                mgl_pid.sensVal = mglGet();
                int p = updatePID(&mgl_pid);
                setMGL(p);
                printf("power: %d\t", p);
                delay(20);
            }
            break;
    }
    while (true) { delay(999); }
}
void controllerTest() {
    while (true) {
        if (joystickGetDigital(1, 8, JOY_UP)) {
            lcdSetText(LCD, 1, "    1    \n");
        } else if (joystickGetDigital(2, 8, JOY_UP)) {
            lcdSetText(LCD, 1, "    2    \n");
        }
        delay(20);
    }
}
#include "auto.h"
void operatorControl() {
    while (0) {
        printEnc();
        delay(20);
    }
    for (int i = 10; i > 0; i--) {
        delay(200);
        printf("%d\n", i);
    }
    // skillsUser();
    skillsAuto();
    // auton1(true, true);
    // skills();
    opT0 = millis();
    /*if (autonMode == nAutons + nSkills) {
        auton1(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &drfb_pid, &fb_pid, false, true);
    }*/
    char rollDir = 0;
    unsigned long tMglOff = 0, tRollersOff = 0;
    double mglHoldAngle = 0;
    bool mglPidRunning = false;

    PidVars DL_brake = pidDef, DR_brake = pidDef;
    DL_brake.kd = 30;
    DR_brake.kd = 30;
    unsigned long dt = 0, prevT = 0;
    int DL_brake_out = 0, DR_brake_out = 0;
    while (true) {
        printEnc();
        // printEnc_pidDRFBFB();
        lcdPrint(LCD, 1, "%d", joystickGetDigital(2, 6, JOY_UP) ? 1 : 0);
        updateLift();
        //----- mobile-goal lift -----//
        if (joystickGetDigital(DM == 0 ? 1 : 2, 8, JOY_RIGHT) || joystickGetDigital(2, 6, JOY_UP)) {
            mglPidRunning = true;
            mgl_pid.doneTime = LONG_MAX;
            mglHoldAngle = MGL_UP_POS;
            tMglOff = 0;
        } else if (joystickGetDigital(DM == 0 ? 1 : 2, 8, JOY_LEFT) || joystickGetDigital(2, 5, JOY_DOWN) || joystickGetDigital(2, 5, JOY_UP)) {
            mglPidRunning = true;
            mgl_pid.doneTime = LONG_MAX;
            mglHoldAngle = MGL_MID_POS - 10;
            tMglOff = 0;
        } else if (joystickGetDigital(DM == 0 ? 1 : 2, 8, JOY_UP)) {
            tMglOff = millis();
            setMGL(-127);
        } else if (joystickGetDigital(2, 8, JOY_DOWN)) {
            tMglOff = millis();
            setMGL(127);
        } else if (joystickGetDigital(2, 6, JOY_DOWN) || (DM == 0 && joystickGetDigital(1, 8, JOY_DOWN))) {
            mglPidRunning = true;
            mgl_pid.doneTime = LONG_MAX;
            mglHoldAngle = MGL_DOWN_POS;
            tMglOff = 0;
        } else if (millis() - tMglOff > 250 || tMglOff == 0) {
            if (!mglPidRunning && mglGet() <= MGL_MAX && mglGet() >= MGL_MIN) {
                mglPidRunning = true;
                mgl_pid.doneTime = LONG_MAX;
                mglHoldAngle = mglGet();
            }
        } else {
            mglPidRunning = false;
            setMGL(0);
        }
        if (mglPidRunning) {
            if (pidMGL(mglHoldAngle, 200)) mglPidRunning = false;
        }
        //----- rollers -----//
        if (joystickGetDigital(1, 7, JOY_RIGHT)) {
            // stop rollers
            rollDir = 0;
            setRollers(0);
        } else if (joystickGetDigital(1, 6, JOY_UP)) {
            // intake
            rollDir = 1;
            tRollersOff = millis();
            setRollers(50);
        } else if (joystickGetDigital(1, 6, JOY_DOWN)) {
            // outtake
            rollDir = -1;
            tRollersOff = millis();
            setRollers(-60);
        } else {
            if (millis() - tRollersOff < 500 && rollDir == -1) {
                setRollers(-50);
            } else if (rollDir != 0) {
                setRollers(25);
            } else {
                setRollers(0);
            }
        }
        //----- drive -----//
        const int td = 15;
        bool moving = true;
        DL_brake_out = ((DL_brake.prevSensVal - eDLGet()) * DL_brake.kd) / dt;
        DR_brake_out = ((DR_brake.prevSensVal - eDRGet()) * DR_brake.kd) / dt;
        if (DM != 1) {
            int drv = joystickGetAnalog(DM == 0 ? 1 : 2, 3);
            int trn = DM == 0 ? joystickGetAnalog(1, 4) : joystickGetAnalog(2, 1);
            if (trn > DRIVE_TURN_MAX) trn = DRIVE_TURN_MAX;
            if (trn < -DRIVE_TURN_MAX) trn = -DRIVE_TURN_MAX;
            if (drv > DRIVE_DRIVE_MAX) drv = DRIVE_DRIVE_MAX;
            if (drv < -DRIVE_DRIVE_MAX) drv = -DRIVE_DRIVE_MAX;
            if (abs(drv) < td) drv = 0;
            if (abs(trn) < td) trn = 0;
            if (drv == 0 && trn == 0) moving = false;
            setDL(drv + trn);
            setDR(drv - trn);
        } else {
            int dl = joystickGetAnalog(2, 3), dr = joystickGetAnalog(2, 2);
            if (abs(dl) < td) dl = 0;
            if (abs(dr) < td) dr = 0;
            if (dl == 0 && dr == 0) moving = false;
            setDL(dl);
            setDR(dr);
        }
        if (!moving) {
            int brakeMax = 100;
            if (DL_brake_out > brakeMax) DL_brake_out = brakeMax;
            if (DL_brake_out < -brakeMax) DL_brake_out = -brakeMax;
            if (DR_brake_out > brakeMax) DR_brake_out = brakeMax;
            if (DR_brake_out < -brakeMax) DR_brake_out = -brakeMax;
            setDL(DL_brake_out);
            setDR(DR_brake_out);
        }
        DL_brake.prevSensVal = eDLGet();
        DR_brake.prevSensVal = eDRGet();
        dt = millis() - prevT;
        prevT = millis();
        delay(20);
    }
}
