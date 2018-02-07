#include "auto.h"
#include "main.h"
#include "pid.h"
#include "setup.h"
//----- updates Arm and Chain-Bar -----//
unsigned long tFbOff = 0, tDrfbOff = 0;
double fbHoldAngle = 0, drfbHoldAngle = 0;
bool fbPidRunning = false, drfbPidRunning = false;
/*
todo:

-wheels on wheelie bar

-add diff PID for driving and turning drift prevention out = dist_PID + diff_PID

*/

unsigned long opT0;
void updateLift() {
    //------ update four bar -----//
    if (joystickGetDigital(2, 7, JOY_UP)) {
        setFB(90);
        tFbOff = millis();
        fbPidRunning = false;
    } else if (joystickGetDigital(2, 7, JOY_DOWN)) {
        setFB(-90);
        tFbOff = millis();
        fbPidRunning = false;
    } else {
        if (joystickGetDigital(2, 5, JOY_UP)) {
            fbHoldAngle = FB_UP_POS;
            tFbOff = 0;
            fbPidRunning = true;
            lcdSetText(LCD, 1, "up");
        } else if (joystickGetDigital(2, 5, JOY_DOWN)) {
            fbHoldAngle = FB_MID_POS;
            tFbOff = 0;
            fbPidRunning = true;
            lcdSetText(LCD, 1, "mid");
        }
        if (millis() - tFbOff > 300) {
            if (!fbPidRunning) {
                fbHoldAngle = fbGet();
                fbPidRunning = true;
            }
            fb_pid_auto.sensVal = fbGet();
            if (fbHoldAngle < FB_MIN_HOLD_ANGLE) fbHoldAngle = FB_MIN_HOLD_ANGLE;
            fb_pid_auto.target = fbHoldAngle;
            lcdPrint(LCD, 2, "%d/%d", (int)fb_pid_auto.sensVal, (int)fb_pid_auto.target);
            setFB(updatePID(&fb_pid_auto));
        } else {
            setFB(0);
            fbPidRunning = false;
        }
    }
    const int t = 15;
    int js = joystickGetAnalog(2, 2);
    if (js > DRFB_MAX) js = DRFB_MAX;
    if (js < -DRFB_MAX) js = -DRFB_MAX;
    if (abs(js) > t) {
        tDrfbOff = millis();
        drfbPidRunning = false;
        setDRFB(js);
    } else if (millis() - tDrfbOff > 300) {
        if (!drfbPidRunning) {
            drfbHoldAngle = drfbGet();
            drfbPidRunning = true;
        }
        drfb_pid.sensVal = drfbGet();
        if (drfbHoldAngle > DRFB_MAX_HOLD_ANGLE) drfbHoldAngle = DRFB_MAX_HOLD_ANGLE;
        drfb_pid.target = drfbHoldAngle;
        setDRFB(updatePID(&drfb_pid));
    } else {
        setDRFB(0);
        drfbPidRunning = false;
    }
}
void test(int n) {
    switch (n) {
        case 0:
            while (!pidDrive(60, 20000)) {
                printEnc_pidDrive();
                delay(20);
            }
            break;
        case 1:
            while (!pidTurn(-180, 20000)) {
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
        lcdPrint(LCD, 1, "L: %d", eDLGet());
        lcdPrint(LCD, 2, "R: %d", eDRGet());
        delay(20);
    }
    for (int i = 10; i > 0; i--) {
        delay(200);
        printf("%d\n", i);
    }
    autonSkills();
    return;
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
        // printEnc();
        updateLift();
        //----- mobile-goal lift -----//
        if (joystickGetDigital(1, 8, JOY_RIGHT) || joystickGetDigital(1, 6, JOY_UP)) {
            mglPidRunning = true;
            mgl_pid.doneTime = LONG_MAX;
            mglHoldAngle = MGL_UP_POS;
            tMglOff = 0;
        } else if (joystickGetDigital(1, 8, JOY_LEFT) || joystickGetDigital(1, 5, JOY_DOWN) || joystickGetDigital(1, 5, JOY_UP)) {
            mglPidRunning = true;
            mgl_pid.doneTime = LONG_MAX;
            mglHoldAngle = MGL_MID_POS - 10;
            tMglOff = 0;
        } else if (joystickGetDigital(1, 8, JOY_UP)) {
            tMglOff = millis();
            setMGL(-127);
        } else if (joystickGetDigital(1, 8, JOY_DOWN)) {
            tMglOff = millis();
            setMGL(127);
        } else if (joystickGetDigital(1, 6, JOY_DOWN)) {
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
        if (joystickGetDigital(2, 7, JOY_RIGHT)) {
            // stop rollers
            rollDir = 0;
            setRollers(0);
        } else if (joystickGetDigital(2, 6, JOY_UP)) {
            // intake
            rollDir = 1;
            tRollersOff = millis();
            setRollers(70);
        } else if (joystickGetDigital(2, 6, JOY_DOWN)) {
            // outtake
            rollDir = -1;
            tRollersOff = millis();
            setRollers(-80);
        } else {
            if (millis() - tRollersOff < 500 && rollDir == -1) {
                setRollers(-60);
            } else if (rollDir != 0) {
                setRollers(25);
            } else {
                setRollers(0);
            }
        }
        //----- drive -----//
        const int td = 15;
        int drv = joystickGetAnalog(1, 3);
        int trn = joystickGetAnalog(1, 1);
        if (trn > DRIVE_TURN_MAX) trn = DRIVE_TURN_MAX;
        if (trn < -DRIVE_TURN_MAX) trn = -DRIVE_TURN_MAX;
        if (drv > DRIVE_DRIVE_MAX) drv = DRIVE_DRIVE_MAX;
        if (drv < -DRIVE_DRIVE_MAX) drv = -DRIVE_DRIVE_MAX;
        if (abs(drv) < td) drv = 0;
        if (abs(trn) < td) trn = 0;
        setDL(drv + trn);
        setDR(drv - trn);
        if (drv == 0 && trn == 0) {
            DL_brake_out = ((DL_brake.prevSensVal - eDLGet()) * DL_brake.kd) / dt;
            DR_brake_out = ((DR_brake.prevSensVal - eDRGet()) * DR_brake.kd) / dt;
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
