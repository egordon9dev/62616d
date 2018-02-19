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
bool prev7u = false, prev7d = false;
void updateLift() {
    //------ update four bar -----//
    int j3 = joystickGetAnalog(2, 3);
    bool cur7u = joystickGetDigital(2, 7, JOY_UP), cur7d = joystickGetDigital(2, 7, JOY_DOWN);
    if (!prev7u && cur7u) {
        fbUpP += 0.5;
    } else if (!prev7d && cur7d) {
        fbUpP -= 0.5;
    } else if (joystickGetDigital(2, 7, JOY_LEFT) || joystickGetDigital(2, 7, JOY_RIGHT)) {
        fbUpP = FB_UP_P0;
    }
    if (abs(j3) > 15) {
        setFB(j3);
        tFbOff = millis();
        fbPidRunning = false;
    } else if (joystickGetDigital(2, 5, JOY_UP)) {
        fbHoldAngle = FB_UP_POS;
        fbPidRunning = true;
    } else if (joystickGetDigital(2, 5, JOY_DOWN)) {
        fbHoldAngle = FB_MID_POS;
        fbPidRunning = true;
    } else if (millis() - tFbOff > 300) {
        if (!fbPidRunning) {
            fbHoldAngle = fbGet();
            fbPidRunning = true;
        }
    } else if (!fbPidRunning) {
        setFB(0);
    }
    if (fbPidRunning) {
        if (fbHoldAngle < FB_MIN_HOLD_ANGLE) fbHoldAngle = FB_MIN_HOLD_ANGLE;
        pidFB(fbHoldAngle, 999999, true);
    }
    const int t = 15;
    int js = joystickGetAnalog(2, 2) * DRFB_MAX / 127.0;
    if (abs(js) > t) {
        tDrfbOff = millis();
        drfbPidRunning = false;
        setDRFB(js);
    } else if (millis() - tDrfbOff > 300) {
        if (!drfbPidRunning) {
            drfbHoldAngle = drfbGet();
            drfbPidRunning = true;
        }
    } else if (!drfbPidRunning) {
        setDRFB(0);
    }
    if (drfbPidRunning) {
        if (drfbHoldAngle > DRFB_MAX_HOLD_ANGLE) drfbHoldAngle = DRFB_MAX_HOLD_ANGLE;
        pidDRFB(drfbHoldAngle, 999999, false);
    }
}
void test(int n) {
    switch (n) {
        case 0:
            while (!pidDrive(30, 1000, false)) {
                printEnc_pidDrive();
                delay(20);
            }
            break;
        case 1:
            while (!pidTurn(160, 100)) {
                printEnc_pidDrive();
                delay(20);
            }
            break;
        case 2:
            while (true) {
                printEnc_pidDRFBFB();
                pidFB(FB_MID_POS, 999999, true);
                delay(20);
            }
            break;
        case 3:
            while (true) {
                printEnc_pidDRFBFB();
                pidDRFB(20, 999999, true);
                delay(20);
            }
            break;
        case 4:
            while (true) {
                printEnc();
                if (pidMGL(MGL_DOWN_POS, 0)) setMGL(0);
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
        printf("LT1: %d\tLT2: %d\tLT3: %d\n", analogReadCalibrated(LT1), analogReadCalibrated(LT2), analogReadCalibrated(LT3));
        delay(20);
    }
    while (0) {
        printEnc();
        delay(20);
    }
    for (int i = 10; i > 0; i--) {
        delay(200);
        printf("%d\n", i);
    }
    auton1(true, 1, true);
    // autonSkills();
    return;
    opT0 = millis();
    /*if (autonMode == nAutons + nSkills) {
        auton1(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &drfb_pid, &fb_pid, false, true);
    }*/
    char rollDir = 0;
    unsigned long tMglOff = 0, tRollersOff = 0;
    double mglHoldAngle = 0;
    bool mglPidRunning = false;

    printf("running\n");
    while (true) {
        // printEnc();
        updateLift();
        //----- mobile-goal lift -----//
        if (joystickGetDigital(1, 8, JOY_RIGHT) || joystickGetDigital(1, 6, JOY_UP)) {
            mglPidRunning = true;
            mglHoldAngle = MGL_UP_POS;
        } else if (joystickGetDigital(1, 8, JOY_LEFT) || joystickGetDigital(1, 5, JOY_UP)) {
            mglPidRunning = true;
            mglHoldAngle = MGL_MID_POS - 10;
        } else if (joystickGetDigital(1, 8, JOY_UP)) {
            tMglOff = millis();
            mglPidRunning = false;
            setMGL(-127);
        } else if (joystickGetDigital(1, 8, JOY_DOWN)) {
            tMglOff = millis();
            mglPidRunning = false;
            setMGL(127);
        } else if (joystickGetDigital(1, 6, JOY_DOWN)) {
            mglPidRunning = true;
            mglHoldAngle = MGL_DOWN_POS;
        } else if (joystickGetDigital(1, 7, JOY_DOWN) || joystickGetDigital(1, 5, JOY_DOWN)) {
            setDownStack();
            tMglOff = millis();
            mglPidRunning = false;
        } else if (!mglPidRunning && millis() - tMglOff > 200) {
            if (mglGet() <= MGL_MAX && mglGet() >= MGL_MIN) {
                mglPidRunning = true;
                mglHoldAngle = mglGet();
            }
        } else if (!mglPidRunning) {
            setMGL(0);
        }
        if (mglPidRunning) {
            if (mglHoldAngle <= 8 && mglGet() <= 8) {
                setMGL(-80);
            } else if (mglHoldAngle >= MGL_DOWN_POS - 8 && mglGet() >= MGL_DOWN_POS - 8) {
                setMGL(80);
            } else {
                pidMGL(mglHoldAngle, 999999);
            }
        }
        //----- rollers -----//
        if (joystickGetDigital(2, 8, JOY_LEFT) || joystickGetDigital(2, 8, JOY_UP) || joystickGetDigital(2, 8, JOY_RIGHT) || joystickGetDigital(2, 8, JOY_DOWN)) {
            // stop rollers
            rollDir = 0;
            setRollers(0);
        } else if (joystickGetDigital(2, 6, JOY_UP)) {
            // intake
            rollDir = 1;
            tRollersOff = millis();
            setRollers(90);
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
        opctrlDrive();
        delay(20);
    }
}
