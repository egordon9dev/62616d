#include "auto.h"
#include "main.h"
#include "pid.h"
#include "setup.h"
//----- updates Arm and four-Bar -----//
unsigned long tFbOff = 0, tDrfbOff = 0;
double fbHoldAngle = 0, drfbHoldAngle = 0;
bool fbPidRunning = false, drfbPidRunning = false;
/*
todo:
-put line follower under cortex
*/

/*
##     ## ########  ########     ###    ######## ########    ##       #### ######## ########
##     ## ##     ## ##     ##   ## ##      ##    ##          ##        ##  ##          ##
##     ## ##     ## ##     ##  ##   ##     ##    ##          ##        ##  ##          ##
##     ## ########  ##     ## ##     ##    ##    ######      ##        ##  ######      ##
##     ## ##        ##     ## #########    ##    ##          ##        ##  ##          ##
##     ## ##        ##     ## ##     ##    ##    ##          ##        ##  ##          ##
 #######  ##        ########  ##     ##    ##    ########    ######## #### ##          ##
*/
unsigned long opT0;
bool prev7u = false, prev7d = false, liftingDrfbMgl = false; /*
 unsigned long prevFbT = 0;
 double prevFb = -1;*/
void updateLift() {                                          /*
                                              if (curSetDownStack) return;
                                              double fbDt = millis() - prevFbT, fbVel;
                                              prevFbT = millis();
                                              double curFb = fbGet();
                                              if (fbDt == 0 || fbDt > 30 || prevFb < 0) {
                                                  fbVel = 0.0;
                                              } else {
                                                  fbVel = (curFb - prevFb) / fbDt;
                                              }
                                              prevFb = curFb;*/

    lcdPrint(LCD, 1, "fb %d drfb %d", (int)fbGet(), (int)drfbGet());
    if (joystickGetDigital(2, 8, JOY_DOWN)) {
        DL_slew.a = 0.3;
        DR_slew.a = 0.3;
        DRIVE_DRIVE_MAX = 80;
        DRIVE_TURN_MAX = 75;

    } else {
        DL_slew.a = 1;
        DR_slew.a = 1;
        DRIVE_DRIVE_MAX = 110;
        DRIVE_TURN_MAX = 90;
    }
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
    prev7u = cur7u;
    prev7d = cur7d;
    if (joystickGetDigital(2, 8, JOY_UP)) {
        autoStack(1, 12);
        fbHoldAngle = FB_UP_POS;
        fbPidRunning = true;
        drfbHoldAngle = drfbGet();
        drfbPidRunning = true;
    } else {
        autoStacking = false;
        if (abs(j3) > JOY_THRESHOLD) {
            setFB(j3);
            tFbOff = millis();
            fbPidRunning = false;
        } else if (joystickGetDigital(2, 6, JOY_DOWN)) {
            setFB(-127);
            tFbOff = millis();
            fbPidRunning = false;
        } else if (joystickGetDigital(2, 5, JOY_UP)) {
            fbHoldAngle = FB_UP_POS;
            fbPidRunning = true;
        } else if (joystickGetDigital(2, 5, JOY_DOWN)) {
            fbHoldAngle = FB_MID_POS;
            fbPidRunning = true;
        } else if (joystickGetDigital(2, 6, JOY_UP)) {
            fbHoldAngle = FB_HALF_UP_POS;
            fbPidRunning = true;
        } else if (joystickGetDigital(2, 7, JOY_LEFT)) {
            fbHoldAngle = FB_FLIP_POS;
            fbPidRunning = true;
            drfbHoldAngle = DRFB_FLIP_POS;
            drfbPidRunning = true;
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
            fb_pid.sensVal = fbGet();
            fb_pid.target = fbHoldAngle;
            setFB(updatePID(&fb_pid));
        }
        int js = joystickGetAnalog(2, 2) * DRFB_MAX / 127.0;
        if (abs(js) > JOY_THRESHOLD) {
            tDrfbOff = millis();
            drfbPidRunning = false;
            liftingDrfbMgl = false;
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
            if (liftingDrfbMgl) {
                if (drfbGet() < DRFB_MGL_ACTIVE + 5) {
                    setDRFB(127);
                    drfbHoldAngle = DRFB_MGL_ACTIVE + 5;
                } else {
                    liftingDrfbMgl = false;
                }
            }
            // don't combine these !!!!!!!
            if (!liftingDrfbMgl) pidDRFB(drfbHoldAngle, 999999, false);
        }
    }
}
void test(int n) {
    switch (n) {
        case 0:
            while (true) {
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                while (!pidDrive(12, 100)) {
                    printEnc_pidDrive();
                    delay(5);
                }
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                while (!pidDrive(-12, 100)) {
                    printEnc_pidDrive();
                    delay(5);
                }
            }
            break;
        case 1:
            DLturn_pid.doneTime = LONG_MAX;
            DRturn_pid.doneTime = LONG_MAX;
            resetDriveEnc();
            while (!pidTurn(90, 100)) {
                printEnc_pidDrive();
                delay(5);
            }
            DLturn_pid.doneTime = LONG_MAX;
            DRturn_pid.doneTime = LONG_MAX;
            resetDriveEnc();
            while (!pidTurn(-90, 100)) {
                printEnc_pidDrive();
                delay(5);
            }
            break;
        case 2:
            while (true) {
                fb_pid_auto.doneTime = LONG_MAX;
                while (!pidFB(FB_UP_POS, 500, true)) {
                    printEnc_pidDRFBFB();
                    delay(5);
                }
                fb_pid_auto.doneTime = LONG_MAX;
                while (!pidFB(FB_MID_POS, 500, true)) {
                    printEnc_pidDRFBFB();
                    delay(5);
                }
            }
            break;
        case 3:
            while (true) {
                drfb_pid_auto.doneTime = LONG_MAX;
                while (!pidDRFB(30, 500, true)) {
                    printEnc_pidDRFBFB();
                    delay(5);
                }
                drfb_pid_auto.doneTime = LONG_MAX;
                while (!pidDRFB(60, 500, true)) {
                    printEnc_pidDRFBFB();
                    delay(5);
                }
            }
            break;
        case 4:
            while (true) {
                printEnc_all();
                pidMGL(MGL_MID_POS, 999999);
                delay(5);
            }
            break;
        case 5:
            while (true) {
                DLshort_pid.doneTime = LONG_MAX;
                DRshort_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                while (!pidDriveShort(6, 100)) {
                    printEnc_pidDrive();
                    delay(5);
                }
                DLshort_pid.doneTime = LONG_MAX;
                DRshort_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                while (!pidDriveShort(-6, 100)) {
                    printEnc_pidDrive();
                    delay(5);
                }
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
        delay(5);
    }
}
/*
 #######  ########   ######   #######  ##    ## ######## ########   #######  ##
##     ## ##     ## ##    ## ##     ## ###   ##    ##    ##     ## ##     ## ##
##     ## ##     ## ##       ##     ## ####  ##    ##    ##     ## ##     ## ##
##     ## ########  ##       ##     ## ## ## ##    ##    ########  ##     ## ##
##     ## ##        ##       ##     ## ##  ####    ##    ##   ##   ##     ## ##
##     ## ##        ##    ## ##     ## ##   ###    ##    ##    ##  ##     ## ##
 #######  ##         ######   #######  ##    ##    ##    ##     ##  #######  ########
*/
#include "auto.h"
void operatorControl() {
    if (1) {
        while (1) {
            pidMGL(MGL_DOWN_POS, 999999);
            printf("mgl %d mgl' %lf\n", (int)mglGet(), (mgl_pid.deriv) / (mgl_pid.kd));
            delay(5);
        }
        while (0) {
            lcdPrint(LCD, 1, "%d %d %d %d", joystickGetAnalog(1, 4), joystickGetAnalog(1, 3), joystickGetAnalog(1, 1), joystickGetAnalog(1, 2));
            lcdPrint(LCD, 2, "%d %d %d %d", joystickGetAnalog(2, 4), joystickGetAnalog(2, 3), joystickGetAnalog(2, 1), joystickGetAnalog(2, 2));
            delay(5);
        }
        while (1) {
            printEnc();
            delay(50);
        }
        if (1) {
            for (int i = 15; i > 0; i--) {
                delay(200);
                printf("%d\n", i);
            }
            auton4(true, 3, true, 20);
        }
        if (0) {
            autoStacking = false;
            while (autoStack(1, 12) == 0) {
                delay(5);
                printEnc_all();
            }
            while (true) delay(5);
        }
        if (0) { test(4); }
        if (0) {
            settingDownStack = false;
            while (!setDownStack()) {
                printEnc();
                delay(5);
            }
            resetMotors();
            while (true) delay(5);
        }
    }
    // shutdownSens();
    opT0 = millis();
    unsigned long tMglOff = 0;
    double mglHoldAngle = mglGet();
    drfbHoldAngle = drfbGet();
    fbHoldAngle = fbGet();
    bool mglPidRunning = false;
    printf("\n\nOPERATOR CONTROL\n\n");
    DL_slew.a = 1.0;
    DR_slew.a = 1.0;
    bool prevSetDownStack = false;
    while (true) {
        // printEnc();
        updateLift();
        //----- mobile-goal lift -----//
        if (joystickGetDigital(1, 8, JOY_RIGHT) || joystickGetDigital(1, 6, JOY_UP)) {
            mglPidRunning = true;
            mglHoldAngle = MGL_UP_POS;
            drfbPidRunning = true;
            liftingDrfbMgl = true;
        } else if (joystickGetDigital(1, 8, JOY_LEFT)) {
            mglPidRunning = true;
            mglHoldAngle = MGL_MID_POS - 10;
            drfbPidRunning = true;
            liftingDrfbMgl = true;
        } else if (joystickGetDigital(1, 8, JOY_UP)) {
            tMglOff = millis();
            mglPidRunning = false;
            setMGL(-127);
            drfbPidRunning = true;
            liftingDrfbMgl = true;
        } else if (joystickGetDigital(1, 8, JOY_DOWN)) {
            tMglOff = millis();
            mglPidRunning = false;
            setMGL(127);
            drfbPidRunning = true;
            liftingDrfbMgl = true;
        } else if (joystickGetDigital(1, 6, JOY_DOWN)) {
            mglPidRunning = true;
            mglHoldAngle = MGL_DOWN_POS;
            drfbPidRunning = true;
            liftingDrfbMgl = true;
        } else if (joystickGetDigital(1, 5, JOY_UP)) {
            mglPidRunning = false;
            curSetDownStack = false;
            tMglOff = LONG_MAX;
        } else if (joystickGetDigital(1, 5, JOY_DOWN) || curSetDownStack) {
            if (prevDrv < 80) {
                if (drfbGet() > DRFB_MGL_ACTIVE + 5 && drfbGet() > drfba[2][1] + 3 && !prevSetDownStack) {
                    settingDownStack = false;
                    curSetDownStack = true;
                }
                if (curSetDownStack && setDownStack()) curSetDownStack = false;
                mglPidRunning = true;
                mglHoldAngle = mglGet();
                fbPidRunning = true;
                fbHoldAngle = fbGet();
                drfbPidRunning = true;
                drfbHoldAngle = drfbGet();
            }
        } else if (!mglPidRunning && (long)millis() - (long)tMglOff > 350L) {
            mglPidRunning = true;
            mglHoldAngle = mglGet();
        } else if (!mglPidRunning) {
            if (tMglOff < LONG_MAX) {
                setMGL(0);
            } else {
                setMGL(-5);
            }
        }
        if (mglPidRunning && !curSetDownStack) { pidMGL(mglHoldAngle, 999999); }
        prevSetDownStack = curSetDownStack;
        opctrlDrive();
        delay(5);
    }
}
