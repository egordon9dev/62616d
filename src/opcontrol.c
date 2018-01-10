#include "auto.h"
#include "main.h"
#include "pid.h"
#include "setup.h"
//----- updates Arm and Chain-Bar -----//
unsigned long tFbOff = 0, tDrfbOff = 0;
double fbHoldAngle = 0, drfbHoldAngle = 0;
bool fbPidRunning = false, drfbPidRunning = false, fbUp = false;
/*
stuff to reduce weight:
-replace MGL 5x5 with 2x5
-half MGL 2 2x25
-replace front MGL 2x15 with standoff

-maybe half the 2 2x15 and 2 2x10 on the fb tower
*/

/*
        fb,drfb     drive,mgl
Joy :   1           2
0   :   Erik        ----
1   :   Rahul       Buelah
2   :   Rahul       Erik
3   :   Erik        Buelah
*/
const uint8_t DM = 0;
unsigned long opT0;
void updateLift() {
    static bool returning = false;
    //------ update four bar -----//
    bool moving = true;
    if (DM == 0 || DM == 3) {
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
    } else {
        int j2 = joystickGetAnalog(1, 2);
        if (abs(j2) > 15) {
            setFB(j2);
            tFbOff = millis();
            fbPidRunning = false;
            returning = false;
            fbUp = false;
        } else {
            moving = false;
        }
    }
    if (!moving) {
        if (((DM == 1 || DM == 2) && joystickGetDigital(1, 8, JOY_UP)) || ((DM == 0 || DM == 3) && joystickGetDigital(1, 5, JOY_UP))) {
            tFbOff = 0;
            fbPidRunning = true;
            returning = false;
            fbUp = true;
        } else if (((DM == 1 || DM == 2) && joystickGetDigital(1, 8, JOY_RIGHT)) || ((DM == 0 || DM == 3) && joystickGetDigital(1, 5, JOY_DOWN))) {
            fbHoldAngle = 40;
            tFbOff = 0;
            fbPidRunning = true;
            returning = false;
            fbUp = false;
        } else if (((DM == 1 || DM == 2) && joystickGetDigital(1, 8, JOY_DOWN)) || ((DM == 0 || DM == 3) && joystickGetDigital(1, 7, JOY_LEFT)) || returning) {
            if (!returning) {
                startReturnLift(true);
                returning = true;
            } else {
                contReturnLift(true, 9999999);
                fbHoldAngle = fbGet();
                drfbHoldAngle = drfbGet();
                tFbOff = 0;
                tDrfbOff = 0;
            }
            fbUp = false;
        }
        if (!returning) {
            if (millis() - tFbOff > 300) {
                if(fbUp) {
                    int drfbA = drfbGet();
                    if(drfbA < 0) drfbA = 0;
                    fbHoldAngle = 128 + drfbA * 0.0777;  // 0:138 (0), 4:140, 8:143, 11:146 (103)
                }
                if (!fbPidRunning) {
                    fbHoldAngle = fbGet();
                    fbPidRunning = true;
                }
                PidVars *pid = &fb_pid_auto;
                pid->sensVal = fbGet();
                if (fbHoldAngle < FB_MIN_CUT) fbHoldAngle = FB_MIN_CUT;
                pid->target = fbHoldAngle;
                setFB(updatePID(pid));
            } else {
                setFB(0);
                fbPidRunning = false;
            }
        }
    }
    const int t = 15;
    int js = DM == 1 || DM == 2 ? joystickGetAnalog(1, 3) : joystickGetAnalog(1, 2);
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
            resetDrive();
            while (!pidDrive(-10, 20000)) {
                printEnc_pidDrive();
                delay(20);
            }
            resetDrive();
            break;
        case 1:
            resetDrive();
            while (!pidTurn(5, 20000)) {
                printEnc_pidDrive();
                delay(20);
            }
            resetDrive();
            break;
        case 2:
            while (true) {
                printEnc_pidDRFBFB();
                fb_pid_auto.target = 138;
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
    opT0 = millis();
    // delay(2000);
    /*if (autonMode == nAutons + nSkills) {
        auton1(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &drfb_pid, &fb_pid, false, true);
    }*/
    char rollDir = 0;
    while (0) {
        printEnc();
        delay(20);
    }
    //test(2);
    // auton1();
    unsigned long tMglOff = 0, tRollersOff = 0;
    double mglHoldAngle = 0;
    bool mglPidRunning = false;

    PidVars DL_brake = pidDef, DR_brake = pidDef;
    DL_brake.kd = 20;
    DR_brake.kd = 20;
    unsigned long dt = 0, prevT = 0;
    int DL_brake_out = 0, DR_brake_out = 0;
    while (true) {
        // printEnc();
        printEnc_pidDRFBFB();
        lcdPrint(LCD, 1, "%d", yawGet());
        updateLift();
        //----- mobile-goal lift -----//
        if (joystickGetDigital(DM == 0 ? 1 : 2, 8, JOY_RIGHT) || ((DM == 1 || DM == 3) && joystickGetDigital(2, 6, JOY_UP))) {
            mglHoldAngle = 4;
            mglPidRunning = true;
            tMglOff = 0;
        } else if (joystickGetDigital(DM == 0 ? 1 : 2, 8, JOY_LEFT) || ((DM == 1 || DM == 3) && joystickGetDigital(2, 5, JOY_DOWN))) {
            mglHoldAngle = 75;
            mglPidRunning = true;
            tMglOff = 0;
        } else if (joystickGetDigital(DM == 0 ? 1 : 2, 8, JOY_UP) || ((DM == 1 || DM == 3) && joystickGetDigital(2, 5, JOY_UP))) {
            tMglOff = millis();
            setMGL(-127);
        } else if ((DM == 1 || DM == 3) && joystickGetDigital(2, 8, JOY_DOWN)) {
            tMglOff = millis();
            setMGL(127);
        } else if (((DM == 1 || DM == 3) && joystickGetDigital(2, 6, JOY_DOWN)) || ((DM == 0 || DM == 2) && joystickGetDigital(DM == 0 ? 1 : 2, 8, JOY_DOWN))) {
            mglHoldAngle = 120;
            mglPidRunning = true;
            tMglOff = 0;
        } else if (millis() - tMglOff > 250 || tMglOff == 0) {
            if (!mglPidRunning) {
                mglPidRunning = true;
                mglHoldAngle = mglGet();
            }
            pidMGL(mglHoldAngle, 0);
        } else {
            mglPidRunning = false;
            setMGL(0);
        }
        //----- rollers -----//
        if (((DM == 1 || DM == 2) && (joystickGetDigital(1, 5, JOY_UP) || joystickGetDigital(1, 5, JOY_DOWN))) || ((DM == 0 || DM == 3) && joystickGetDigital(1, 7, JOY_RIGHT))) {
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
        const int td = 10;
        bool moving = true;
        DL_brake_out = ((DL_brake.prevSensVal - eDLGet()) * DL_brake.kd) / dt;
        DR_brake_out = ((DR_brake.prevSensVal - eDRGet()) * DR_brake.kd) / dt;
        if (DM == 0 || DM == 2) {
            int j3 = joystickGetAnalog(DM == 0 ? 1 : 2, 3);
            int j4 = joystickGetAnalog(DM == 0 ? 1 : 2, 4);
            if (abs(j3) < td) j3 = 0;
            if (abs(j4) < td) j4 = 0;
            if (j3 == 0 && j4 == 0) moving = false;
            setDL(j3 + j4);
            setDR(j3 - j4);
        } else {
            int j2 = joystickGetAnalog(2, 2), j3 = joystickGetAnalog(2, 3);
            if (abs(j2) < td) j2 = 0;
            if (abs(j3) < td) j3 = 0;
            if (j2 == 0 && j3 == 0) moving = false;
            setDL(j3);
            setDR(j2);
        }
        if (!moving) {
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
