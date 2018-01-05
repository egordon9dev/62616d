#include "auto.h"
#include "main.h"
#include "pid.h"
#include "setup.h"
//----- updates Arm and Chain-Bar -----//
unsigned long tFbOff = 0, tDrfbOff = 0;
double fbHoldAngle = 0, drfbHoldAngle = 0;
bool fbPidRunning = false, drfbPidRunning = false;
bool returning = false;
// MUST BE EITHER 0 or 1 !!!!!!!
const uint8_t DM = 1;

void updateLift() {
    if (DM == 2) return;
    //------ update four bar -----//
    bool moving = true;
    if (DM == 0) {
        if (joystickGetDigital(1, 5, JOY_UP)) {
            setFB(127);
            tFbOff = millis();
            fbPidRunning = false;
            returning = false;
        } else if (joystickGetDigital(1, 5, JOY_DOWN)) {
            setFB(-127);
            tFbOff = millis();
            fbPidRunning = false;
            returning = false;
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
        } else {
            moving = false;
        }
        if (joystickGetDigital(1, 8, JOY_UP)) {
            fbHoldAngle = 141;
        } else if (joystickGetDigital(1, 8, JOY_RIGHT)) {
            fbHoldAngle = 40;
        } else if (joystickGetDigital(1, 8, JOY_DOWN)) {
            returning = true;
        }
    }
    if (!moving) {
        if (returning) {
            returnLift(false);
        } else if (millis() - tFbOff > 300) {
            if (!fbPidRunning) {
                fbHoldAngle = fbGet();
                fbPidRunning = true;
            }
            fb_pid.sensVal = fbGet();
            int cut = FB_MIN_CUT - drfbGet();
            if (fbHoldAngle < cut) fbHoldAngle = cut;
            fb_pid.target = fbHoldAngle;
            setFB(updatePID(&fb_pid));
        } else {
            setFB(0);
            fbPidRunning = false;
        }
    }
    const int t = 15;
    int j3 = DM == 1 ? joystickGetAnalog(1, 3) : joystickGetAnalog(1, 2);
    if (abs(j3) > t) {
        tDrfbOff = millis();
        drfbPidRunning = false;
        returning = false;
        setDRFB(j3);
    } else if (millis() - tDrfbOff > 300) {
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
void test(int n) {
    switch (n) {
        case 0:
            resetDrive();
            while (!pidDrive(20, 20000)) {
                printEnc_pidDrive();
                delay(20);
            }
            resetDrive();
            break;
        case 1:
            resetDrive();
            while (!pidTurn(-5, 20000)) {
                printEnc_pidDrive();
                delay(20);
            }
            resetDrive();
            break;
        case 2:
            while (true) {
                printEnc_pidDRFBFB();
                fb_pid.target = 140;
                fb_pid.sensVal = fbGet();
                int p = updatePID(&fb_pid);
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
    while (true) {
        delay(999);
    }
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
    // delay(2000);
    if (autonMode == nAutons + nSkills) {
        // auton1(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &drfb_pid, &fb_pid, false, true);
    }
    char rollDir = 0;
    while (0) {
        printEnc();
        delay(20);
    }
    test(1);
    // auton1();
    unsigned long tMglOff = 0;
    double mglHoldAngle = 0;
    bool mglPidRunning = false;

    PidVars DL_brake = pidDef, DR_brake = pidDef;
    DL_brake.kd = 20;
    DR_brake.kd = 20;
    unsigned long dt = 0, prevT = 0;
    int DL_brake_out = 0, DR_brake_out = 0;
    while (true) {
        printEnc();
        lcdPrint(LCD, 1, "%d", yawGet());
        updateLift();
        //----- mobile-goal lift -----//
        if (joystickGetDigital(DM == 1 ? 2 : 1, 8, JOY_RIGHT) || (DM != 0 && joystickGetDigital(DM == 1 ? 2 : 1, 6, JOY_UP))) {
            mglHoldAngle = 0;
            mglPidRunning = true;
            tMglOff = 0;
        } else if (DM != 0 && joystickGetDigital(DM == 1 ? 2 : 1, 5, JOY_DOWN)) {
            mglHoldAngle = 93;
            mglPidRunning = true;
            tMglOff = 0;
        } else if (joystickGetDigital(DM == 1 ? 2 : 1, 8, JOY_UP) || (DM != 0 && joystickGetDigital(DM == 1 ? 2 : 1, 5, JOY_UP))) {
            mglHoldAngle = mglGet();
            tMglOff = millis();
            mglPidRunning = false;
            setMGL(-127);
        } else if (joystickGetDigital(DM == 1 ? 2 : 1, 8, JOY_DOWN) || (DM != 0 && joystickGetDigital(DM == 1 ? 2 : 1, 6, JOY_DOWN))) {
            mglHoldAngle = mglGet();
            tMglOff = millis();
            mglPidRunning = false;
            setMGL(127);
        } else if (millis() - tMglOff > 250) {
            if (!mglPidRunning) {
                mglPidRunning = true;
                mglHoldAngle = mglGet();
            }
            pidMGL(mglHoldAngle, 0);
        } else {
            mglPidRunning = false;
            setMGL(0);
        }
        if (DM != 2) {
            //----- rollers -----//
            if ((DM == 1 && (joystickGetDigital(1, 5, JOY_UP) || joystickGetDigital(1, 5, JOY_DOWN))) || (DM == 0 && joystickGetDigital(1, 8, JOY_LEFT))) {
                // stop rollers
                rollDir = 0;
                setRollers(0);
            } else if (joystickGetDigital(1, 6, JOY_UP)) {
                // intake
                rollDir = 1;
                setRollers(55);
            } else if (joystickGetDigital(1, 6, JOY_DOWN)) {
                // outtake
                rollDir = -1;
                setRollers(-90);
            } else {
                if (rollDir == 1) {
                    setRollers(25);
                } else if (rollDir == -1) {
                    setRollers(-50);
                } else {
                    setRollers(0);
                }
            }
        }
        //----- drive -----//
        const int td = 10;
        bool moving = true;
        DL_brake_out = ((DL_brake.prevSensVal - eDLGet()) * DL_brake.kd) / dt;
        DR_brake_out = ((DR_brake.prevSensVal - eDRGet()) * DR_brake.kd) / dt;
        if (DM == 0) {
            int j3 = joystickGetAnalog(1, 3);
            int j4 = joystickGetAnalog(1, 4);
            if ((abs(j3) > td) || (abs(j4) > td)) {
                setDL(j3 + j4);
                setDR(j3 - j4);
            } else {
                moving = false;
            }
        } else {
            int j2 = joystickGetAnalog(DM == 1 ? 2 : 1, 2);
            int j3 = joystickGetAnalog(DM == 1 ? 2 : 1, 3);

            if ((abs(j2) > td) || (abs(j3) > td)) {
                setDL(j3);
                setDR(j2);
            } else {
                moving = false;
            }
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
