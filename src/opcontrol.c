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
uint8_t DM = 1;

void updateLift(PidVars *drfb_pid, PidVars *fb_pid) {
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
            returnLift(drfb_pid, fb_pid);
        } else if (millis() - tFbOff > 300) {
            if (!fbPidRunning) {
                fbHoldAngle = fbGet();
                fbPidRunning = true;
            }
            fb_pid->sensVal = fbGet();
            fb_pid->target = fbHoldAngle;
            setFB(updatePID(fb_pid));
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
        drfb_pid->sensVal = drfbGet();
        if (drfbHoldAngle > DRFB_MAX_CUT) drfbHoldAngle = DRFB_MAX_CUT;
        drfb_pid->target = drfbHoldAngle;
        setDRFB(updatePID(drfb_pid));
    } else {
        setDRFB(0);
        drfbPidRunning = false;
    }
}
// for testing auton pid
int autonDrive(double dist, int wait, PidVars *left, PidVars *right);
int autonTurn(double angle, int wait, PidVars *pid);
void test(int n) {
    switch (n) {
        case 0:
            resetDrive(&DL_pid, &DR_pid, &turn_pid);
            while (!autonTurn(-5, 20000, &turn_pid)) {
                printEnc_pidDrive(&DL_pid, &DR_pid, &turn_pid);
                delay(10);
            }
            resetDrive(&DL_pid, &DR_pid, &turn_pid);
            break;
        case 1:
            while (true) {
                printEnc_pidDRFBFB(&drfb_pid, &fb_pid);
                fb_pid.target = 120;
                fb_pid.sensVal = fbGet();
                int p = updatePID(&fb_pid);
                setFB(p);
                printf("power: %d\t", p);
                delay(10);
            }
            break;
        case 2:
            while (true) {
                printEnc_pidDRFBFB(&drfb_pid, &fb_pid);
                drfb_pid.target = 5;
                drfb_pid.sensVal = drfbGet();
                int p = updatePID(&drfb_pid);
                setDRFB(p);
                printf("power: %d\t", p);
                delay(10);
            }
            break;
    }
}
void operatorControl() {
    if (autonMode == nAutons + nSkills) {
        // auton1(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &drfb_pid, &fb_pid, false, true);
    }
    bool clawOpen = false;
    long tClawOpen = millis();
    while (0) {
        printEnc();
        delay(10);
    }
    if (0) {
        test(2);
    }
    unsigned long tMglOff = 0;
    double mglHoldAngle = 0;
    bool mglPidRunning = false;
    DM = 0;
    for (int i = 0; i < 10; i++) {
        if (isJoystickConnected(1) && isJoystickConnected(2)) {
            DM = 1;
        }
        delay(1);
    }
    while (true) {
        printEnc();
        lcdPrint(LCD, 1, "%d", yawGet());
        updateLift(&drfb_pid, &fb_pid);
        //----- mobile-goal lift -----//
        if (joystickGetDigital(DM + 1, 8, JOY_RIGHT) || (DM == 1 && joystickGetDigital(2, 6, JOY_UP))) {
            mglHoldAngle = 0;
            mglPidRunning = true;
            tMglOff = 0;
        } else if (DM == 1 && joystickGetDigital(2, 5, JOY_DOWN)) {
            mglHoldAngle = 93;
            mglPidRunning = true;
            tMglOff = 0;
        } else if (joystickGetDigital(DM + 1, 8, JOY_UP) || (DM == 1 && joystickGetDigital(2, 5, JOY_UP))) {
            mglHoldAngle = mglGet();
            tMglOff = millis();
            mglPidRunning = false;
            setMGL(-127);
        } else if (joystickGetDigital(DM + 1, 8, JOY_DOWN) || (DM == 1 && joystickGetDigital(2, 6, JOY_DOWN))) {
            mglHoldAngle = mglGet();
            tMglOff = millis();
            mglPidRunning = false;
            setMGL(127);
        } else if (millis() - tMglOff > 250) {
            if (!mglPidRunning) {
                mglPidRunning = true;
                mglHoldAngle = mglGet();
            }
            pidMGL(&mgl_pid, mglHoldAngle, 0);
        } else {
            mglPidRunning = false;
            setMGL(0);
        }
        //----- claw -----//
        if (joystickGetDigital(1, 6, JOY_DOWN)) {
            // open
            clawOpen = true;
            tClawOpen = millis();
            setClaw(70);
        } else if (joystickGetDigital(1, 6, JOY_UP)) {
            // close
            clawOpen = false;
            setClaw(-70);
        } else {
            if (clawOpen == true) {
                if (millis() - tClawOpen < 300) {
                    setClaw(10);  //+
                } else {
                    setClaw(0);  //+
                }
            } else {
                setClaw(-15);  //-
            }
        }
        //----- drive -----//
        const int td = 15;
        bool moving = true;
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
            int j2 = joystickGetAnalog(2, 2);
            int j3 = joystickGetAnalog(2, 3);
            if ((abs(j2) > td) || (abs(j3) > td)) {
                setDL(j3);
                setDR(j2);
            } else {
                moving = false;
            }
        }
        if (!moving) {
            setDL(0);
            setDR(0);
        }
        delay(10);
    }
}
