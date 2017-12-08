#include "auto.h"
#include "main.h"
#include "pid.h"
#include "setup.h"
//----- updates Arm and Chain-Bar -----//
unsigned long tFbOff = 0, tDrfbOff = 0;
double fbHoldAngle = 0, drfbHoldAngle = 0;
bool fbPidRunning = false, drfbPidRunning = false;
bool returning = false;
void updateLift(PidVars *drfb_pid, PidVars *fb_pid) {
    //----- update arm -----//
    if (joystickGetDigital(1, 7, JOY_RIGHT)) {
        returning = true;
    }
    if (joystickGetDigital(1, 7, JOY_LEFT)) {
        returning = false;
    }
    if (joystickGetDigital(1, 7, JOY_UP)) {
        // insert auto stack code here..............
        // c .........................................
    }
    //------ update four bar -----//
    if (joystickGetDigital(1, 5, JOY_UP)) {
        setFB(-127);
        tFbOff = millis();
        fbPidRunning = false;
        returning = false;
    } else if (joystickGetDigital(1, 5, JOY_DOWN)) {
        setFB(127);
        tFbOff = millis();
        fbPidRunning = false;
        returning = false;
    } else {
        if (returning) {
            returnLift(drfb_pid, fb_pid);
            return;
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
    int j2 = joystickGetAnalog(1, 2);
    if (abs(j2) > t) {
        tDrfbOff = millis();
        drfbPidRunning = false;
        returning = false;
        setDRFB(j2);
    } else if (millis() - tDrfbOff > 300) {
        if (!drfbPidRunning) {
            drfbHoldAngle = drfbGet();
            drfbPidRunning = true;
        }
        drfb_pid->sensVal = drfbGet();
        drfb_pid->target = drfbHoldAngle;
        setDRFB(updatePID(drfb_pid));
    } else {
        setDRFB(0);
        drfbPidRunning = false;
    }
}
// for testing auton pid
int autonDrive(double dist, int wait, PidVars *left, PidVars *right, bool turning);
void test() {
    resetDrive(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid);
    while (!autonDrive(90, 20000, &DLturn_pid, &DRturn_pid, true)) {
        printEnc_pidDrive(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid);
        delay(20);
    }
}
void operatorControl() {
    if (autonMode == nAutons + nSkills) {
        //auton1(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &drfb_pid, &fb_pid, false, true);
    }
    bool clawOpen = false;
    long tClawOpen = millis();
    bool mglAutoUp = false;

    while (true) {
        printEnc();
        updateLift(&drfb_pid, &fb_pid);
        //----- mobile-goal lift -----//
        if (joystickGetDigital(1, 8, JOY_RIGHT)) {
            mglAutoUp = true;
        } else if (joystickGetDigital(1, 8, JOY_UP)) {
            mglAutoUp = false;
            setMGL(-127);
        } else if (joystickGetDigital(1, 8, JOY_DOWN)) {
            setMGL(127);
            mglAutoUp = false;
        } else if (mglAutoUp) {
            setMGL(-127);  // automatically continue lifting mobile goal
        } else {
            setMGL(0);
        }
        //----- claw -----//
        if (joystickGetDigital(1, 6, JOY_DOWN)) {
            // close
            clawOpen = false;
            setClaw(-70);
        } else if (joystickGetDigital(1, 6, JOY_UP)) {
            // open
            clawOpen = true;
            tClawOpen = millis();
            setClaw(70);
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
        int j3 = joystickGetAnalog(1, 3);
        int j4 = joystickGetAnalog(1, 4);
        if ((abs(j3) > td) || (abs(j4) > td)) {
            setDL(j3 + j4);
            setDR(j3 - j4);
        } else {
            setDL(0);
            setDR(0);
        }

        delay(20);
    }
}
