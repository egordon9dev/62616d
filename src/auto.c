#include "main.h"
#include "pid.h"
#include "setup.h"
// 15, 20
void auton1(bool leftSide, bool stack2) {
    int yaw = 0;
    unsigned long t0 = millis();
    unsigned long funcT0 = millis();
    int i = 0;
    double prevSens[3] = {0, 0, 0};
    while (millis() - funcT0 < 15000) {
        int j = 0;
        if (i == j++) {  // deploy
            if (millis() - t0 < 100) {
                setRollers(50);
            } else {
                setRollers(20);
            }
            if (fbGet() > 15) {
                setMGL(127);
                pidDRFB(20, 999999, true);
            }
            pidFB(FB_UP_POS, 999999, true);
            if (mglGet() > 85) i++;
        } else if (i == j++) {  // grab MG
            pidMGL(MGL_DOWN_POS, 999999);
            if (stack2) {
                pidFB(FB_UP_POS + 8, 999999, true);
                pidDRFB(15, 999999, true);
            } else {
                pidFB(FB_MID_POS, 999999, true);
                pidDRFB(35, 999999, true);
            }
            if (pidDrive(54, 200)) {
                mgl_pid.doneTime = LONG_MAX;
                if (stack2) resetDriveEnc();
                setDL(0);
                setDR(0);
                t0 = millis();
                i++;
            }
        } else if (i == j++) {  // lift MG
            if (stack2) {
                if (mglGet() < 80) pidDrive(8, 999999);
                pidFB(FB_UP_POS + 8, 999999, true);
                pidDRFB(15, 999999, true);
                if (pidMGL(MGL_UP_POS, 0)) { i++; }
            } else {
                pidFB(FB_MID_POS, 999999, true);
                pidDRFB(35, 999999, true);
                if (millis() - t0 > 100) pidMGL(MGL_UP_POS, 999999);
                setRollers(-80);
                if (mglGet() < 100) {
                    setRollers(0);
                    if (!stack2) {
                        resetDriveEnc();
                        DL_pid.doneTime = LONG_MAX;
                        DR_pid.doneTime = LONG_MAX;
                    }
                    i++;
                }
            }
        } else if (i == j++) {  // stack cone 1
            if (stack2) {
                pidDrive(8, 999999);
                pidFB(FB_UP_POS + 8, 999999, true);
                if (drfbGet() < 10) {
                    setRollers(-90);
                    i++;
                }
                if (pidDRFB(0, 999999, true)) i++;
                fb_pid_auto.doneTime = LONG_MAX;
            } else {
                i++;
            }
        } else if (i == j++) {  // hover lift over cone 2
            if (stack2) {
                pidDrive(8, 999999);
                pidDRFB(15, 999999, true);
                if (drfbGet() > 8) {
                    pidFB(FB_MID_POS - 5, 999999, true);
                    if (fbGet() < FB_MID_POS + 15) { i++; }
                } else {
                    pidFB(FB_UP_POS + 3, 999999, true);
                }
            } else {
                i++;
            }
        } else if (i == j++) {  // grab cone 2
            if (stack2) {
                pidDrive(8, 999999);
                setRollers(70);
                pidFB(FB_MID_POS - 5, 999999, true);
                pidDRFB(0, 999999, true);
                if (drfbGet() < 3 && fbGet() < FB_MID_POS + 4) {
                    resetDriveEnc();
                    DL_pid.doneTime = LONG_MAX;
                    DR_pid.doneTime = LONG_MAX;
                    t0 = millis();
                    i++;
                }
            } else {
                i++;
            }
        } else if (i == j++) {  // lift cone 2
            if (stack2) {
                pidDrive(-55, 200);
                pidDRFB(22, 999999, true);
                if (millis() - t0 > 300) setRollers(20);
                if (drfbGet() > 15) {
                    pidFB(FB_UP_POS, 999999, true);
                    if (fbGet() > FB_UP_POS - 10) {
                        drfb_pid_auto.doneTime = LONG_MAX;
                        setRollers(20);
                        i++;
                    }
                } else {
                    pidFB(FB_MID_POS, 999999, true);
                }
            } else {
                i++;
            }
        } else if (i == j++) {  // stack cone 2
            if (stack2) {
                pidDrive(-55, 200);
                if (drfbGet() < 5) setRollers(-50);
                pidFB(FB_UP_POS, 999999, true);
                if (pidDRFB(0, 0, true)) {
                    drfb_pid_auto.doneTime = LONG_MAX;
                    i++;
                }
            } else {
                i++;
            }
        } else if (i == j++) {
            pidFB(FB_UP_POS, 999999, true);
            pidDRFB(20, 999999, true);
            pidMGL(MGL_UP_POS, 999999);
            if (pidDrive(stack2 ? -55 : -47, 200)) {
                resetDriveEnc();
                turn_pid.doneTime = LONG_MAX;
                i++;
                yaw += leftSide ? 36 : -36;
            }
        } else if (i == j++) {
            setFB(0);
            setRollers(0);
            pidDRFB(22, 999999, true);
            if (pidTurn(yaw, 200)) {
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                i++;
            }
        } else if (i == j++) {
            pidDRFB(20, 999999, true);
            if (pidDrive(-22, 200)) {
                resetDriveEnc();
                turn_pid.doneTime = LONG_MAX;
                i++;
                yaw += leftSide ? 90 : -90;
            }
        } else if (i == j++) {
            pidDRFB(20, 999999, true);
            pidMGL(MGL_MID_POS - 40, 999999);
            if (pidTurn(yaw, 200)) {
                resetDriveEnc();
                i++;
            }
        } else if (i == j++) {  // score MG
            pidDrive(33, 999999);
            prevSens[0] = prevSens[1];
            prevSens[1] = prevSens[2];
            prevSens[2] = eDLGet() + eDRGet();
            pidDRFB(20, 999999, true);
            if (eDLGet() + eDRGet() < 40) {
                pidMGL(MGL_MID_POS - 40, 999999);
            } else {
                pidMGL(MGL_MID_POS, 999999);
            }
            if (eDLGet() + eDRGet() - (prevSens[0] + prevSens[1] + prevSens[2]) / 3.0 <= 6 && eDLGet() + eDRGet() > 40) {
                i++;
                resetDriveEnc();
                t0 = millis();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } /*else if (i == j++) {
            pidDRFB(0, 999999, true);
            pidMGL(MGL_MID_POS + 18, 999999);
            setDL(20);
            setDR(20);
            if (mglGet() > MGL_MID_POS + 4) {
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                t0 = millis();
                i++;
            }
        } */
        else if (i == j++) {
            setDRFB(0);
            if (millis() - t0 < 100) {
                pidMGL(MGL_MID_POS + 18, 999999);
            } else {
                pidMGL(MGL_MID_POS, 999999);
            }
            if (pidDrive(-24, 200)) i++;
        } else if (i == j++) {
            break;
        }
        printEnc_all();
        delay(20);
    }
    resetMotors();
    printf("\n\nTIME: %lf\n", (millis() - funcT0) / 1000.0);
}

void skills() {
    int i = 0;
    unsigned long funcT0 = millis(), t0 = millis();
    while (millis() - funcT0 < 15000) {
        int j = 0;
        if (i == j++) {  // deploy
            if (millis() - t0 < 100) {
                setRollers(50);
            } else {
                setRollers(20);
            }
            if (fbGet() > 15) {
                setMGL(127);
                pidDRFB(20, 999999, true);
            }
            pidFB(FB_UP_POS, 999999, true);
        } else if (i == j++) {  // d 28
        } else if (i == j++) {  // t
        } else if (i == j++) {
        } else if (i == j++) {
        } else if (i == j++) {
        } else if (i == j++) {
        } else if (i == j++) {
        } else if (i == j++) {
        } else if (i == j++) {
            break;
        }
        printEnc_all();
        delay(20);
    }
    resetMotors();
    printf("\n\nTIME: %lf\n", (millis() - funcT0) / 1000.0);
}
/*
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. If the robot is disabled or communications is lost, the autonomous task
 * will be stopped by the kernel. Re-enabling the robot will restart the task,
 * not re-start it from where it left off.
 *
 * Code running in the autonomous task cannot access information from the VEX
 * Joystick. However, the autonomous function can be invoked from another task
 * if a VEX Competition Switch is not available, and it can access joystick
 * information if called in this way.
 *
 * The autonomous task may exit, unlike operatorControl() which should never
 * exit. If it does so, the robot will await a switch to another mode or
 * disable/enable cycle.
 */
void autonomous() {
    resetDriveEnc();
    auton1(true, false);
}
