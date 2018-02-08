#include "main.h"
#include "pid.h"
#include "setup.h"
// 15, 20
void auton1(bool leftSide, bool stack2) {
    unsigned long t0 = millis(), lastT = millis();
    unsigned long funcT0 = millis();
    int i = 0, prevI = 0;
    double prevSens[3] = {0, 0, 0};
    unsigned long breakTime = 4000;
    int driveT = 700;
    while (true) {
        if (i != prevI) lastT = millis();
        if (millis() - lastT > breakTime) break;
        prevI = i;
        int j = 0;
        if (i == j++) {  // deploy
            if (millis() - t0 < 200) {
                setRollers(70);
            } else {
                setRollers(25);
            }
            if (fbGet() > 15) {
                setMGL(127);
                pidDRFB(20, 999999, true);
            }
            pidFB(FB_UP_POS, 999999, true);
            if (mglGet() > 93) {
                i++;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // grab MG
            setRollers(25);
            if (pidMGL(MGL_DOWN_POS, 0)) setMGL(20);
            if (stack2) {
                pidFB(FB_UP_POS + 8, 999999, true);
                pidDRFB(15, 999999, true);
            } else {
                pidFB(FB_MID_POS, 999999, true);
                pidDRFB(40, 999999, true);
            }
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN >= 58) pidMGL(MGL_UP_POS, 999999);
            if (pidDrive(60, driveT)) {
                mgl_pid.doneTime = LONG_MAX;
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                if (stack2) resetDriveEnc();
                setDL(0);
                setDR(0);
                t0 = millis();
                i++;
            }
        } else if (i == j++) {  // lift MG
            if (stack2) {
                pidFB(FB_UP_POS + 8, 999999, true);
                pidDRFB(15, 999999, true);
                if (pidMGL(MGL_UP_POS, 0)) {
                    i++;
                    setMGL(0);
                }
            } else {  // stack cone 1
                pidFB(FB_MID_POS, 999999, true);
                pidDRFB(40, 999999, true);
                if (millis() - t0 > 200 && pidMGL(MGL_UP_POS, 0)) setMGL(0);
                setRollers(-80);
                if (stack2) pidDrive(6, 999999);
                if (mglGet() < 70) {
                    setRollers(0);
                    if (!stack2) {
                        resetDriveEnc();
                        DL_pid.doneTime = LONG_MAX;
                        DR_pid.doneTime = LONG_MAX;
                    }
                    i++;
                }
            }
        } else if (i == j++) {  // finish lifting MG
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (stack2) {  // stack cone 1
                pidFB(FB_UP_POS + 8, 999999, true);
                pidDrive(6, 999999);
                if (drfbGet() < 15) {
                    setRollers(-90);
                    i++;
                }
                pidDRFB(0, 999999, true);
                fb_pid_auto.doneTime = LONG_MAX;
            } else {
                i++;
            }
        } else if (i == j++) {  // hover lift over cone 2
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (stack2) {
                pidDRFB(15, 999999, true);
                pidDrive(6, 999999);
                if (drfbGet() > 5) {
                    pidFB(FB_MID_POS - 5, 999999, true);
                    if (fbGet() < FB_MID_POS + 15) { i++; }
                } else {
                    pidFB(FB_UP_POS + 3, 999999, true);
                }
            } else {
                i++;
            }
        } else if (i == j++) {  // grab cone 2
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (stack2) {
                pidDrive(6, 999999);
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
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (stack2) {
                pidDrive(-63, driveT);
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
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (stack2) {
                pidDrive(-63, driveT);
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
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidDrive(stack2 ? -63 : -55, driveT)) {
                resetDriveEnc();
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                i++;
            }
        } else if (i == j++) {
            setFB(0);
            setRollers(0);
            pidDRFB(22, 999999, true);
            if (pidTurn(leftSide ? 43 : -43, driveT)) {
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                i++;
            }
        } else if (i == j++) {
            pidDRFB(20, 999999, true);
            if (pidDrive(-21, driveT)) {
                resetDriveEnc();
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                i++;
            }
        } else if (i == j++) {
            pidDRFB(20, 999999, true);
            if (pidTurn(leftSide ? 90 : -90, driveT)) {
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                i++;
                t0 = millis();
                breakTime = 3000;
            }
        } else if (i == j++) {  // score MG
            prevSens[0] = prevSens[1];
            prevSens[1] = prevSens[2];
            prevSens[2] = eDLGet() + eDRGet();
            pidDrive(24, 999999);
            double d = (eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN;
            if (eDLGet() + eDRGet() - (prevSens[0] + prevSens[1] + prevSens[2]) / 3.0 <= 6 && d > 15.0) {
                mgl_pid.doneTime = LONG_MAX;
                t0 = millis();
                i++;
            }
            if (d > 12) {
                pidMGL(MGL_MID_POS, 999999);
            } else {
                pidMGL(MGL_UP_POS, 999999);
            }
            pidDRFB(20, 999999, true);
        } else if (i == j++) {
            pidDRFB(20, 999999, true);
            if (pidMGL(MGL_DOWN_POS, 0) || millis() - t0 > 1000) {
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                i++;
            }
        } else if (i == j++) {
            pidDRFB(20, 999999, true);
            double d = (eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN;
            if (d > -1.5 || pidMGL(MGL_MID_POS + 8, 0)) {
                if (pidDrive(-20, driveT)) i++;
            } else {
                setDL(0);
                setDR(0);
            }
        } else if (i == j++) {
            break;
        }
        printEnc_all();
        delay(20);
    }
    resetMotors();
    printf("\n\nTIME: %lf\n", (millis() - funcT0) / 1000.0);
}

void autonSkills() {
    int i = 0;
    unsigned long funcT0 = millis(), t0 = millis();
    double prevSens[3] = {0, 0, 0};
    auton1(true, false);
    resetDriveEnc();
    DLturn_pid.doneTime = LONG_MAX;
    DRturn_pid.doneTime = LONG_MAX;
    mgl_pid.doneTime = LONG_MAX;
    double drfbA = 0, fbA = FB_UP_POS;
    int driveT = 700;
    while (millis() - funcT0 < 60000) {
        int j = 0;
        pidDRFB(drfbA, 999999, true);
        pidFB(fbA, 999999, true);
        if (i == j++) {
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidTurn(-90, driveT)) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (pidDrive(12, driveT)) {
                i++;
                resetDriveEnc();
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            double a = (eDRGet() - eDLGet()) * 0.5 / DRIVE_TICKS_PER_DEG;
            if (a < -15 && pidMGL(MGL_DOWN_POS, 0)) setMGL(0);
            if (pidTurn(-90, driveT)) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // grab MG 2
            if (pidMGL(MGL_DOWN_POS, 0)) setMGL(0);
            if (pidDrive(29, driveT)) {
                i++;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // lift MG 2
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (mglGet() < 70) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidDrive(-24, driveT)) {
                i++;
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                resetDriveEnc();
            }
        } else if (i == j++) {
            if (pidTurn(-175, driveT)) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // score MG 2
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN > 30) {
                pidMGL(MGL_MID_POS, 999999);
            } else {
                pidMGL(MGL_UP_POS, 999999);
            }
            if (pidDrive(16, driveT)) {
                i++;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_DOWN_POS, 0)) {
                i++;
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                resetDriveEnc();
            }
        } else if (i == j++) {  // get rid of MG 2
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN < -1.0) pidMGL(MGL_UP_POS, 999999);
            if (pidDrive(-8, driveT)) {
                i++;
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                resetDriveEnc();
            }
        } else if (i == j++) {
            pidMGL(MGL_UP_POS, 999999);
            if (pidTurn(78, driveT)) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            pidMGL(MGL_UP_POS, 999999);
            if (pidDrive(30, driveT)) {
                i++;
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                resetDriveEnc();
            }
        } else if (i == j++) {
            double a = (eDRGet() - eDLGet()) * 0.5 / DRIVE_TICKS_PER_DEG;
            if (a > 30) {
                pidMGL(MGL_DOWN_POS, 999999);
            } else {
                pidMGL(40, 999999);
            }
            if (pidTurn(90, driveT)) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // grab MG 3
            if (pidMGL(MGL_DOWN_POS, 0)) setMGL(0);
            if (pidDrive(25, driveT)) {
                i++;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // lift MG 3
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (mglGet() < 50) {
                i++;
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                resetDriveEnc();
            }
        } else if (i == j++) {
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidTurn(170, driveT)) {
                i++;
                resetDriveEnc();
            }
        } else if (i == j++) {  // score MG 3
            pidMGL(MGL_MID_POS, 999999);
            setDL(127);
            setDR(127);
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN > 34) {
                i++;
                t0 = millis();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // get rid of MG 3
            pidMGL(MGL_MID_POS, 999999);
            if (millis() - t0 < 400) {
                setDL(-127);
                setDR(-127);
            } else if (pidDrive(32, driveT)) {
                i++;
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                resetDriveEnc();
            }
        } else if (i == j++) {
            pidMGL(MGL_UP_POS, 999999);
            if (pidTurn(180, driveT)) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_DOWN_POS, 0)) setMGL(0);
            if (pidDrive(32, driveT)) {
                i++;
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                resetDriveEnc();
            }
        } else if (i == j++) {
            if (pidTurn(20, driveT)) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // grab MG 4
            if (pidDrive(50, driveT)) {
                mgl_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // lift MG 4
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (mglGet() < 60 && pidDrive(50, driveT)) {
                i++;
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                resetDriveEnc();
            }
        } else if (i == j++) {
            if (pidTurn(90, driveT)) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (pidDrive(24, driveT)) {
                i++;
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                resetDriveEnc();
            }
        } else if (i == j++) {
            if (pidTurn(90, driveT)) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // score MG 4
            pidDrive(33, 999999);
            prevSens[0] = prevSens[1];
            prevSens[1] = prevSens[2];
            prevSens[2] = eDLGet() + eDRGet();
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN < 10.0) {
                pidMGL(MGL_MID_POS - 40, 999999);
            } else {
                pidMGL(MGL_MID_POS + 2, 999999);
            }
            if (eDLGet() + eDRGet() - (prevSens[0] + prevSens[1] + prevSens[2]) / 3.0 <= 6 && (eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN > 10.0) {
                i++;
                resetDriveEnc();
                t0 = millis();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (millis() - t0 < 100) {
                pidMGL(MGL_MID_POS + 18, 999999);
            } else {
                pidMGL(MGL_MID_POS, 999999);
            }
            if (pidDrive(-20, driveT)) i++;
        } else if (i == j++) {
            break;
        }
        printEnc_all();
        printf("drive %d, %d", eDLGet(), eDRGet());
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
    int i = 0;
    if (autonMode == i++) {
        return;
    } else if (autonMode == i++) {
        auton1(true, false);
    } else if (autonMode == i++) {
        auton1(false, false);
    } else if (autonMode == i++) {
        auton1(true, true);
    } else if (autonMode == i++) {
        auton1(false, true);
    } else if (autonMode == i++) {
        autonSkills();
    }
    while (true) { delay(20); }
}
