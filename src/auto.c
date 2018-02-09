#include "main.h"
#include "pid.h"
#include "setup.h"
// 15, 20
void auton1(bool leftSide, bool stack2, bool loaderSide) {
    unsigned long t0 = millis(), lastT = millis();
    unsigned long funcT0 = millis();
    int i = 0, prevI = 0;
    double prevSens[3] = {0, 0, 0};
    unsigned long breakTime = 4000;
    int driveT = 100;
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
            if (mglGet() > 93 || (loaderSide && fbGet() > 15 && millis() - t0 > 200)) {
                i++;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // grab MG
            setRollers(25);
            pidFB(FB_UP_POS, 999999, true);
            pidDRFB(18, 999999, true);
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN >= 58) {
                pidMGL(MGL_UP_POS, 999999);
            } else if (pidMGL(MGL_DOWN_POS, 0)) {
                setMGL(0);
            }
            if (pidDrive(60, driveT)) {
                mgl_pid.doneTime = LONG_MAX;
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                setDL(0);
                setDR(0);
                t0 = millis();
                i++;
            }
        } else if (i == j++) {  // lift MG
            pidFB(FB_UP_POS, 999999, true);
            pidDRFB(18, 999999, true);
            if (stack2) {
                if (pidMGL(MGL_UP_POS, 0)) {
                    i++;
                    setMGL(0);
                    resetDriveEnc();
                }
            } else {
                pidMGL(MGL_UP_POS, 0);
                if (mglGet() < 70) i++;
            }
        } else if (i == j++) {  // finish lifting MG
            if (stack2) {       // stack cone 1
                pidFB(FB_UP_POS, 999999, true);
                pidDRFB(5, 999999, true);
                pidDrive(6, 999999);
                if (drfbGet() < 15) setRollers(-90);
                if (drfbGet() < 10) i++;
            } else {
                i++;
            }
        } else if (i == j++) {  // hover lift over cone 2
            if (stack2) {
                pidDRFB(18, 999999, true);
                pidDrive(6, 999999);
                pidFB(FB_MID_POS - 5, 999999, true);
                if (fbGet() < FB_MID_POS + 15) {
                    i++;
                    setRollers(70);
                }
            } else {
                i++;
            }
        } else if (i == j++) {  // grab cone 2
            if (stack2) {
                pidDrive(6, 999999);
                pidFB(FB_MID_POS - 5, 999999, true);
                pidDRFB(0, 999999, true);
                if (drfbGet() < 3 && fbGet() < FB_MID_POS + 4) {
                    setRollers(20);
                    resetDriveEnc();
                    DL_pid.doneTime = LONG_MAX;
                    DR_pid.doneTime = LONG_MAX;
                    i++;
                }
            } else {
                i++;
            }
        } else if (i == j++) {  // lift cone 2
            if (stack2) {
                pidDrive(-62, driveT);
                pidDRFB(25, 999999, true);
                if (drfbGet() > 15) {
                    pidFB(FB_UP_POS, 999999, true);
                    if (fbGet() > FB_UP_POS - 10) {
                        drfb_pid_auto.doneTime = LONG_MAX;
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
                pidDrive(-62, driveT);
                if (drfbGet() < 5) setRollers(-70);
                pidFB(FB_UP_POS, 999999, true);
                pidDRFB(0, 999999, true);
                if (drfbGet() < 3) {
                    drfb_pid_auto.doneTime = LONG_MAX;
                    i++;
                }
            } else {
                i++;
            }
        } else if (i == j++) {
            setRollers(-70);
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);  // finsh lifting mgl for stack 1 cone
            pidFB(FB_UP_POS, 999999, true);
            pidDRFB(20, 999999, true);
            if (pidDrive(stack2 ? -62 : -56, driveT)) {
                resetDriveEnc();
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                i++;
            }
        } else if (i == j++) {
            setFB(0);
            setRollers(0);
            pidDRFB(20, 999999, true);
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
                t0 = millis();
                breakTime = 3000;
                i++;
            }
        } else if (i == j++) {  // score MG
            double d = (eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN;
            prevSens[0] = prevSens[1];
            prevSens[1] = prevSens[2];
            prevSens[2] = eDLGet() + eDRGet();
            pidDrive(22, 999999);
            if (d > 10.0 && !stack2) pidMGL(MGL_MID_POS, 999999);
            pidDRFB(20, 999999, true);
            if (eDLGet() + eDRGet() - (prevSens[0] + prevSens[1] + prevSens[2]) / 3.0 <= 6 && d > 15.0) {
                t0 = millis();
                setDL(60);
                setDR(60);
                i++;
            }
        } else if (i == j++) {
            pidDRFB(20, 999999, true);
            pidMGL(MGL_DOWN_POS, 999999);
            if (millis() - t0 > 300) {
                setDL(20);
                setDR(20);
            }
            if (mglGet() >= MGL_MID_POS + 10 || millis() - t0 > 1000) {
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                i++;
            }
        } else if (i == j++) {
            pidDRFB(20, 999999, true);
            pidDrive(-20, 999999);
            pidMGL(MGL_DOWN_POS, 999999);
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN < -1.0) i++;
        } else if (i == j++) {
            pidMGL(MGL_MID_POS + 8, 999999);
            if (mglGet() < MGL_MID_POS + 13) {
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
    auton1(true, false, true);
    resetDriveEnc();
    DLturn_pid.doneTime = LONG_MAX;
    DRturn_pid.doneTime = LONG_MAX;
    mgl_pid.doneTime = LONG_MAX;
    double drfbA = 0, fbA = FB_UP_POS;
    int driveT = 100;
    printf("\nauton finished\n\n");
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
            if (pidDrive(15, driveT)) {
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
            if (pidDrive(28, driveT)) {
                printf("\nMG 2 grabbed: lift MG 2...\n\n");
                i++;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // lift MG 2
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (mglGet() < 50) {
                i++;
                resetDriveEnc();
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidTurn(-175, driveT)) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // score MG 2: SLAM method
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN > 8.0) {
                pidMGL(MGL_MID_POS, 999999);
            } else if (pidMGL(MGL_UP_POS, 0)) {
                setMGL(0);
            }
            if (pidDrive(32, driveT)) {
                printf("\nMG 2 slammed:\n\n");
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
            if (pidDrive(33, driveT)) {
                i++;
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                resetDriveEnc();
            }
        } else if (i == j++) {
            if ((eDRGet() - eDLGet()) * 0.5 / DRIVE_TICKS_PER_DEG > 30) {
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
            if (pidDrive(31, driveT)) {
                printf("\nMG 3 grabbed: lift MG 3...\n\n");
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
            if (pidTurn(175, driveT)) {
                i++;
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                resetDriveEnc();
            }
        } else if (i == j++) {  // score MG 3: SLAM method
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN > 10.0) {
                pidMGL(MGL_MID_POS, 999999);
            } else {
                pidMGL(MGL_UP_POS, 999999);
            }
            if (pidDrive(33, driveT)) {
                printf("\nMG 3 slammed: get rid MG 3...\n\n");
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // get rid of MG 3
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidDrive(-9, driveT)) {
                i++;
                resetDriveEnc();
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidTurn(52, driveT)) {
                resetDriveEnc();
                i++;
            }
        } else if (i == j++) {  // square up against wall
            prevSens[0] = prevSens[1];
            prevSens[1] = prevSens[2];
            prevSens[2] = eDLGet() + eDRGet();
            pidDrive(22, 999999);
            if (eDLGet() + eDRGet() - (prevSens[0] + prevSens[1] + prevSens[2]) / 3.0 <= 6 && (eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN > 8.0) {
                printf("\nsquared up against wall:\n\n");
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                i++;
            }
        } else if (i == j++) {
            if (pidDrive(-23, driveT)) {
                i++;
                resetDriveEnc();
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // line up with MG 4
            if ((eDRGet() - eDLGet()) * 0.5 / DRIVE_TICKS_PER_DEG > 90.0 && pidMGL(MGL_DOWN_POS, 0)) setMGL(0);
            if (pidTurn(133, driveT)) {
                resetDriveEnc();
                i++;
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // grab MG 4
            if (pidMGL(MGL_DOWN_POS, 0)) setMGL(0);
            if (pidDrive(60, driveT)) {
                printf("\nMG 4 grabbed: lift MG 4...\n\n");
                mgl_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                i++;
            }
        } else if (i == j++) {  // lift MG 4
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (mglGet() < 100 && pidDrive(32, driveT)) {
                i++;
                resetDriveEnc();
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (pidTurn(90, driveT)) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (pidDrive(16, driveT)) {
                i++;
                resetDriveEnc();
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (pidTurn(-90, driveT)) {
                i++;
                resetDriveEnc();
            }
        } else if (i == j++) {  // score MG 4: 20pt
            prevSens[0] = prevSens[1];
            prevSens[1] = prevSens[2];
            prevSens[2] = eDLGet() + eDRGet();
            pidDrive(27, 999999);
            pidMGL(MGL_UP_POS, 999999);
            if (eDLGet() + eDRGet() - (prevSens[0] + prevSens[1] + prevSens[2]) / 3.0 <= 6 && (eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN > 15.0) {
                printf("\nMG 4 scored in 20pt:\n\n");
                t0 = millis();
                setDL(20);
                setDR(20);
                i++;
            }
        } else if (i == j++) {
            pidMGL(MGL_DOWN_POS, 999999);
            if (mglGet() >= MGL_MID_POS + 13 || millis() - t0 > 1000) {
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                i++;
            }
        } else if (i == j++) {
            pidDRFB(20, 999999, true);
            pidDrive(-20, 999999);
            pidMGL(MGL_DOWN_POS, 999999);
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN < -1.0) {
                mgl_pid.doneTime = LONG_MAX;
                i++;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_MID_POS + 8, 0)) {
                if (pidDrive(-20, driveT)) {
                    i++;
                    resetDriveEnc();
                    DLturn_pid.doneTime = LONG_MAX;
                    DRturn_pid.doneTime = LONG_MAX;
                }
            } else {
                setDL(20);
                setDR(20);
            }
        } else if (i == j++) {
            if (pidTurn(90, driveT)) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (pidDrive(14, driveT)) {
                i++;
                resetDriveEnc();
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if ((eDRGet() - eDLGet()) * 0.5 / DRIVE_TICKS_PER_DEG > 30) {
                if (pidMGL(MGL_DOWN_POS, 0)) setMGL(0);
            }
            if (pidTurn(90, driveT)) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // grab MG 5
            if (pidMGL(MGL_DOWN_POS, 0)) setMGL(0);
            if (pidDrive(22, driveT)) {
                i++;
                resetDriveEnc();
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (mglGet() < 50) {
                if (pidTurn(175, driveT)) {
                    i++;
                    resetDriveEnc();
                    DL_pid.doneTime = LONG_MAX;
                    DR_pid.doneTime = LONG_MAX;
                }
            }
        } else if (i == j++) {  // score MG 5: SLAM method
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN > 8.0) {
                pidMGL(MGL_MID_POS, 999999);
            } else if (pidMGL(MGL_UP_POS, 0)) {
                setMGL(0);
            }
            if (pidDrive(32, driveT)) {
                printf("\nMG 5 slammed:\n\n");
                i++;
                resetDriveEnc();
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidDrive(120, driveT)) {
                resetDriveEnc();
                i++;
            }
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
        auton1(true, false, false);
    } else if (autonMode == i++) {
        auton1(false, false, false);
    } else if (autonMode == i++) {
        auton1(true, true, false);
    } else if (autonMode == i++) {
        auton1(false, true, false);
    } else if (autonMode == i++) {
        autonSkills();
    }
    while (true) { delay(20); }
}
