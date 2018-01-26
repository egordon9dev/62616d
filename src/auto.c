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
    while (true) {
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
            if (mglGet() > 85) {
                i++;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // grab MG
            if (pidMGL(MGL_DOWN_POS, 0)) setMGL(0);
            if (stack2) {
                pidFB(FB_UP_POS + 8, 999999, true);
                pidDRFB(15, 999999, true);
            } else {
                pidFB(FB_MID_POS, 999999, true);
                pidDRFB(35, 999999, true);
            }
            if (pidDrive(60, 200)) {
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
                if (pidMGL(MGL_UP_POS, 0)) {
                    i++;
                    setMGL(0);
                }
            } else {
                pidFB(FB_MID_POS, 999999, true);
                pidDRFB(35, 999999, true);
                if (millis() - t0 > 100 && pidMGL(MGL_UP_POS, 0)) setMGL(0);
                setRollers(-80);
                if (mglGet() < 80) {
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
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
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
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
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
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
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
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (stack2) {
                pidDrive(-61, 200);
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
                pidDrive(-61, 200);
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
            if (pidDrive(stack2 ? -61 : -53, 200)) {
                resetDriveEnc();
                turn_pid.doneTime = LONG_MAX;
                i++;
                yaw += leftSide ? 41 : -41;
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
            setDRFB(0);
            if (millis() - t0 < 100) {
                pidMGL(MGL_MID_POS + 18, 999999);
            } else {
                pidMGL(MGL_MID_POS, 999999);
            }
            if (pidDrive(-20, 200)) i++;
        } else if (i == j++) {
            break;
        }
        printEnc_all();
        delay(20);
    }
    resetMotors();
    printf("\n\nTIME: %lf\n", (millis() - funcT0) / 1000.0);
}

void skillsAuto() {
    int i = 0;
    unsigned long funcT0 = millis(), t0 = millis();
    double prevSens[3] = {0, 0, 0};
    double prevVel[3] = {0, 0, 0};
    auton1(true, false);
    int yaw = yawGet();
    turn_pid.doneTime = LONG_MAX;
    mgl_pid.doneTime = LONG_MAX;
    yaw += 55;
    double drfbA = 0, fbA = FB_UP_POS;
    while (millis() - funcT0 < 60000) {
        int j = 0;
        pidDRFB(drfbA, 999999, true);
        pidFB(fbA, 999999, true);
        if (i == j++) {
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidTurn(yaw, 200)) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // align bot to wall
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidDrive(-21, 200)) {
                i++;
                turn_pid.doneTime = LONG_MAX;
                yaw -= 95;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidTurn(yaw, 200)) {
                i++;
                resetDriveEnc();
            }
        } else if (i == j++) {  // square bot on wall
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            pidDrive(21, 999999);
            if (eDLGet() + eDRGet() - (prevSens[0] + prevSens[1] + prevSens[2]) / 3.0 <= 6 && (eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN > 15) {
                i++;
                t0 = millis();
            }
            prevSens[0] = prevSens[1];
            prevSens[1] = prevSens[2];
            prevSens[2] = eDLGet() + eDRGet();
        } else if (i == j++) {  // square bot on wall
            setDL(20);
            setDR(20);
            if (millis() - t0 > 200) {
                yaw = yawGet();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                i++;
            }
        } else if (i == j++) {
            if (pidDrive(-8, 200)) {
                i++;
                turn_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
                yaw -= 149;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_DOWN_POS, 0)) setMGL(0);
            if (pidTurn(yaw, 200)) {
                i++;
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                resetDriveEnc();
            }
        } else if (i == j++) {  // grab MG 2
            if (pidMGL(MGL_DOWN_POS, 0)) setMGL(0);
            if (pidDrive(38, 200)) {
                i++;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // lift MG 2
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (mglGet() < 50) {
                i++;
                turn_pid.doneTime = LONG_MAX;
                yaw -= 160;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidTurn(yaw, 200)) {
                i++;
                resetDriveEnc();
            }
        } else if (i == j++) {  // score MG 2
            pidMGL(MGL_MID_POS, 999999);
            setDL(127);
            setDR(127);
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN > 42) {
                i++;
                t0 = millis();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // get rid of MG 2
            pidMGL(MGL_MID_POS, 999999);
            if (millis() - t0 < 400) {
                setDL(-127);
                setDR(-127);
            } else if (pidDrive(41, 200)) {
                i++;
                turn_pid.doneTime = LONG_MAX;
                yaw += 93;
            }
        } else if (i == j++) {
            pidMGL(MGL_MID_POS, 999999);
            if (pidTurn(yaw, 200)) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            pidMGL(MGL_MID_POS, 999999);
            if (pidDrive(31, 200)) {
                i++;
                turn_pid.doneTime = LONG_MAX;
                yaw += 92;
            }
        } else if (i == j++) {
            pidMGL(MGL_MID_POS, 999999);
            if (pidTurn(yaw, 200)) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // grab MG 3
            if (pidMGL(MGL_DOWN_POS, 0)) setMGL(0);
            if (pidDrive(25, 200)) {
                i++;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // lift MG 3
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (mglGet() < 50) {
                i++;
                turn_pid.doneTime = LONG_MAX;
                yaw += 170;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidTurn(yaw, 200)) {
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
            } else if (pidDrive(32, 200)) {
                i++;
                turn_pid.doneTime = LONG_MAX;
                yaw += 180;
            }
        } else if (i == j++) {
            pidMGL(MGL_UP_POS, 999999);
            if (pidTurn(yaw, 200)) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_DOWN_POS, 0)) setMGL(0);
            if (pidDrive(32, 200)) {
                i++;
                turn_pid.doneTime = LONG_MAX;
                yaw += 10
            }
        } else if (i == j++) {
            if (pidTurn(yaw, 200)) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // grab MG 4
            if (pidDrive(50, 200)) {
                mgl_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // lift MG 4
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (mglGet() < 60 && pidDrive(50, 200)) {
                i++;
                yaw += 90;
            }
        } else if (i == j++) {
            if (pidTurn(yaw, 200)) {
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (pidDrive(24, 200)) {
                i++;
                turn_pid.doneTime = LONG_MAX;
                yaw -= 90;
            }
        } else if (i == j++) {
            if (pidTurn(yaw, 200)) {
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
            if (pidDrive(-20, 200)) i++;
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
void skillsUserAutonTask() { auton1(false, false); }
void skillsUser() {
    TaskHandle at = taskCreate(skillsUserAutonTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
    while (!joystickGetDigital(2, 7, JOY_DOWN)) { delay(10); }
    taskDelete(at);
    resetMotors();
}
void autonomous() {
    resetDriveEnc();
    auton1(true, false);
}
