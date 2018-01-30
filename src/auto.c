#include "main.h"
#include "pid.h"
#include "setup.h"
// 15, 20
void auton1(bool leftSide, bool stack2) {
    int yaw = 0;
    unsigned long t0 = millis(), lastT = millis();
    unsigned long funcT0 = millis();
    int i = 0, prevI = 0;
    double prevSens[3] = {0, 0, 0};
    unsigned long breakTime = 4000, contTime = 4000;
    while (true) {
        if (i != prevI) lastT = millis();
        if (millis() - lastT > breakTime) break;
        if (millis() - lastT > contTime) i++;
        prevI = i;
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
            if (mglGet() > 93) {
                i++;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // grab MG
            if (pidMGL(MGL_DOWN_POS, 0)) setMGL(20);
            if (stack2) {
                pidFB(FB_UP_POS + 8, 999999, true);
                pidDRFB(15, 999999, true);
            } else {
                pidFB(FB_MID_POS, 999999, true);
                pidDRFB(35, 999999, true);
            }
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN >= 58) pidMGL(MGL_UP_POS, 999999);
            if (pidDrive(60, 100)) {
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
        } else if (i == j++) {  // (mostly) lift MG
            if (stack2) {
                pidFB(FB_UP_POS + 8, 999999, true);
                pidDRFB(15, 999999, true);
                if (pidMGL(MGL_UP_POS, 0)) {
                    i++;
                    setMGL(0);
                }
            } else {  // stack cone 1
                pidFB(FB_MID_POS, 999999, true);
                pidDRFB(35, 999999, true);
                if (millis() - t0 > 200 && pidMGL(MGL_UP_POS, 0)) setMGL(0);
                setRollers(-80);
                if (stack2) pidDrive(6, 999999);
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
                pidDrive(-63, 100);
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
                pidDrive(-63, 100);
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
            if (pidDrive(stack2 ? -63 : -57, 100)) {
                resetDriveEnc();
                turn_pid.doneTime = LONG_MAX;
                i++;
                yaw += leftSide ? 41 : -48;
            }
        } else if (i == j++) {
            setFB(0);
            setRollers(0);
            pidDRFB(22, 999999, true);
            if (pidTurn(yaw, 0)) {
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                i++;
            }
        } else if (i == j++) {
            pidDRFB(20, 999999, true);
            if (pidDrive(-16, 100)) {
                resetDriveEnc();
                turn_pid.doneTime = LONG_MAX;
                i++;
                yaw += leftSide ? 88 : -94;
            }
        } else if (i == j++) {
            pidDRFB(20, 999999, true);
            if (pidTurn(yaw, 0)) {
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                i++;
                contTime = 1000;
            }
        } else if (i == j++) {  // score MG
            if (pidDrive(25, 0)) {
                resetDriveEnc();
                t0 = millis();
                i++;
            }
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN < 10.0) {
                pidMGL(MGL_MID_POS - 40, 999999);
            } else {
                pidMGL(MGL_MID_POS, 999999);
            }
            pidDRFB(20, 999999, true);
        } else if (i == j++) {  // score MG
            pidMGL(MGL_MID_POS, 999999);
            setDL(127);
            setDR(127);
            prevSens[0] = prevSens[1];
            prevSens[1] = prevSens[2];
            prevSens[2] = eDLGet() + eDRGet();
            pidDRFB(20, 999999, true);
            if (eDLGet() + eDRGet() - (prevSens[0] + prevSens[1] + prevSens[2]) / 3.0 <= 6 && millis() - t0 > 300) {
                i++;
                t0 = millis();
            }
        } else if (i == j++) {
            pidMGL(MGL_MID_POS + 10, 999999);
            setDL(-90);
            setDR(-90);
            pidDRFB(20, 999999, true);
            if (millis() - t0 > 350) {
                resetDriveEnc();
                t0 = millis();
                i++;
            }
        } else if (i == j++) {
            pidMGL(MGL_MID_POS, 999999);
            setDL(127);
            setDR(127);
            prevSens[0] = prevSens[1];
            prevSens[1] = prevSens[2];
            prevSens[2] = eDLGet() + eDRGet();
            pidDRFB(20, 999999, true);
            if (eDLGet() + eDRGet() - (prevSens[0] + prevSens[1] + prevSens[2]) / 3.0 <= 6 && millis() - t0 > 300) {
                i++;
                t0 = millis();
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            setDRFB(0);
            pidMGL(MGL_MID_POS + 8, 999999);
            if (pidDrive(-16, 100)) i++;
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
            if (pidDrive(-27, 200)) {
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
            pidDrive(22, 999999);
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
                yaw -= 155;
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
            if (pidDrive(32, 200)) {
                i++;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // lift MG 2
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (mglGet() < 50) {
                i++;
                turn_pid.doneTime = LONG_MAX;
                yaw -= 165;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidTurn(yaw, 200)) {
                i++;
                resetDriveEnc();
            }
        } else if (i == j++) {  // score MG 2
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN > 30) {
                pidMGL(MGL_MID_POS, 999999);
            } else {
                pidMGL(MGL_MID_POS - 20, 999999);
            }
            setDL(127);
            setDR(127);
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN > 35) {
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
            } else if (pidDrive(39, 200)) {
                i++;
                turn_pid.doneTime = LONG_MAX;
                yaw += 88;
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
            if (pidDrive(37, 200)) {
                i++;
                turn_pid.doneTime = LONG_MAX;
                yaw += 92;
            }
        } else if (i == j++) {
            pidMGL(MGL_MID_POS - 30, 999999);
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
                yaw += 20;
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
void skillsDriver() {
    TaskHandle at = taskCreate(skillsUserAutonTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_LOWEST + 1);
    while (!joystickGetDigital(2, 7, JOY_DOWN) && !joystickGetDigital(2, 8, JOY_DOWN)) { delay(10); }
    taskDelete(at);
    resetMotors();
    lcdSetText(LCD, 1, "DONE YAY");
    lcdSetText(LCD, 2, "DONE YAY");
}
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
        skillsAuto();
    }
    while (true) { delay(20); }
}
