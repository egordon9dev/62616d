#include "main.h"
#include "pid.h"
#include "setup.h"
/*
todo:
auton1: 1|2|3 field cones in 20pt
auton2: 3 driver load cones in 20pt
make skills: 574C skills run
*/

// 15, 20
void auton1(bool leftSide, int stackH, bool loaderSide) {
    unsigned long t0 = millis(), lastT = millis();
    unsigned long funcT0 = millis();
    int i = 0, prevI = 0, u = 0;
    double prevSens[3] = {0, 0, 0};
    unsigned long breakTime = 5000;
    int driveT = 250;
    static double drfba1, drfba2;
    while (millis() - funcT0 < 15000) {
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
                pidMGL(MGL_DOWN_POS, 0);
                pidDRFB(0, 999999, true);  // 10
            }
            pidFB(FB_UP_POS, 999999, true);
            if (mglGet() > 30 || (loaderSide && mglGet() > 5)) {
                i++;
                u = 0;
                mgl_pid.doneTime = LONG_MAX;
                drfb_pid_auto.doneTime = LONG_MAX;
                fb_pid_auto.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // grab MG
            setRollers(25);
            pidMGL(MGL_DOWN_POS, 0);
            int h = 0;
            if (u == h++) {
                bool fbDone = pidFB(FB_UP_POS, 0, true), drfbDone = pidDRFB(0, 0, true);
                if (fbDone && drfbDone) {
                    u++;
                    fb_pid_auto.doneTime = LONG_MAX;
                    drfb_pid_auto.doneTime = LONG_MAX;
                }
            } else if (u == h++) {
                pidFB(FB_MID_POS, 999999, true);
                pidDRFB(40, 999999, true);
            }
            double d = (eDRGet() + eDLGet()) * 0.5 / DRIVE_TICKS_PER_IN;
            if (d > 52.5) pidMGL(MGL_UP_POS, 999999);
            double dTarget = 54.0;
            pidDrive(dTarget, 999999, false);
            if (fabs(dTarget - d) < 0.5) {
                mgl_pid.doneTime = LONG_MAX;
                i++;
            }
        } else if (i == j++) {  // lift MG
            pidFB(FB_UP_POS, 999999, true);
            pidDRFB(20, 999999, true);
            if (stackH > 1) {
                if (pidMGL(MGL_UP_POS, 0)) {
                    DL_pid.doneTime = LONG_MAX;
                    DR_pid.doneTime = LONG_MAX;
                    resetDriveEnc();
                    t0 = millis();
                    u = 0;
                    i++;
                }
            } else {
                pidMGL(MGL_UP_POS, 999999);
                if (mglGet() < 110) i++;
            }
        } else if (i == j++) {
            if (stackH > 1) {
                int h = 0, umax = -1;
                if (u == h++) {  // stack cone 1
                    pidFB(FB_UP_POS, 999999, true);
                    pidDRFB(0, 999999, true);
                    if (drfbGet() < 5) {
                        setRollers(-60);
                        u++;
                    }
                } else if (u == h++) {  // hover cone 2
                    pidDRFB(15, 999999, true);
                    if (drfbGet() > 10) {
                        pidFB(FB_MID_POS, 999999, true);
                        setRollers(-20);
                    } else {
                        pidFB(FB_UP_POS, 999999, true);
                    }
                    if (fbGet() < FB_UP_POS - 15) u++;
                } else if (u == h++) {
                    umax = u;
                }
                if (pidDrive(10, umax == -1 ? 999999 : driveT, false)) {  // drive to cone 2
                    setRollers(60);
                    i++;
                } else {  // continue hover cone 2
                    pidFB(FB_MID_POS, 999999, true);
                    pidDRFB(15, 999999, true);
                }
            } else {
                i++;
            }
        } else if (i == j++) {  // grab cone 2
            if (stackH > 1) {
                pidFB(FB_MID_POS - 11, 999999, true);
                pidDRFB(0, 999999, true);
                if (fbGet() < FB_MID_POS - 7 && drfbGet() < 3) {
                    setRollers(25);
                    resetDriveEnc();
                    DL_pid.doneTime = LONG_MAX;
                    DR_pid.doneTime = LONG_MAX;
                    u = 0;
                    i++;
                }
            } else {
                i++;
            }
        } else if (i == j++) {
            if (stackH > 2) {
                int h = 0, umax = -1;
                if (u == h++) {  // lift cone 2
                    pidDRFB(20, 999999, true);
                    pidFB(FB_UP_POS, 999999, true);
                    if (drfbGet() > 16 && fbGet() > FB_UP_POS - 5) {
                        u++;
                        t0 = millis();
                    }
                } else if (u == h++) {  // stack cone 2
                    pidFB(FB_UP_POS, 999999, true);
                    pidDRFB(0, 999999, true);
                    if (drfbGet() < 5) setRollers(-60);
                    if (drfbGet() < 2) u++;
                } else if (u == h++) {  // hover cone 3
                    pidDRFB(20, 999999, true);
                    if (drfbGet() > 15) pidFB(FB_MID_POS, 999999, true);
                    if (fbGet() < FB_UP_POS - 15) {
                        setRollers(-20);
                        u++;
                    }
                } else if (u == h++) {
                    pidFB(FB_MID_POS, 999999, true);
                    pidDRFB(20, 999999, true);
                    umax = u;
                }
                if (pidDrive(7, u == umax ? driveT : 999999, false)) {  // drive to cone 3
                    setRollers(60);
                    i++;
                }
            } else {
                i++;
            }
        } else if (i == j++) {  // grab cone 3
            if (stackH > 2) {
                pidDRFB(0, 999999, true);
                pidFB(FB_MID_POS - 11, 999999, true);
                if (fbGet() < FB_MID_POS - 7 && drfbGet() < 2) {
                    setRollers(25);
                    i++;
                }
            } else {
                i++;
            }
        } else if (i == j++) {
            resetDriveEnc();
            DL_pid.doneTime = LONG_MAX;
            DR_pid.doneTime = LONG_MAX;
            drfb_pid_auto.doneTime = LONG_MAX;
            fb_pid_auto.doneTime = LONG_MAX;
            i++;
            u = 0;
        } else if (i == j++) {
            int h = 0;
            double d;
            if (stackH > 2) {
                d = -48.0;
                drfba1 = 28.0;
                drfba2 = 10.0;
            } else if (stackH > 1) {
                d = -41.0;
                drfba1 = 20.0;
                drfba2 = 5.0;
            } else {
                d = -49.0;
                drfba1 = 15.0;
                drfba2 = 0.0;
            }
            if (u == h++) {  // lift cone 1|2|3
                bool drfbDone = pidDRFB(drfba1, 200, true), fbDone = pidFB(FB_UP_POS, 200, true);
                if (drfbDone && fbDone) {
                    drfb_pid_auto.doneTime = LONG_MAX;
                    u++;
                }
            } else if (u == h++) {
                pidFB(FB_UP_POS, 999999, true);
                setRollers(-60);
                if (pidDRFB(drfba2, 0, true)) {
                    u++;
                    drfb_pid_auto.doneTime = LONG_MAX;
                }
            } else if (u == h++) {
                pidFB(FB_UP_POS, 999999, true);
                if (pidDRFB(drfba1, 0, true)) {
                    u++;
                    setRollers(0);
                }
            } else if (u == h++) {
                pidFB(FB_UP_POS, 999999, true);
                pidDRFB(drfba1, 999999, true);
            }
            if (pidDrive(d, driveT, false)) {
                i++;
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                resetDriveEnc();
            }
        } else if (i == j++) {
            setFB(0);
            pidDRFB(drfba1, 999999, true);
            double a = leftSide ? -157 : 157;
            if (pidTurn(a, driveT)) {
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                i++;
            }
        } else if (i == j++) {
            pidDRFB(drfba1, 999999, true);
            double d = 40;
            if (pidDrive(d, driveT, false)) {
                resetDriveEnc();
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                i++;
            }
        } else if (i == j++) {
            pidDRFB(drfba1, 999999, true);
            double a = leftSide ? -70.0 : 70.0;
            if (pidTurn(a, driveT)) {
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
            pidDrive(42, 999999, false);
            pidMGL(MGL_MID_POS - 5, 999999);
            pidDRFB(drfba1, 999999, true);
            if (eDLGet() + eDRGet() - prevSens[1] <= 10 && d > 15.0) {
                t0 = LONG_MAX;
                setDL(60);
                setDR(60);
                u = 0;
                i++;
            }
        } else if (i == j++) {  // get rid of MG
            pidDRFB(drfba1, 999999, true);
            pidMGL(MGL_MID_POS + 21, 999999);
            setDL(25);
            setDR(25);
            if (mglGet() >= MGL_MID_POS + 15) t0 = millis();
            if (millis() - t0 > 300) {
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                i++;
            }
        } else if (i == j++) {
            pidDRFB(drfba1, 999999, true);
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN < -6.0) {
                pidMGL(MGL_UP_POS, 999999);
            } else {
                pidMGL(MGL_MID_POS + 15, 999999);
            }
            if (pidDrive(-10, driveT, false)) i++;
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
    while (true) {
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
            if (pidDrive(15.5, driveT, false)) {
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
            if (pidDrive(28, driveT, false)) {
                printf("\nMG 2 grabbed: lift MG 2...\n\n");
                i++;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // lift MG 2
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (mglGet() < 30) {
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
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN > 15.0) {
                pidMGL(MGL_MID_POS, 999999);
            } else if (pidMGL(MGL_UP_POS, 0)) {
                setMGL(0);
            }
            if (pidDrive(33, driveT, false)) {
                printf("\nMG 2 slammed:\n\n");
                i++;
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                resetDriveEnc();
            }
        } else if (i == j++) {
            pidMGL(MGL_UP_POS, 999999);
            if (pidTurn(81, driveT)) {  // 78 (1)
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            pidMGL(MGL_UP_POS, 999999);
            if (pidDrive(33, driveT, false)) {  // 31(2)
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
            if (pidDrive(33, driveT, false)) {
                printf("\nMG 3 grabbed: lift MG 3...\n\n");
                i++;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // lift MG 3
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (mglGet() < 30) {
                i++;
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                resetDriveEnc();
            }
        } else if (i == j++) {
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidDrive(-20, driveT, false)) {
                i++;
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                resetDriveEnc();
            }
        } else if (i == j++) {
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidTurn(172, driveT)) {
                i++;
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                t0 = millis();
            }
        } else if (i == j++) {  // score MG 3: SLAM method
            double d = (eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN;
            if ((d > 5 && pidMGL(MGL_DOWN_POS, 0) && pidDrive(16, driveT, false)) || millis() - t0 > 4000) {
                printf("\nMG 3 slammed: get rid MG 3...\n\n");
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // get rid of MG 3
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidDrive(-13.5, driveT, false)) {
                i++;
                resetDriveEnc();
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidTurn(55, driveT)) {  // 52(2)
                resetDriveEnc();
                i++;
            }
        } else if (i == j++) {  // square up against wall
            prevSens[0] = prevSens[1];
            prevSens[1] = prevSens[2];
            prevSens[2] = eDLGet() + eDRGet();
            pidDrive(23, 999999, false);
            if (eDLGet() + eDRGet() - (prevSens[0] + prevSens[1] + prevSens[2]) / 3.0 <= 6 && (eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN > 8.0) {
                printf("\nsquared up against wall:\n\n");
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                i++;
            }
        } else if (i == j++) {
            if (pidDrive(-20.5, driveT, false)) {
                i++;
                resetDriveEnc();
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // line up with MG 4
            if ((eDRGet() - eDLGet()) * 0.5 / DRIVE_TICKS_PER_DEG > 90.0 && pidMGL(MGL_DOWN_POS, 0)) setMGL(0);
            if (pidTurn(132.5, driveT)) {
                resetDriveEnc();
                i++;
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // grab MG 4
            if (pidMGL(MGL_DOWN_POS, 0)) setMGL(0);
            pidDrive(93, driveT, false);
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN > 60.0) {
                printf("\nMG 4 grabbed: lift MG 4...\n\n");
                mgl_pid.doneTime = LONG_MAX;
                i++;
            }
        } else if (i == j++) {  // lift MG 4
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidDrive(93, driveT, false)) {
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
            if (pidDrive(19, driveT, false)) {
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
            pidDrive(35, 999999, false);
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
            if (mglGet() >= MGL_MID_POS + 11 || millis() - t0 > 1000) {
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                i++;
            }
        } else if (i == j++) {
            pidDRFB(20, 999999, true);
            pidDrive(-20, 999999, false);
            pidMGL(MGL_DOWN_POS, 999999);
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN < -1.0) {
                mgl_pid.doneTime = LONG_MAX;
                t0 = millis();
                i++;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_MID_POS + 8, 0) || millis() - t0 > 800) {
                if (pidDrive(-20, driveT, false)) {
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
            if (pidDrive(16.5, driveT, false)) {
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
            if (pidDrive(22, driveT, false)) {
                i++;
                resetDriveEnc();
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (mglGet() < 30) {
                if (pidTurn(-180, driveT)) {
                    i++;
                    resetDriveEnc();
                    DL_pid.doneTime = LONG_MAX;
                    DR_pid.doneTime = LONG_MAX;
                }
            }
        } else if (i == j++) {  // score MG 5: SLAM method
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN > 15.0) {
                pidMGL(MGL_MID_POS, 999999);
            } else if (pidMGL(MGL_UP_POS, 0)) {
                setMGL(0);
            }
            if (pidDrive(31, driveT, false)) {
                printf("\nMG 5 slammed:\n\n");
                i++;
                resetDriveEnc();
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidTurn(138, driveT)) {
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                i++;
            }
        } else if (i == j++) {  // grab MG 6
            pidMGL(MGL_DOWN_POS, 999999);
            if (pidDrive(60, driveT, false)) {
                i++;
                resetDriveEnc();
                mgl_pid.doneTime = LONG_MAX;
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // lift MG 6
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (mglGet() < 35 && pidDrive(-40, driveT, false)) {
                i++;
                resetDriveEnc();
                DLturn_pid.doneTime = LONG_MAX;
                DRturn_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {
            if (pidTurn(180, driveT)) {
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                i++;
            }
        } else if (i == j++) {
            pidMGL(MGL_MID_POS, 999999);
            if (pidDrive(17, driveT, false)) {
                mgl_pid.doneTime = LONG_MAX;
                i++;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_UP_POS, 0)) {
                setMGL(0);
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
        auton1(true, 1, false);
    } else if (autonMode == i++) {
        auton1(true, 2, false);
    } else if (autonMode == i++) {
        auton1(true, 3, false);
    } else if (autonMode == i++) {
        auton1(false, 1, false);
    } else if (autonMode == i++) {
        auton1(false, 2, false);
    } else if (autonMode == i++) {
        auton1(false, 3, false);
    } else if (autonMode == i++) {
        autonSkills();
    }
    while (true) { delay(20); }
}
