#include "main.h"
#include "pid.h"
#include "setup.h"
/*
todo:

-3 cone 20 pt zone auton
-2 cone 10 pt zone + 1 cone stationary goal
*/

/*
 ######   ########     ###    ########
##    ##  ##     ##   ## ##   ##     ##
##        ##     ##  ##   ##  ##     ##
##   #### ########  ##     ## ########  ####
##    ##  ##   ##   ######### ##     ## ####
##    ##  ##    ##  ##     ## ##     ##  ##
 ######   ##     ## ##     ## ########  ##

 ######   ######   #######  ########  ########    ##     ##  ######
##    ## ##    ## ##     ## ##     ## ##          ###   ### ##    ##
##       ##       ##     ## ##     ## ##          #### #### ##
 ######  ##       ##     ## ########  ######      ## ### ## ##   ####
      ## ##       ##     ## ##   ##   ##          ##     ## ##    ##
##    ## ##    ## ##     ## ##    ##  ##          ##     ## ##    ##
 ######   ######   #######  ##     ## ########    ##     ##  ######
*/
// POST-CONDITION: motors set or reset
// note: this function blocks
// fix this: this function should probably take a loaderSide parameter
bool grabMGAuton(int driveT) {
    int i = 0, prevI = 0, u = 0;
    unsigned long prevT = millis();
    unsigned long breakT = 5000;
    double d0;
    bool usDone = false;
    while (true) {
        bool allowRepeat = true;
        while (allowRepeat) {
            allowRepeat = false;
            int j = 0;
            if (i == j++) {  // deploy
                if (mglGet() > 5) {
                    i++;
                    breakT = 3000;
                    mgl_pid.doneTime = LONG_MAX;
                    drfb_pid_auto.doneTime = LONG_MAX;
                    fb_pid_auto.doneTime = LONG_MAX;
                    DL_pid.doneTime = LONG_MAX;
                    DR_pid.doneTime = LONG_MAX;
                    u = 0;
                    usDone = false;
                    resetDriveEnc();
                } else {
                    setMGL(127);
                    pidFB(FB_UP_POS, 999999, true);
                    pidDRFB(35, 999999, true);
                    setDL(0);
                    setDR(0);
                }
            } else if (i == j++) {  // grab MG
                pidFB(FB_UP_POS, 999999, true);
                pidDRFB(35, 999999, true);
                double d = (eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN;
                if (usPredict(1) < 4.0 && d > 50.5) usDone = true;
                if (usDone) {
                    pidMGL(MGL_UP_POS, 999999);
                } else {
                    setMGL(127);
                }
                int h = 0;
                bool uDone = false;
                if (u == h++) {
                    if (pidDrive(51.5, driveT)) u++;
                } else if (u == h++) {
                    if (usPredict(1) < 3.5) {
                        pidDrive(51.5, 999999);
                    } else {
                        setDL(50);
                        setDR(50);
                    }
                    uDone = true;
                }
                if ((usPredict(1) < 4.5 && uDone) || d > 56) {
                    d0 = d;
                    i++;
                }
            } else if (i == j++) {  // lift MG
                pidFB(FB_UP_POS, 999999, true);
                pidDRFB(35, 999999, true);
                pidMGL(MGL_UP_POS, 999999);
                pidDrive(d0, 999999);
                if (mglGet() < 60) {  // low battery: 110
                    return true;
                }
            }
            if (i != prevI) {
                prevT = millis();
                allowRepeat = true;
            }
            prevI = i;
            // safety first (ptc tripped or robot got stuck)
            if (millis() - prevT > breakT) {
                resetMotors();
                return false;
            }
        }
        printf("grb mg ");
        printEnc();
        delay(5);
    }
}
// POST-CONDITION: motors set or reset
bool scoreMG(bool leftSide, int zone) {
    int i = 0, prevI = 0, u = 0, prevU = 0;
    unsigned long prevT = millis();
    unsigned long breakT = 3000;
    double driveD = 0;
    bool usDone;
    while (true) {
        bool allowRepeat = true;
        while (allowRepeat) {
            driveD = (eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN;
            allowRepeat = false;
            int j = 0;
            if (i == j++) {  // score MG
                if (zone == 20) {
                    pipeDrive();
                    if (setDownStack()) i++;
                    printf("setdownstk ");
                } else if (zone == 10) {
                    setDR(20);
                    setDL(20);
                    if (setDownStack()) i++;
                } else {
                    i++;
                }
            } else if (i == j++) {
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                usDone = false;
                i++;
            } else if (i == j++) {  // get rid of MG
                setDRFB(-20);
                pidFB(FB_UP_POS, 999999, true);
                if (zone == 20) {
                    setDR(-127);
                    setDL(-127);
                    if (usPredict(1) > 13) usDone = true;
                    if (usDone) {
                        setMGL(-60);
                    } else {
                        setMGL(20);
                    }
                    if (driveD < -8.0) i++;
                } else if (zone == 10) {
                    int h = 0;
                    if (u == h++) {
                        if (driveD > -4.0) {
                            pidMGL(MGL_DOWN_POS, 999999);
                        } else {
                            pidMGL(MGL_UP_POS, 0);
                        }
                        if (pidDrive(-6, 0)) {
                            resetDriveEnc();
                            DLturn_pid.doneTime = LONG_MAX;
                            DRturn_pid.doneTime = LONG_MAX;
                            u++;
                        }
                    } else if (u == h++) {
                        pidMGL(MGL_UP_POS, 0);
                        if (pidTurn(leftSide ? 30 : -30, 0)) {
                            resetDriveEnc();
                            DL_pid.doneTime = LONG_MAX;
                            DR_pid.doneTime = LONG_MAX;
                            u++;
                        }
                    } else if (u == h++) {
                        pidMGL(MGL_UP_POS, 0);
                        if (pidDrive(-20, 0)) u++;
                    } else if (u == h++) {
                        i++;
                    }
                } else {
                    setMGL(127);
                    setDL(-127);
                    setDR(-127);
                    if (driveD < -10.0) i++;
                }
            } else if (i == j++) {
                return true;
            }
            if (i != prevI || u != prevU) {
                prevT = millis();
                allowRepeat = true;
            }
            prevI = i;
            prevU = u;
            // safety first (ptc tripped or robot got stuck)
            if (millis() - prevT > breakT) {
                resetMotors();
                return false;
            }
        }
        printEnc();
        delay(5);
    }
}
/*
   ###    ##     ## ########  #######  ##    ##     #######
  ## ##   ##     ##    ##    ##     ## ###   ##    ##     ##
 ##   ##  ##     ##    ##    ##     ## ####  ##           ##
##     ## ##     ##    ##    ##     ## ## ## ##     #######
######### ##     ##    ##    ##     ## ##  ####    ##
##     ## ##     ##    ##    ##     ## ##   ###    ##
##     ##  #######     ##     #######  ##    ##    #########
*/
// loader cones
void auton2(bool leftSide, int stackH, int zone) {
    unsigned long prevT = millis(), funcT0 = millis(), t0 = millis();
    int i = 0, prevI = 0, u = 0, prevU = 0, y = 0, prevY = 0;
    double prevSens[3] = {0, 0, 0};
    unsigned long breakTime = 5000;
    int driveT = 80;
    double driveD = 0, d0 = 0;
    double da = 0;
    while (millis() - funcT0 < 15000) {
        bool allowRepeat = true;
        while (allowRepeat) {
            driveD = (eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN;
            allowRepeat = false;
            int j = 0;
            if (i == j++) {  // deploy
                if (grabMGAuton(driveT)) {
                    i++;
                    mgl_pid.doneTime = LONG_MAX;
                    DR_pid.doneTime = LONG_MAX;
                    DL_pid.doneTime = LONG_MAX;
                    resetDriveEnc();
                    u = 0;
                    y = 0;
                    t0 = LONG_MAX;
                    d0 = 999999.0;
                    breakTime = 4000;
                } else {
                    goto endLoop;
                }
            } else if (i == j++) {  // stack cone 1, align to loader
                bool mglDone = pidMGL(MGL_UP_POS, 0);
                int h = 0;
                bool uDone = false;
                if (u == h++) {
                    if (mglDone) {
                        if (stackConeQ(0)) u++;
                    } else {
                        pidFB(FB_UP_POS, 999999, true);
                        pidDRFB(35, 999999, true);
                    }
                } else if (u == h++) {
                    pidFB(FB_UP_POS, 999999, true);
                    pidDRFB(DRFB_LDR_UP, 999999, true);
                    uDone = true;
                }
                int g = 0;
                bool yDone = false;
                printf("%d ", (int)driveD);
                if (y == g++) {
                    if (d0 < 999) {
                        pidDriveShort(d0, 999999);
                    } else {
                        pidDrive(-36, 999999);
                    }
                    if (lt1Get() < LT_LIGHT && d0 > 999) {
                        t0 = millis();
                        d0 = driveD + 3;
                    }
                    if (millis() - t0 > 200 && driveD > d0 - 0.4) {
                        resetDriveEnc();
                        DLturn_pid.doneTime = LONG_MAX;
                        DRturn_pid.doneTime = LONG_MAX;
                        y++;
                    }
                } else if (y == g++) {
                    int a = leftSide ? 59 : -59;  // 60
                    if (mglGet() < MGL_UP_POS + 40 && pidTurn(a, driveT)) yDone = true;
                }
                if (uDone && yDone) {
                    i++;
                    autoStacking = false;
                }
            } else if (i == j++) {  // stack cones
                breakTime = 999999;
                if (autoStack(2, stackH)) {
                    i++;
                    DL_pid.doneTime = LONG_MAX;
                    DR_pid.doneTime = LONG_MAX;
                    resetDriveEnc();
                    breakTime = 3000;
                    da = limInt(drfba[stackH - 1][1], 25, 999);
                }
            } else if (i == j++) {
                pidDRFB(da, 999999, true);
                pidFB(FB_UP_POS, 999999, true);
                if (pidDrive(-5, driveT)) {
                    DLturn_pid.doneTime = LONG_MAX;
                    DRturn_pid.doneTime = LONG_MAX;
                    resetDriveEnc();
                    i++;
                }
            } else if (i == j++) {
                pidDRFB(da, 999999, true);
                pidFB(FB_UP_POS, 999999, true);
                double a;
                if (zone == 20) {
                    a = leftSide ? 126.5 : -126.5;
                } else if (zone == 10) {
                    a = leftSide ? 129.5 : -129.5;
                    pidMGL(MGL_MID_POS + 15, 999999);
                } else {
                    a = leftSide ? 99.5 : -99.5;
                }
                if (pidTurn(a, driveT)) {
                    DL_pid.doneTime = LONG_MAX;
                    DR_pid.doneTime = LONG_MAX;
                    resetDriveEnc();
                    i++;
                }
            } else if (i == j++) {
                pidDRFB(da, 999999, true);
                if (zone == 20 || zone == 10) {
                    pidFB(FB_UP_POS, 999999, true);
                } else {
                    pidMGL(MGL_DOWN_POS, 999999);
                    pidFB(FB_MID_POS, 999999, true);
                }
                double d;
                if (zone == 20) {
                    d = 43.5;
                } else if (zone == 10) {
                    d = 30;
                } else {
                    d = 12;
                }
                if (pidDrive(d, driveT)) {
                    i++;
                    DLturn_pid.doneTime = LONG_MAX;
                    DRturn_pid.doneTime = LONG_MAX;
                    resetDriveEnc();
                }
            } else if (i == j++) {
                if (zone != 5) {
                    pidDRFB(da, 999999, true);
                    pidMGL(MGL_MID_POS, 999999);
                    fb_pid_auto.sensVal = fbGet();
                    fb_pid_auto.target = 69;
                    pidFB(limInt(updatePID(&fb_pid_auto), -30, 30), 999999, true);
                    double a;
                    if (zone == 20) {
                        a = leftSide ? -45 : 45;
                    } else {
                        a = leftSide ? -45 : 45;
                    }
                    if (pidTurn(a, driveT)) i++;
                } else {
                    i++;
                }
            } else if (i == j++) {
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                i++;
            } else if (i == j++) {  // get into MG scoring position
                if (zone != 5) {
                    pidDRFB(drfba[stackH - 1][1], 999999, true);
                    prevSens[0] = prevSens[1];
                    prevSens[1] = prevSens[2];
                    prevSens[2] = eDLGet() + eDRGet();
                    if (zone == 20) {
                        setDL(127);
                        setDR(127);
                        pidMGL(MGL_MID_POS, 999999);
                        pidFB(69, 999999, true);
                        if (eDLGet() + eDRGet() - prevSens[1] <= 4 && driveD > 20.0) { i++; }
                    } else if (zone == 10) {
                        pidFB(69, 999999, true);
                        pidMGL(MGL_MID_POS, 999999);
                        setDL(80);
                        setDR(80);
                        if ((eDLGet() + eDRGet() - prevSens[1] <= 15 && driveD > 3.0)) i++;
                    }
                } else {
                    i++;
                }
            } else if (i == j++) {
                u = 0;
                i++;
                resetDriveEnc();
            } else if (i == j++) {
                scoreMG(leftSide, zone);
                goto endLoop;
            }
            if (i != prevI || u != prevU || y != prevY) {
                prevT = millis();
                allowRepeat = true;
            }
            prevI = i;
            prevU = u;
            prevY = y;
            // safety first (ptc tripped or robot got stuck)
            if (millis() - prevT > breakTime) {
                printf("\n\nTIMEOUT (%ld ms)\n\n", breakTime);
                goto endLoop;
            }
        }
        printEnc_all();
        delay(5);
    }
endLoop:
    resetMotors();
    printf("\n\nTIME: %lf\n", (millis() - funcT0) / 1000.0);
}
/*
   ###    ##     ## ########  #######  ##    ##     #######
  ## ##   ##     ##    ##    ##     ## ###   ##    ##     ##
 ##   ##  ##     ##    ##    ##     ## ####  ##           ##
##     ## ##     ##    ##    ##     ## ## ## ##     #######
######### ##     ##    ##    ##     ## ##  ####           ##
##     ## ##     ##    ##    ##     ## ##   ###    ##     ##
##     ##  #######     ##     #######  ##    ##     #######
*/
// grab wall field cones
void auton3(bool leftSide, int stackH, bool loaderSide, int zone) {
    unsigned long prevT = millis(), funcT0 = millis(), t0 = millis();
    int i = 0, prevI = 0, u = 0, prevU = 0, y = 0, prevY = 0;
    unsigned long breakTime = 5000;
    int driveT = 80;
    double d0, d0L, d0R, driveD, driveDL, driveDR, driveAL, driveAR;
    bool driveDone = false;
    while (millis() - funcT0 < 15000) {
        bool allowRepeat = true;
        while (allowRepeat) {
            allowRepeat = false;
            driveD = (eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN;
            driveDL = eDLGet() / DRIVE_TICKS_PER_IN;
            driveDR = eDRGet() / DRIVE_TICKS_PER_IN;
            driveAL = eDLGet() / DRIVE_TICKS_PER_DEG;
            driveAR = eDRGet() / DRIVE_TICKS_PER_DEG;
            int j = 0;
            if (i == j++) {  // deploy
                if (grabMGAuton(driveT)) {
                    i++;
                    mgl_pid.doneTime = LONG_MAX;
                    DRshort_pid.doneTime = LONG_MAX;
                    DLshort_pid.doneTime = LONG_MAX;
                    DR_pid.doneTime = LONG_MAX;
                    DL_pid.doneTime = LONG_MAX;
                    resetDriveEnc();
                    d0L = 999999.0;
                    d0R = 999999.0;
                    u = 0;
                } else {
                    goto endLoop;
                }
            } else if (i == j++) {
                int h = 0;
                bool uDone = false;
                bool mglDone = pidMGL(MGL_UP_POS, 0);
                if (u == h++) {
                    if (mglDone) {
                        if (stackConeQ(0)) u++;
                    } else {
                        pidFB(FB_UP_POS, 999999, true);
                        pidDRFB(35, 999999, true);
                    }
                } else if (u == h++) {
                    pidFB(FB_MID_POS + 20, 999999, true);
                    if (fbGet() > FB_MID_POS + 10) {
                        pidDRFB(20, 999999, true);
                    } else {
                        pidDRFB(0, 999999, true);
                    }
                    if (fbGet() > FB_MID_POS + 15) uDone = true;
                }
                driveDone = false;
                if (d0L < 999 && d0R < 999) {
                    driveDone = pidTurnSweep(d0L, d0R, true, true, true, 150);
                    if (fabs(driveDL - d0L) > 0.7 || fabs(driveDR - d0R) > 0.7) {
                        DLshort_pid.doneTime = LONG_MAX;
                        DRshort_pid.doneTime = LONG_MAX;
                        driveDone = false;
                    }
                } else {
                    pidTurnSweep(-16, -16, true, driveDL < -0.9, false, 999999);
                }
                if (lt2Get() < LT_LIGHT && d0L > 999 && d0R > 999) {
                    d0L = driveDL - 0.5;  // 2
                    d0R = driveDR - 0.5;
                }
                if (uDone && driveDone) {
                    resetDriveEnc();
                    DLturn_pid.doneTime = LONG_MAX;
                    DRturn_pid.doneTime = LONG_MAX;
                    i++;
                }
            } else if (i == j++) {  // hover cone 2
                printf("trn c2 ");
                pidDRFB(20, 999999, true);
                pidFB(FB_MID_POS + 20, 999999, true);
                if (drfbGet() > 15 && fbGet() > FB_MID_POS + 10 && pidTurnSweep(-48, 46, true, driveAL < -30, false, driveT)) {
                    t0 = LONG_MAX;
                    setDL(0);
                    setDR(0);
                    i++;
                }
            } else if (i == j++) {  // grab cone 2 fix this: below this point function not done yet
                printf("grb c2 ");
                setDRFB(-127);
                if (drfbGet() < 7) {
                    if (fbGet() > FB_MID_POS - 1) {
                        setFB(-127);
                    } else {
                        setFB(50);
                    }
                } else {
                    pidFB(FB_MID_POS + 20, 999999, true);
                }
                if (fbGet() < FB_MID_POS + 2 && drfbGet() < 1.5) t0 = millis();
                if ((long)millis() - (long)t0 > 150L) {
                    i++;
                    resetDriveEnc();
                    DL_pid.doneTime = LONG_MAX;
                    DR_pid.doneTime = LONG_MAX;
                    u = 0;
                    y = 0;
                }
            } else if (i == j++) {
                printf("stk c2 ");
                int h = 0;
                bool uDone = false;
                if (u == h++) {
                    if (liftConeQ(1)) u++;
                } else if (u == h++) {
                    if (stackConeQ(1)) u++;
                } else if (u == h++) {
                    pidFB(FB_MID_POS + 20, 999999, true);
                    if (fbGet() > FB_MID_POS + 20) {
                        pidDRFB(25, 999999, true);
                    } else {
                        pidDRFB(drfba[1][1], 999999, true);
                    }
                    uDone = true;
                }
                int g = 0;
                bool yDone = false;
                if (y == g++) {
                    if (pidDrive(-20, driveT)) {
                        resetDriveEnc();
                        DLturn_pid.doneTime = LONG_MAX;
                        DRturn_pid.doneTime = LONG_MAX;
                        y++;
                    }
                } else if (y == g++) {
                    if (pidTurn(130, driveT)) {
                        resetDriveEnc();
                        DL_pid.doneTime = LONG_MAX;
                        DR_pid.doneTime = LONG_MAX;
                        y++;
                    }
                } else if (y == g++) {
                    if (pidDrive(55, driveT)) {
                        resetDriveEnc();
                        DL_pid.doneTime = LONG_MAX;
                        DR_pid.doneTime = LONG_MAX;
                        y++;
                    }
                } else if (y == g++) {
                    if (pidTurnSweep(25, 10, true, true, false, driveT)) {
                        resetDriveEnc();
                        yDone = true;
                    }
                }
                if (uDone && yDone) i++;
            } else if (i == j++) {
                goto endLoop;
            }
            if (i != prevI || u != prevU || y != prevY) {
                prevT = millis();
                allowRepeat = true;
            }
            prevI = i;
            prevU = u;
            prevY = y;
            // safety first (ptc tripped or robot got stuck)
            if (millis() - prevT > breakTime) goto endLoop;
        }
        printEnc();
        delay(5);
    }
endLoop:
    resetMotors();
    printf("\n\nTIME: %lf\n", (millis() - funcT0) / 1000.0);
}
/*
   ###    ##     ## ########  #######  ##    ## ##
  ## ##   ##     ##    ##    ##     ## ###   ## ##    ##
 ##   ##  ##     ##    ##    ##     ## ####  ## ##    ##
##     ## ##     ##    ##    ##     ## ## ## ## ##    ##
######### ##     ##    ##    ##     ## ##  #### #########
##     ## ##     ##    ##    ##     ## ##   ###       ##
##     ##  #######     ##     #######  ##    ##       ##
*/
// claw in front of cone when scoring
// grab wall field cones
void auton4(bool leftSide, int stackH, bool loaderSide, int zone) {
    unsigned long prevT = millis(), funcT0 = millis(), t0 = millis();
    int i = 0, prevI = 0, u = 0, prevU = 0, y = 0, prevY = 0;
    unsigned long breakTime = 3500;
    int driveT = 80;
    double d0, driveD, driveDL, driveDR, driveAL, driveAR;
    double prevSens[3] = {0, 0, 0};
    while (millis() - funcT0 < 15000) {
        bool allowRepeat = true;
        while (allowRepeat) {
            allowRepeat = false;
            driveD = (eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN;
            driveDL = eDLGet() / DRIVE_TICKS_PER_IN;
            driveDR = eDRGet() / DRIVE_TICKS_PER_IN;
            driveAL = eDLGet() / DRIVE_TICKS_PER_DEG;
            driveAR = eDRGet() / DRIVE_TICKS_PER_DEG;
            int j = 0;
            if (i == j++) {  // deploy
                if (grabMGAuton(driveT)) {
                    i++;
                    resetDriveEnc();
                    u = 0;
                } else {
                    goto endLoop;
                }
            } else if (i == j++) {  // hvr c2
                printf("hvr c2 ");
                pidFB(FB_UP_POS, 999999, true);
                pidMGL(MGL_UP_POS, 999999);
                if (mglGet() < 30) {
                    pidTurnSweep(10.5, 10, true, true, false, 999999);
                    if (driveDL > 9.5 && driveDR > 9) {
                        u = 0;
                        t0 = LONG_MAX;
                        i++;
                    }
                } else {
                    setDL(0);
                    setDR(0);
                }
                if (mglGet() < 5) {
                    pidDRFB(drfba[0][1] + 5, 999999, true);
                } else {
                    pidDRFB(35, 999999, true);
                }
            } else if (i == j++) {  // grb c2
                printf("grb c2 ");
                pidTurnSweep(10.5, 10, true, true, false, 999999);
                setMGL(-20);
                if (fbGet() < FB_MID_POS - 2) {
                    pidFB(FB_MID_POS, 999999, true);
                } else {
                    if (drfbGet() > 2) {
                        pidFB(FB_MID_POS + 20, 999999, true);
                    } else {
                        setFB(-127);
                    }
                }
                if (fbGet() < FB_UP_POS - 25) {
                    setDRFBUnlim(-127);
                } else {
                    setDRFB(-30);
                }
                if (fbGet() < FB_MID_POS + 1 && drfbGet() < 1.5 && t0 > millis()) t0 = millis();
                if ((long)millis() - (long)t0 > 200) {
                    u = 0;
                    y = 0;
                    DL_pid.doneTime = LONG_MAX;
                    DR_pid.doneTime = LONG_MAX;
                    resetDriveEnc();
                    i++;
                }
            } else if (i == j++) {  // stk c2 hvr c3
                printf("hvr c3");
                setMGL(-20);
                bool uDone = false;
                int h = 0;
                if (u == h++) {
                    if (liftConeQ(1)) u++;
                } else if (u == h++) {
                    if (stackConeQ(1)) u++;
                } else if (u == h++) {  // hvr c3
                    pidDRFB(27, 999999, true);
                    pidFB(FB_MID_POS + 20, 999999, true);
                    uDone = true;
                }
                int g = 0;
                bool yDone = false;
                if (y == g++) {
                    if (pidTurnSweep(-23, -7, true, driveDL < -2, false, driveT)) {
                        resetDriveEnc();
                        y++;
                    }
                } else if (y == g++) {
                    pidTurnSweep(1, 2.5, driveDR > 0.5, true, true, 999999);
                    if (driveDL > 0.5 && driveDR > 1.75) yDone = true;
                }
                if (uDone && yDone) {
                    t0 = LONG_MAX;
                    i++;
                }
            } else if (i == j++) {  // grb c3
                printf("grb c3 ");
                pidTurnSweep(1, 2.5, driveDR > 0.5, true, true, 999999);
                setDRFBUnlim(-127);
                setMGL(-20);
                if (fbGet() > FB_MID_POS - 2) {
                    setFB(-127);
                } else {
                    pidFB(FB_MID_POS, 999999, true);
                }
                if (fbGet() < FB_MID_POS + 1 && drfbGet() < 1 && t0 > millis()) t0 = millis();
                if ((long)millis() - (long)t0 > 200L) {
                    i++;
                    resetDriveEnc();
                    DL_pid.doneTime = LONG_MAX;
                    DR_pid.doneTime = LONG_MAX;
                    u = 0;
                    y = 0;
                }
            } else if (i == j++) {
                setMGL(-20);
                printf("stk c3 ");
                int h = 0;
                bool uDone = false;
                if (u == h++) {
                    if (liftConeQ(2)) u++;
                } else if (u == h++) {
                    pidFB(FB_UP_POS, 999999, true);
                    pidDRFB(drfba[2][1], 999999, true);
                    uDone = true;
                }
                int g = 0;
                bool yDone = false;
                if (y == g++) {
                    printf("drvBkwd ");
                    if (pidDrive(-10, driveT)) {
                        resetDriveEnc();
                        DLturn_pid.doneTime = LONG_MAX;
                        DRturn_pid.doneTime = LONG_MAX;
                        y++;
                    }
                } else if (y == g++) {
                    printf("trn ");
                    if (pidTurn(111.5, driveT)) {
                        resetDriveEnc();
                        y++;
                    }
                } else if (y == g++) {
                    printf("drvFwd ");
                    pidDrive(999, 999999);
                    if (driveDL > 50.0 && driveDR > 50.0) {
                        resetDriveEnc();
                        DLturn_pid.doneTime = LONG_MAX;
                        DRturn_pid.doneTime = LONG_MAX;
                        y++;
                    }
                } else if (y == g++) {
                    printf("swp ");
                    if (pidTurnSweep(90, 0, true, true, false, driveT)) {
                        resetDriveEnc();
                        yDone = true;
                    }
                }
                if (uDone && yDone) i++;
            } else if (i == j++) {  // get into MG scoring position
                printf("PpDrv");
                pidDRFB(drfba[stackH - 1][1], 999999, true);
                if (zone == 20) {
                    pidMGL(MGL_MID_POS, 999999);
                    fb_pid_auto.target = 69;
                    fb_pid_auto.sensVal = fbGet();
                    setFB(limInt(updatePID(&fb_pid_auto), -30, 127));
                    if (pipeDrive()) { i++; }
                } else if (zone == 10) {
                    pidFB(69, 999999, true);
                    pidMGL(MGL_MID_POS, 999999);
                    setDL(80);
                    setDR(80);
                    if (pipeDrive()) { i++; }
                }
            } else if (i == j++) {
                u = 0;
                i++;
                resetDriveEnc();
            } else if (i == j++) {
                printf("scrMG ");
                if (scoreMG(leftSide, zone)) {
                    i++;
                } else {
                    goto endLoop;
                }
            } else if (i == j++) {
                if (zone == 20) {
                    pidMGL(80, 999999);  // fix this wrong angle
                    setDL(0);
                    setDR(0);
                    pidFB(FB_UP_POS, 999999, true);
                    pidDRFB(drfba[stackH - 1][0] + 10, 999999, true);
                    printf("AUTON FINISHED");
                } else {
                    i++;
                }
            }
            if (i != prevI || u != prevU || y != prevY) {
                prevT = millis();
                allowRepeat = true;
            }
            prevI = i;
            prevU = u;
            prevY = y;
            // safety first (ptc tripped or robot got stuck)
            if (millis() - prevT > breakTime) goto endLoop;
        }
        printEnc();
        printf("aT %lf ", (millis() - funcT0) / 1000.0);
        delay(5);
    }
endLoop:
    resetMotors();
    printf("\n\nTIME: %lf\n", (millis() - funcT0) / 1000.0);
}

/*
   ###    ##     ## ########  #######  ##    ##
  ## ##   ##     ##    ##    ##     ## ###   ##
 ##   ##  ##     ##    ##    ##     ## ####  ##
##     ## ##     ##    ##    ##     ## ## ## ##
######### ##     ##    ##    ##     ## ##  ####
##     ## ##     ##    ##    ##     ## ##   ###
##     ##  #######     ##     #######  ##    ##

 ######  ##    ## #### ##       ##        ######
##    ## ##   ##   ##  ##       ##       ##    ##
##       ##  ##    ##  ##       ##       ##
 ######  #####     ##  ##       ##        ######
      ## ##  ##    ##  ##       ##             ##
##    ## ##   ##   ##  ##       ##       ##    ##
 ######  ##    ## #### ######## ########  ######
*/
void autonSkills() {
    int i = 0;
    unsigned long funcT0 = millis(), t0 = millis();
    double prevSens[3] = {0, 0, 0};
    auton4(true, 1, true, 20);
    resetDriveEnc();
    DLturn_pid.doneTime = LONG_MAX;
    DRturn_pid.doneTime = LONG_MAX;
    mgl_pid.doneTime = LONG_MAX;
    double drfbA = 0, fbA = FB_UP_POS;
    int driveT = 100;
    printf("\nauton finished\n\n");
    double driveD = 0;
    while (true) {
        driveD = (eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN;
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
            if (pidDrive(15.5, driveT)) {
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
            if (driveD > 15.0) {
                pidMGL(MGL_MID_POS, 999999);
            } else if (pidMGL(MGL_UP_POS, 0)) {
                setMGL(0);
            }
            if (pidDrive(33, driveT)) {
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
            if (pidDrive(33, driveT)) {  // 31(2)
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
            if (pidDrive(33, driveT)) {
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
            if (pidDrive(-20, driveT)) {
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
            if ((d > 5 && pidMGL(MGL_DOWN_POS, 0) && pidDrive(16, driveT)) || millis() - t0 > 4000) {
                printf("\nMG 3 slammed: get rid MG 3...\n\n");
                i++;
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                mgl_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // get rid of MG 3
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidDrive(-13.5, driveT)) {
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
            pidDrive(23, 999999);
            if (eDLGet() + eDRGet() - (prevSens[0] + prevSens[1] + prevSens[2]) / 3.0 <= 6 && (eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN > 8.0) {
                printf("\nsquared up against wall:\n\n");
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                resetDriveEnc();
                i++;
            }
        } else if (i == j++) {
            if (pidDrive(-20.5, driveT)) {
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
            pidDrive(93, driveT);
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN > 60.0) {
                printf("\nMG 4 grabbed: lift MG 4...\n\n");
                mgl_pid.doneTime = LONG_MAX;
                i++;
            }
        } else if (i == j++) {  // lift MG 4
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (pidDrive(93, driveT)) {
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
            if (pidDrive(19, driveT)) {
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
            pidDrive(35, 999999);
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
            pidDrive(-20, 999999);
            pidMGL(MGL_DOWN_POS, 999999);
            if ((eDLGet() + eDRGet()) * 0.5 / DRIVE_TICKS_PER_IN < -1.0) {
                mgl_pid.doneTime = LONG_MAX;
                t0 = millis();
                i++;
            }
        } else if (i == j++) {
            if (pidMGL(MGL_MID_POS + 8, 0) || millis() - t0 > 800) {
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
            if (pidDrive(16.5, driveT)) {
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
            if (pidDrive(31, driveT)) {
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
            if (pidDrive(60, driveT)) {
                i++;
                resetDriveEnc();
                mgl_pid.doneTime = LONG_MAX;
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
            }
        } else if (i == j++) {  // lift MG 6
            if (pidMGL(MGL_UP_POS, 0)) setMGL(0);
            if (mglGet() < 35 && pidDrive(-40, driveT)) {
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
            if (pidDrive(17, driveT)) {
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
        delay(5);
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
    auton4(true, 3, true, 20);
    /*
    if (autoSel.nAuton == 1) {
        auton1(autoSel.leftSide, autoSel.stackH, autoSel.loaderSide, autoSel.zone);
    } else if (autoSel.nAuton == 2) {
        auton2(autoSel.leftSide, autoSel.stackH, autoSel.zone);
    }
    while (true) { delay(9999); }*/
}
