#include "main.h"
#include "pid.h"
#include "setup.h"

int autonDrive(double dist, int wait, PidVars *left, PidVars *right, bool turning) {
    pidDrive(dist, left, right, turning);
    if (left->doneTime + wait < millis() && right->doneTime + wait < millis()) {
        left->doneTime = LONG_MAX;
        right->doneTime = LONG_MAX;
        return 1;
    }
    return 0;
}
void spinCycle(unsigned long t0) {
    if (((millis() - t0) / 200) % 2 == 1) {
        setMGL(127);
    } else {
        setMGL(-127);
    }
}
void driverSkillsAuton(PidVars *DL_pid, PidVars *DR_pid, PidVars *DLturn_pid, PidVars *DRturn_pid, PidVars *arm_pid, PidVars *cb_pid) {
    printf("starting auton.....");
    unsigned long t0 = millis();
    double cbAngle = 180, armAngle = 75;
    int step = 0;
    resetMotors();
    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
    t0 = millis();
    int waitT = 200;
    while (!joystickGetDigital(1, 8, JOY_DOWN) && !joystickGetDigital(1, 8, JOY_UP) && !joystickGetDigital(1, 8, JOY_RIGHT)) {
        pidArm(arm_pid, armAngle);
        pidCB(cb_pid, cbAngle);
        switch (step) {
            case 0:
                setClaw(-30);
                cbAngle = 140;
                armAngle = 73;
                step += autonDrive(-60, waitT, DL_pid, DR_pid, false);
                if (step == 1) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 1:
                setMGL(-127);
                if (!digitalRead(MGL_LIM)) {
                    step++;
                }
                break;
            case 2:
                armAngle = getArm(0);
                cbAngle = getCB(0);
                step += autonDrive(55, waitT, DL_pid, DR_pid, false);
                if (step == 3) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 3:
                setClaw(25);
                step += autonDrive(45, waitT, DLturn_pid, DRturn_pid, true);
                if (step == 4) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 4:
                step += autonDrive(24, waitT, DL_pid, DR_pid, false);
                if (step == 5) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 5:
                step += autonDrive(90, waitT, DLturn_pid, DRturn_pid, true);
                if (step == 6) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                }
                break;
            case 6:
                setClaw(0);
                cbAngle = 140;
                if (millis() - t0 < 1800) {
                    setDL(-127);
                    setDR(-127);
                } else {
                    setDL(0);
                    setDR(0);
                    t0 = millis();
                    step++;
                }
                break;
            case 7:
                if (millis() - t0 < 750) {
                    setMGL(127);
                } else {
                    t0 = millis();
                    step++;
                }
                break;
            case 8:
                spinCycle(t0);
                if (millis() - t0 < 600) {
                    setDL(127);
                    setDR(127);
                } else {
                    setDL(0);
                    setDR(0);
                    t0 = millis();
                    step++;
                }
                break;
            case 9:
                setMGL(-127);
                if (!digitalRead(MGL_LIM)) {
                    step++;
                }
                break;
            default:
                resetMotors();
        }
        printEnc_pidDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
        delay(20);
    }
}
// MG 20pt LEFT SIDE
void auton0(PidVars *DL_pid, PidVars *DR_pid, PidVars *DLturn_pid, PidVars *DRturn_pid, PidVars *arm_pid, PidVars *cb_pid) {
    printf("starting auton.....");
    unsigned long t0 = millis();
    double cbAngle = 180, armAngle = 75;
    int step = 0;
    resetMotors();
    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
    t0 = millis();
    int waitT = 200;
    while (true) {
        pidArm(arm_pid, armAngle);
        pidCB(cb_pid, cbAngle);
        switch (step) {
            case 0:
                setClaw(-30);
                cbAngle = 140;
                armAngle = 73;
                step += autonDrive(-60, waitT, DL_pid, DR_pid, false);
                if (step == 1) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 1:
                setMGL(-127);
                if (!digitalRead(MGL_LIM)) {
                    step++;
                }
                break;
            case 2:
                armAngle = getArm(0);
                cbAngle = getCB(0);
                step += autonDrive(55, waitT, DL_pid, DR_pid, false);
                if (step == 3) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 3:
                setClaw(25);
                step += autonDrive(45, waitT, DLturn_pid, DRturn_pid, true);
                if (step == 4) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 4:
                step += autonDrive(24, waitT, DL_pid, DR_pid, false);
                if (step == 5) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 5:
                step += autonDrive(90, waitT, DLturn_pid, DRturn_pid, true);
                if (step == 6) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                }
                break;
            case 6:
                setClaw(0);
                cbAngle = 140;
                if (millis() - t0 < 1800) {
                    setDL(-127);
                    setDR(-127);
                } else {
                    setDL(0);
                    setDR(0);
                    t0 = millis();
                    step++;
                }
                break;
            case 7:
                if (millis() - t0 < 750) {
                    setMGL(127);
                } else {
                    t0 = millis();
                    step++;
                }
                break;
            case 8:
                spinCycle(t0);
                if (millis() - t0 < 600) {
                    setDL(127);
                    setDR(127);
                } else {
                    setDL(0);
                    setDR(0);
                    t0 = millis();
                    step++;
                }
                break;
            case 9:
                setMGL(-127);
                if (!digitalRead(MGL_LIM)) {
                    step++;
                }
                break;
            default:
                resetMotors();
        }
        printEnc_pidDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
        delay(20);
    }
}
// MG 20pt RIGHT SIDE
void auton1(PidVars *DL_pid, PidVars *DR_pid, PidVars *DLturn_pid, PidVars *DRturn_pid, PidVars *arm_pid, PidVars *cb_pid) {
    printf("starting auton.....");
    unsigned long t0 = millis();
    double cbAngle = 180, armAngle = 75;
    int step = 0;
    resetMotors();
    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
    t0 = millis();
    int waitT = 200;
    while (true) {
        pidArm(arm_pid, armAngle);
        pidCB(cb_pid, cbAngle);
        switch (step) {
            case 0:
                setClaw(-30);
                cbAngle = 140;
                armAngle = 73;
                step += autonDrive(-60, waitT, DL_pid, DR_pid, false);
                if (step == 1) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 1:
                setMGL(-127);
                if (!digitalRead(MGL_LIM)) {
                    step++;
                }
                break;
            case 2:
                armAngle = getArm(0);
                cbAngle = getCB(0);
                step += autonDrive(55, waitT, DL_pid, DR_pid, false);
                if (step == 3) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 3:
                setClaw(25);
                step += autonDrive(-45, waitT, DLturn_pid, DRturn_pid, true);
                if (step == 4) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 4:
                step += autonDrive(24, waitT, DL_pid, DR_pid, false);
                if (step == 5) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 5:
                step += autonDrive(-90, waitT, DLturn_pid, DRturn_pid, true);
                if (step == 6) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                }
                break;
            case 6:
                setClaw(0);
                cbAngle = 140;
                if (millis() - t0 < 1800) {
                    setDL(-127);
                    setDR(-127);
                } else {
                    setDL(0);
                    setDR(0);
                    t0 = millis();
                    step++;
                }
                break;
            case 7:
                if (millis() - t0 < 750) {
                    setMGL(127);
                } else {
                    t0 = millis();
                    step++;
                }
                break;
            case 8:
                spinCycle(t0);
                if (millis() - t0 < 600) {
                    setDL(127);
                    setDR(127);
                } else {
                    setDL(0);
                    setDR(0);
                    t0 = millis();
                    step++;
                }
                break;
            case 9:
                setMGL(-127);
                if (!digitalRead(MGL_LIM)) {
                    step++;
                }
                break;
            default:
                resetMotors();
        }
        printEnc_pidDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
        delay(20);
    }
}
// MG 10pt LEFT SIDE
void auton2(PidVars *DL_pid, PidVars *DR_pid, PidVars *DLturn_pid, PidVars *DRturn_pid, PidVars *arm_pid, PidVars *cb_pid) {
    unsigned long t0 = millis();
    double cbAngle = 180, armAngle = 75;
    int step = 0;
    resetMotors();
    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
    t0 = millis();
    int waitT = 200;
    while (true) {
        pidArm(arm_pid, armAngle);
        pidCB(cb_pid, cbAngle);
        switch (step) {
            case 0:
                setClaw(-30);
                cbAngle = 140;
                armAngle = 73;
                step += autonDrive(-60, waitT, DL_pid, DR_pid, false);
                if (step == 1) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 1:
                setMGL(-127);
                if (!digitalRead(MGL_LIM)) {
                    step++;
                }
                break;
            case 2:
                armAngle = getArm(0);
                cbAngle = getCB(0);
                step += autonDrive(55, waitT, DL_pid, DR_pid, false);
                if (step == 3) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 3:
                setClaw(30);
                step += autonDrive(163, waitT, DLturn_pid, DRturn_pid, true);
                if (step == 4) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                }
                break;
            case 4:
                cbAngle = 140;
                if (millis() - t0 < 800) {
                    setDL(-127);
                    setDR(-127);
                } else {
                    step++;
                }
                if (step == 5) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                }
                break;
            case 5:
                if (millis() - t0 < 1000) {
                    setMGL(127);
                } else {
                    t0 = millis();
                    step++;
                }
                break;
            case 6:
                spinCycle(t0);
                if (millis() - t0 > 600) {
                    step += autonDrive(22, waitT, DL_pid, DR_pid, false);
                }
                if (step == 7) {
                    t0 = millis();
                }
                break;
            case 7:
                setMGL(-127);
                if (!digitalRead(MGL_LIM)) {
                    step++;
                }
                break;
            default:
                resetMotors();
        }
        delay(20);
    }
}
// MG 10pt RIGHT SIDE
void auton3(PidVars *DL_pid, PidVars *DR_pid, PidVars *DLturn_pid, PidVars *DRturn_pid, PidVars *arm_pid, PidVars *cb_pid) {
    unsigned long t0 = millis();
    double cbAngle = 180, armAngle = 75;
    int step = 0;
    resetMotors();
    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
    t0 = millis();
    int waitT = 200;
    while (true) {
        pidArm(arm_pid, armAngle);
        pidCB(cb_pid, cbAngle);
        switch (step) {
            case 0:
                setClaw(-30);
                cbAngle = 140;
                armAngle = 73;
                step += autonDrive(-60, waitT, DL_pid, DR_pid, false);
                if (step == 1) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 1:
                setMGL(-127);
                if (!digitalRead(MGL_LIM)) {
                    step++;
                }
                break;
            case 2:
                armAngle = getArm(0);
                cbAngle = getCB(0);
                step += autonDrive(55, waitT, DL_pid, DR_pid, false);
                if (step == 3) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 3:
                setClaw(30);
                step += autonDrive(-163, waitT, DLturn_pid, DRturn_pid, true);
                if (step == 4) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                }
                break;
            case 4:
                cbAngle = 140;
                if (millis() - t0 < 800) {
                    setDL(-127);
                    setDR(-127);
                } else {
                    step++;
                }
                if (step == 5) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                }
                break;
            case 5:
                if (millis() - t0 < 1000) {
                    setMGL(127);
                } else {
                    t0 = millis();
                    step++;
                }
                break;
            case 6:
                spinCycle(t0);
                if (millis() - t0 > 600) {
                    step += autonDrive(22, waitT, DL_pid, DR_pid, false);
                }
                if (step == 7) {
                    t0 = millis();
                }
                break;
            case 7:
                setMGL(-127);
                if (!digitalRead(MGL_LIM)) {
                    step++;
                }
                break;
            default:
                resetMotors();
        }
        delay(20);
    }
}
// start: 8 inches from wall
void skills0(PidVars *DL_pid, PidVars *DR_pid, PidVars *DLturn_pid, PidVars *DRturn_pid, PidVars *arm_pid, PidVars *cb_pid) {
    int step = 0, substep = 0;
    unsigned long t0 = millis();
    double cbAngle = 150, armAngle = 72;
    int turnT = 1000, driveT = 1000;
    while (true) {
        pidArm(arm_pid, armAngle);
        pidCB(cb_pid, cbAngle);
        switch (step) {
            //-------- BEGIN 1st MG --------
            case 0:
                setClaw(-30);
                cbAngle = 150;
                armAngle = 72;
                step += autonDrive(-37, driveT, DL_pid, DR_pid, false);
                if (step == 1) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 1:
                setMGL(-127);
                if (!digitalRead(MGL_LIM)) {
                    step++;
                }
                break;
            case 2:
                armAngle = getArm(0);
                cbAngle = getCB(0);
                step += autonDrive(180, turnT, DLturn_pid, DRturn_pid, true);
                if (step == 3) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 3:
                setClaw(30);
                step += autonDrive(-28, driveT, DL_pid, DR_pid, false);
                if (step == 4) {
                    setClaw(0);
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 4:
                cbAngle = 150;
                armAngle = 72;
                step += autonDrive(55, turnT, DLturn_pid, DRturn_pid, true);
                if (step == 5) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 5:
                switch (substep) {
                    case 0:
                        substep += autonDrive(-13, driveT, DL_pid, DR_pid, false);
                        if (substep == 1) {
                            resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                        }
                        break;
                    case 1:
                        substep += autonDrive(-55, turnT, DLturn_pid, DRturn_pid, true);
                        if (substep == 2) {
                            resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                        }
                        break;
                    case 2:
                        substep += autonDrive(-29, driveT, DL_pid, DR_pid, false);
                        if (killPID(270, 1, DL_pid) + killPID(270, 1, DR_pid) > 0) {
                            substep++;
                        };
                        if (substep == 3) {
                            resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                            t0 = millis();
                        }
                        break;
                    case 3:
                        if (millis() - t0 < 900) {
                            setMGL(127);
                        } else {
                            substep++;
                            t0 = millis();
                        }
                        break;
                    case 4:
                        spinCycle(t0);
                        if (millis() - t0 > 400) {
                            substep += autonDrive(30, driveT, DL_pid, DR_pid, false);
                        }
                        if (substep == 5) {
                            setMGL(0);
                            resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                        }
                        break;
                    default:
                        substep = 0;
                        step++;
                }
                break;
            case 6:
                step++;
                break;
            //-------- END 1st MG --------
            //-------- BEGIN 2nd MG --------
            case 7:
                step += autonDrive(90, turnT, DLturn_pid, DRturn_pid, true);
                if (step == 8) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                }
                break;
            case 8:
                step += autonDrive(-26, 1.5 * driveT, DL_pid, DR_pid, false);
                if (millis() - t0 < 500) {
                    setMGL(127);
                } else {
                    setMGL(0);
                }
                if (step == 9) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 9:
                step += autonDrive(90, turnT, DLturn_pid, DRturn_pid, true);
                if (millis() - t0 < 1000) {
                    setMGL(127);
                } else {
                    setMGL(0);
                }
                if (step == 10) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 10:
                step += autonDrive(-37, driveT, DL_pid, DR_pid, false);
                if (abs(DL_pid->target - DL_pid->sensVal) < 110 || abs(DR_pid->target - DR_pid->sensVal) < 110) {
                    setMGL(-127);
                }
                if (step == 11) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 11:
                setMGL(-127);
                if (!digitalRead(MGL_LIM)) {
                    step++;
                }
                break;
            case 12:
                step += autonDrive(172, turnT, DLturn_pid, DRturn_pid, true);
                if (step == 13) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 13:
                if (killPID(300, 1, DL_pid) + killPID(300, 1, DR_pid) == 2) {
                    step++;
                }
                step += autonDrive(-52, driveT, DL_pid, DR_pid, false);
                if (killPID(300, 1, DL_pid) + killPID(300, 1, DR_pid) == 2) {
                    step++;
                }
                if (step == 14) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                }
                break;
            case 14:
                if (millis() - t0 < 1100) {
                    setMGL(127);
                } else {
                    setMGL(0);
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                    step++;
                }
                break;
            case 15:
                spinCycle(t0);
                if (millis() - t0 > 400) {
                    step += autonDrive(13, driveT, DL_pid, DR_pid, false);
                }
                if (step == 16) {
                    setMGL(0);
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                }
                break;
            //-------- END 2nd MG ----------
            //-------- BEGIN 3rd MG --------
            case 16:
                setMGL(-127);
                step += autonDrive(-90, turnT, DLturn_pid, DRturn_pid, true);
                if (step == 17) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 17:
                step += autonDrive(-26, driveT, DL_pid, DR_pid, false);
                if (step == 18) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 18:
                step += autonDrive(-88, 2 * turnT, DLturn_pid, DRturn_pid, true);
                if (step == 19) {
                    t0 = millis();
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 19:
                if (millis() - t0 > 800) {
                    step += autonDrive(-68, driveT, DL_pid, DR_pid, false);
                }
                if (millis() - t0 < 1000) {
                    setMGL(127);
                } else {
                    setMGL(0);
                }
                if (step == 20) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                }
                break;
            case 20:
                setMGL(-127);
                if (!digitalRead(MGL_LIM)) {
                    step++;
                }
                break;
            case 21:
                step += autonDrive(-32, driveT, DL_pid, DR_pid, false);
                if (step == 22) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 22:
                step += autonDrive(-55, turnT, DLturn_pid, DRturn_pid, true);
                if (step == 23) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 23:
                step += autonDrive(-13, driveT, DL_pid, DR_pid, false);
                if (step == 24) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 24:
                step += autonDrive(55, turnT, DLturn_pid, DRturn_pid, true);
                if (step == 25) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 25:
                step += autonDrive(-24, driveT, DL_pid, DR_pid, false);
                if (killPID(300, 1, DL_pid) + killPID(300, 1, DR_pid) == 2) {
                    step++;
                }
                if (step == 26) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                }
                break;
            case 26:
                if (millis() - t0 < 900) {
                    setMGL(127);
                } else {
                    step++;
                    t0 = millis();
                }
                break;
            case 27:
                spinCycle(t0);
                if (millis() - t0 < 500) {
                    step += autonDrive(30, driveT, DL_pid, DR_pid, false);
                } else {
                    setMGL(0);
                }
                if (step == 28) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                }
                break;
            default:
                resetMotors();
        }
        delay(20);
    }
}

void skills1(PidVars *DL_pid, PidVars *DR_pid, PidVars *DLturn_pid, PidVars *DRturn_pid, PidVars *arm_pid, PidVars *cb_pid) {
    return;

    int step = 0;
    unsigned long t0 = millis();
    while (true) {
        double cbAngle = 130, armAngle = 72;
        pidArm(arm_pid, armAngle);
        pidCB(cb_pid, cbAngle);
        switch (step) {
            case 0:
                step += autonDrive(-38, 200, DL_pid, DR_pid, false);
                if (step == 1) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    setDL(0);
                    setDR(0);
                }
                break;
            case 1:
                setMGL(-127);
                if (!digitalRead(MGL_LIM)) {
                    step++;
                }
                break;
            case 2:
                step += autonDrive(180, 200, DLturn_pid, DRturn_pid, true);
                if (step == 3) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    setDL(0);
                    setDR(0);
                    t0 = millis();
                }
                break;
            case 3:
                if (millis() - t0 < 2600) {
                    setDL(127);
                    setDR(127);
                    t0 = millis();
                } else if (millis() - t0 < 3000) {
                    setDL(-127);
                    setDR(-127);
                    t0 = millis();
                } else if (millis() - t0 < 400) {
                    setDL(-100);
                    setDR(100);
                    t0 = millis();
                } else if (millis() - t0 < 1000) {
                    setDL(120);
                    setDR(120);
                    t0 = millis();
                } else if (millis() - t0 < 950) {
                    spinCycle(t0);
                    setDL(-100);
                    setDR(-100);
                    t0 = millis();
                } else if (millis() - t0 < 1000) {
                    spinCycle(t0);
                    setDL(-127);
                    setDR(-127);
                } else {
                    setDL(0);
                    setDR(0);
                    step++;
                }
                break;
        }
        delay(20);
    }
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
    switch (autonMode) {
        case 0:
            auton0(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &arm_pid, &cb_pid);
            break;
        case 1:
            auton1(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &arm_pid, &cb_pid);
            break;
        case 2:
            auton2(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &arm_pid, &cb_pid);
            break;
        case 3:
            auton3(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &arm_pid, &cb_pid);
            break;
        case 4:
            skills0(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &arm_pid, &cb_pid);
            break;
        case 5:
            skills1(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &arm_pid, &cb_pid);
            break;
    }
}
