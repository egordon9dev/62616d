#include "main.h"
#include "pid.h"
#include "setup.h"

int autonDrive(double dist, PidVars *left, PidVars *right, bool turning) {
    pidDrive(dist, left, right, turning);
    int wait = 200;
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
void auton0(PidVars *DL_pid, PidVars *DR_pid, PidVars *DLturn_pid, PidVars *DRturn_pid, PidVars *arm_pid, PidVars *cb_pid) {
    printf("starting auton.....");
    unsigned long t0 = millis();
    double cbAngle = 180, armAngle = 75;
    int step = 0;
    resetMotors();
    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
    t0 = millis();
    bool pidRunning = false;
    while (true) {
        pidArm(arm_pid, armAngle);
        pidCB(cb_pid, cbAngle);
        switch (step) {
            case 0:
                setClaw(-127);
                if (millis() - t0 > 300) {
                    setClaw(-30);
                }
                cbAngle = 130;
                armAngle = 75;
                step += autonDrive(-68, DL_pid, DR_pid, false);
                if (step == 1) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    setDL(0);
                    setDR(0);
                    printf("\n\nstep: 1\n\n");
                }
                break;
            case 1:
                setMGL(-127);
                if (!digitalRead(MGL_LIM)) {
                    step += autonDrive(15, DLturn_pid, DRturn_pid, true);
                    if (step == 2) {
                        resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                        setDL(0);
                        setDR(0);
                        printf("\n\nstep: 2\n\n");
                        t0 = millis();
                    }
                }
                break;
            case 2:
                step += autonDrive(48, DL_pid, DR_pid, false);
                if (stack(arm_pid, cb_pid, 0)) {
                    setClaw(25);
                }
                if (step == 3) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                    printf("\n\nstep: 3\n\n");
                    setDL(0);
                    setDR(0);
                }
                break;
            case 3:
                if (stack(arm_pid, cb_pid, 0)) {
                    setClaw(25);
                }
                step += autonDrive(145, DLturn_pid, DRturn_pid, true);
                if (step == 4) {
                    setClaw(0);
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                    printf("\n\nstep: 4\n\n");
                }
                break;
            case 4:

                if (!digitalRead(MGL_LIM)) {
                    if (stack(arm_pid, cb_pid, 0)) {
                        setClaw(25);
                    }
                }
                if (millis() - t0 < 2500) {
                    setDL(-127);
                    setDR(-127);
                } else {
                    setDL(0);
                    setDR(0);
                    step++;
                    t0 = millis();
                    printf("\n\nstep 5:\n\n");
                }
                break;
            case 5:
                if (millis() - t0 < 750) {
                    setMGL(127);
                } else {
                    step++;
                    t0 = millis();
                    printf("\n\nstep 6:\n\n");
                }
                break;
            case 6:
                //-----------------------------------------------------------------------------------------------------------
                //									SPIN-CYCLE
                //-----------------------------------------------------------------------------------------------------------
                spinCycle(t0);

                if (millis() - t0 < 700) {
                    setDL(127);
                    setDR(127);
                } else {
                    step++;
                    setDL(0);
                    setDR(0);
                    t0 = millis();
                    printf("\n\nstep 7:\n\n");
                }
                break;
            case 7:
                if (digitalRead(MGL_LIM)) {
                    setMGL(-127);
                } else {
                    step++;
                    t0 = millis();
                    printf("\n\nstep 7:\n\n");
                }
                break;

                /*
                case 4:
                        setDR(-127);
                        setDL(-127);
                        delay(1000);

                        setMGL(127);
                        delay(800);
                        setMGL(0);

                        setDL(127);
                        setDR(127);
                        delay(300);
                        setMGL(-127);
                        delay(1700);

                        setDL(0);
                        setDR(0);
                        setMGL(0);
                        setClaw(0);
                        step++;

                        if(step == 9) {
                                t0 = millis();
                                printf("\n\nstep: 9\n\n");
                        }
                        break;*/
        }
        printEnc_pidDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
        delay(20);
    }
}
void auton1(PidVars *DL_pid, PidVars *DR_pid, PidVars *DLturn_pid, PidVars *DRturn_pid, PidVars *arm_pid, PidVars *cb_pid) {}
void auton2(PidVars *DL_pid, PidVars *DR_pid, PidVars *DLturn_pid, PidVars *DRturn_pid, PidVars *arm_pid, PidVars *cb_pid) {}
void auton3(PidVars *DL_pid, PidVars *DR_pid, PidVars *DLturn_pid, PidVars *DRturn_pid, PidVars *arm_pid, PidVars *cb_pid) {}
// start: 8 inches from wall
void skills0(PidVars *DL_pid, PidVars *DR_pid, PidVars *DLturn_pid, PidVars *DRturn_pid, PidVars *arm_pid, PidVars *cb_pid) {
    int step = 0, substep = 0;
    unsigned long t0 = millis();
    while (true) {
        double cbAngle = 130, armAngle = 72;
        pidArm(arm_pid, armAngle);
        pidCB(cb_pid, cbAngle);
        switch (step) {
            //-------- BEGIN 1st MG --------
            case 0:
                setClaw(-25);
                cbAngle = 130;
                armAngle = 72;
                step += autonDrive(-41, DL_pid, DR_pid, false);
                if (abs(DL_pid->target - DL_pid->sensVal) < 110 || abs(DR_pid->target - DR_pid->sensVal) < 110) {
                    setMGL(-127);
                }
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
                stack(arm_pid, cb_pid, 0);
                step += autonDrive(180, DLturn_pid, DRturn_pid, true);
                if (step == 3) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                }
                break;
            case 3:
                setClaw(25);
                step += autonDrive(-20, DL_pid, DR_pid, false);
                if (step == 4) {
                    setClaw(0);
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 4:
                cbAngle = 130;
                armAngle = 72;
                step += autonDrive(15, DLturn_pid, DRturn_pid, true);
                if (step == 5) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                }
                break;
            case 5: /*
                 if (millis() - t0 < 1300) {
                     setDL(-127);
                     setDR(-127);
                 } else if (millis() - t0 < 2300) {
                     setMGL(127);
                 } else if (millis() - t0 < 2500) {
                     // spinCycle(t0);
                     setDL(127);
                     setDR(127);
                 } else {
                     t0 = millis();
                     step++;
                 }*/
                switch (substep) {
                    case 0:
                        substep += autonDrive(-43, DL_pid, DR_pid, false);
                        if (substep == 1) {
                            resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                            t0 = millis();
                        }
                        break;
                    case 1:
                        if (millis() - t0 < 600) {
                            setMGL(127);
                        } else {
                            substep++;
                            t0 = millis();
                        }
                        break;
                    case 2:
                        if (millis() - t0 < 500) {
                            spinCycle(t0);
                        } else {
                            setMGL(0);
                        }
                        substep += autonDrive(33, DL_pid, DR_pid, false);
                        if (substep == 3) {
                            resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                            t0 = millis();
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
                step += autonDrive(90, DLturn_pid, DRturn_pid, true);
                if (step == 8) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                }
                break;
            case 8:
                step += autonDrive(-15, DL_pid, DR_pid, false);
                if (millis() - t0 < 1000) {
                    setMGL(127);
                } else {
                    setMGL(0);
                }
                if (step == 9) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 9:
                step += autonDrive(90, DLturn_pid, DRturn_pid, true);
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
                step += autonDrive(-40, DL_pid, DR_pid, false);
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
                step += autonDrive(177, DLturn_pid, DRturn_pid, true);
                if (step == 13) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 13:
                step += autonDrive(-53, DL_pid, DR_pid, false);
                if (step == 14) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                }
                break;
            case 14:
                if (millis() - t0 < 500) {
                    setMGL(127);
                } else {
                    setMGL(0);
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    step++;
                }
                break;
            case 15:
                spinCycle(t0);
                step += autonDrive(10, DL_pid, DR_pid, false);
                if (step == 16) {
                    setMGL(0);
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                    t0 = millis();
                }
                break;
            //-------- END 2nd MG ----------
            //-------- BEGIN 3rd MG --------
            case 16:
                step += autonDrive(-90, DLturn_pid, DRturn_pid, true);
                if (step == 17) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 17:
                step += autonDrive(-37, DL_pid, DR_pid, false);
                if (step == 18) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 18:
                step += autonDrive(-79, DLturn_pid, DRturn_pid, true);
                if (step == 19) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 19:
                step += autonDrive(-65, DL_pid, DR_pid, false);
                if (step == 20) {
                    resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
                }
                break;
            case 20:
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
                step += autonDrive(-38, DL_pid, DR_pid, false);
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
                step += autonDrive(180, DLturn_pid, DRturn_pid, true);
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
