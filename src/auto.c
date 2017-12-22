#include "main.h"
#include "pid.h"
#include "setup.h"

int autonDrive(double dist, int wait) {
    pidDrive(dist);
    if (DL_pid.doneTime + wait < millis() && DR_pid.doneTime + wait < millis()) {
        DL_pid.doneTime = LONG_MAX;
        DR_pid.doneTime = LONG_MAX;
        return 1;
    }
    return 0;
}
int autonTurn(double angle, int wait) {
    pidTurn(angle);
    if (turn_pid.doneTime + wait < millis()) {
        turn_pid.doneTime = LONG_MAX;
        return 1;
    }
    return 0;
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
    switch (autonMode) { /*
         case 0:
             auton1(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &drfb_pid, &fb_pid, false, false);
             break;
         case 1:
             auton1(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &drfb_pid, &fb_pid, true, false);
             break;
         case 2:
             auton3(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &drfb_pid, &fb_pid, false);
             break;
         case 3:
             auton3(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &drfb_pid, &fb_pid, true);
             break;
         case 4:
             auton4(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &drfb_pid, &fb_pid);
             break;
         case 5:
             skills0(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &drfb_pid, &fb_pid);
             break;*/
    }
}
