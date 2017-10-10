#include "main.h"
#include "pid.h"
#include "setup.h"

void auton1(PidVars* DL_pid, PidVars* DR_pid, PidVars* DLturn_pid, PidVars* DRturn_pid, PidVars* arm_pid, PidVars* cb_pid) {
	//autonDrive(-10, DL_pid, DR_pid, false);
	unsigned long t0 = millis();
	double cbAngle = 180, armAngle = 69;
	int step = 0;
	while(digitalRead(MGL_LIM)) {
		setMGL(-127);
		delay(20);
	}
	resetMotors();
	resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
	delay(2000);
	t0 = millis();
	while (true) {/*
		if(!joystickGetDigital(1, 6, JOY_DOWN)) {
			resetMotors();
			resetDrive();
			t0 = millis();
			continue;
		}*/
		pidArm(arm_pid, armAngle);
		pidCB(cb_pid, cbAngle);
		switch(step) {
			case 0:
				cbAngle = 130;
				armAngle = 69;
				if(millis()-t0 < 3000) {
					setMGL(127);
				} else {
					setMGL(0);
					t0 = millis();
					step++;
				}
				break;
			case 1:
				step += autonDrive(-41, DL_pid, DR_pid, false);
				if(step == 2) {
					resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
				}
				if(millis()-t0 < 100) {
					setMGL(127);
				} else {
					setMGL(0);
				}
				setClaw(-20);
				break;
			case 2:
				step += autonDrive(-180, DLturn_pid, DRturn_pid, true);
				if(step == 3) {
					resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
				}
				if(digitalRead(MGL_LIM)) {
					setMGL(-127);
				} else {
					setMGL(0);
				}
				break;
			case 3:
				//finish lifting mobile goal
				if(digitalRead(MGL_LIM)) {
					setMGL(-127);
				} else {
					setMGL(0);
				}
				step += autonDrive(-50, DL_pid, DR_pid, false);
				if(step == 4) {
					resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
				}
				break;
			/*
			case 3:
				stack(arm_pid, cb_pid, 0);
				step += autonDrive(180, DL_pid, DR_pid, true);
				break;
			case 4:
				step += autonDrive(60, DL_pid, DR_pid, false);
				t0 = millis();
				break;
			case 5:
				if(millis()-t0 < 2000) {
					setMGL(127);
				} else {
					setMGL(0);
				}
				break;
			case 6:
				step += autonDrive(-20, DL_pid, DR_pid, false);
				break;*/
		}
		printEnc_pidDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
		delay(20);
	}
}
/*
 * Runs the user autonomous code. This function will be started in its own task with the default
 * priority and stack size whenever the robot is enabled via the Field Management System or the
 * VEX Competition Switch in the autonomous mode. If the robot is disabled or communications is
 * lost, the autonomous task will be stopped by the kernel. Re-enabling the robot will restart
 * the task, not re-start it from where it left off.
 *
 * Code running in the autonomous task cannot access information from the VEX Joystick. However,
 * the autonomous function can be invoked from another task if a VEX Competition Switch is not
 * available, and it can access joystick information if called in this way.
 *
 * The autonomous task may exit, unlike operatorControl() which should never exit. If it does
 * so, the robot will await a switch to another mode or disable/enable cycle.
 */
void autonomous() {
	auton1(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &arm_pid, &cb_pid);
}
