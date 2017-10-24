#include "main.h"
#include "pid.h"
#include "setup.h"

int autonDrive(double dist, PidVars *left, PidVars *right, bool turning)
{
	pidDrive(dist, left, right, turning);
	int wait = 200;
	if (left->doneTime + wait < millis() && right->doneTime + wait < millis())
	{
		left->doneTime = LONG_MAX;
		right->doneTime = LONG_MAX;
		return 1;
	}
	return 0;
}

void auton1(PidVars *DL_pid, PidVars *DR_pid, PidVars *DLturn_pid, PidVars *DRturn_pid, PidVars *arm_pid, PidVars *cb_pid)
{
	printf("starting auton.....");
	unsigned long t0 = millis();
	double cbAngle = 180, armAngle = 75;
	int step = 0;
	resetMotors();
	resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
	t0 = millis();
	while (true)
	{
		pidArm(arm_pid, armAngle);
		pidCB(cb_pid, cbAngle);
		switch (step)
		{
		case 0:
			setClaw(-127);
			if (millis() - t0 > 300)
			{
				setClaw(-30);
			}
			cbAngle = 130;
			armAngle = 75;
			step += autonDrive(-68, DL_pid, DR_pid, false);
			if (step == 1)
			{
				resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
				setDL(0);
				setDR(0);
				printf("\n\nstep: 1\n\n");
			}
			break;
		case 1:
			step += autonDrive(10, DLturn_pid, DRturn_pid, true);
			if (step == 2)
			{
				resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
				setDL(0);
				setDR(0);
				printf("\n\nstep: 2\n\n");
				t0 = millis();
			}
			break;
		case 2:
			setMGL(-127);
			if (millis() - t0 > 2500)
			{
				stack(arm_pid, cb_pid, 1);
			}
			if (millis() - t0 > 2700)
			{
				setClaw(25);
			}
			if (!digitalRead(MGL_LIM))
			{
				step += autonDrive(48, DL_pid, DR_pid, false);
			}

			if (step == 3)
			{
				resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
				setDL(0);
				setDR(0);
				printf("\n\nstep: 3\n\n");
			}
			break;
		case 3:
			setMGL(-127);
			if (millis() - t0 > 2500)
			{
				stack(arm_pid, cb_pid, 1);
			}
			if (millis() - t0 > 2700)
			{
				setClaw(25);
			}
			step += autonDrive(160, DLturn_pid, DRturn_pid, true);
			if (step == 4)
			{
				resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
				t0 = millis();
				printf("\n\nstep: 4\n\n");
			}
			break;
		case 4:
			if (millis() - t0 < 3500)
			{
				setDL(-127);
				setDR(-127);
			}
			else
			{
				setDL(0);
				setDR(0);
				step++;
				t0 = millis();
				printf("\n\nstep 5:\n\n");
			}
			break;
		case 5:
			if (millis() - t0 < 750)
			{
				setMGL(127);
			}
			else
			{
				step++;
				t0 = millis();
				printf("\n\nstep 6:\n\n");
			}
			break;
		case 6:
			//-----------------------------------------------------------------------------------------------------------
			//									SPIN-CYCLE
			//-----------------------------------------------------------------------------------------------------------
			if (((millis() - t0) / 200) % 2 == 1)
			{
				setMGL(127);
			}
			else
			{
				setMGL(-127);
<<<<<<< HEAD
				if(millis() - t0 > 2500) {
					stack(arm_pid, cb_pid, 1);
				}
				if(millis() - t0 > 2700) {
					setClaw(25);
				}
				step += autonDrive(160, DLturn_pid, DRturn_pid, true);
				if(step == 4) {
					resetDrive(DL_pid, DR_pid, DLturn_pid, DRturn_pid);
					t0 = millis();
					printf("\n\nstep: 4\n\n");
				}
				break;
			case 4:
				if(millis()-t0 < 2700) {
					setDL(-127);
					setDR(-127);
				} else {
					setDL(0);setDR(0);
					step++;
					t0 = millis();
					printf("\n\nstep 5:\n\n");
				}
				break;
			case 5:
				if(millis()-t0 < 750) {
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
				if(((millis()-t0)/200) % 2 == 1) {
					setMGL(127);
				} else {
					setMGL(-127);
				}


				if(millis()-t0 < 700) {
					setDL(127);
					setDR(127);
				} else {
					step++;
					setDL(0);setDR(0);
					t0 = millis();
					printf("\n\nstep 7:\n\n");
				}
				break;
			case 7:
				if(digitalRead(MGL_LIM)) {
					setMGL(-127);
				} else {
					step++;
					t0 = millis();
					printf("\n\nstep 7:\n\n");
				}
				break;
=======
			}
>>>>>>> 81cf8fbdb018f61cee6fb786127c6f129a154324

			if (millis() - t0 < 1300)
			{
				setDL(127);
				setDR(127);
			}
			else
			{
				step++;
				setDL(0);
				setDR(0);
				t0 = millis();
				printf("\n\nstep 7:\n\n");
			}
			break;
		case 7:
			if (digitalRead(MGL_LIM))
			{
				setMGL(-127);
			}
			else
			{
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
void autonomous()
{
	switch (autonSelect)
	{
	case 1:
		auton1(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &arm_pid, &cb_pid);
		break;

	case 2:
		//autonSkills();
		break;

	case 3:
		return;
		break;
	}
}
