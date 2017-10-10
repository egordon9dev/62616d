#include "main.h"
#include "pid.h"
#include "encoders.h"
#include <math.h>
#include "ports.h"

const int MAX_POWER = 127;
bool wtf = false;

//angle settings for autonomous cone stacking
const int ARM = 0;
const int CB = 1;
int stackAngles[][2] = {
//	  ARM | CB
	{ 69,   90 },
	{ 69,   100 },
	{ 69,   110 },
};
int returnAngle[] = { 60, 300 };

void limMotorVal(int* n) {
	if(*n > MAX_POWER) *n = MAX_POWER;
	if(*n < -MAX_POWER) *n = -MAX_POWER;
}
int getLimMotorVal(int n) {
	if(n > MAX_POWER) return MAX_POWER;
	if(n < -MAX_POWER) return -MAX_POWER;
	return n;
}
void setDL(int n) {//	set left drive motors
	limMotorVal(&n);
	motorSet(M0_1, n);
	motorSet(M2, n);
}
void setDR(int n) {//	set right drive motors
	limMotorVal(&n);
	motorSet(M3_4, -n);
	motorSet(M5, -n);
}
// if turning, dist is in degrees
// if not turning, dist is in inches
void pidDrive(double dist, PidVars* left, PidVars* right, bool turning) {
	left->sensVal = eDLGet();
	right->sensVal = eDRGet();
	if(turning) {
		left->target = dist * -3.5610;
		int leftPow = updatePID(left);
		limMotorVal(&leftPow);
		setDL(leftPow);
		right->target = dist * 3.5610;
		int rightPow = updatePID(right);
		limMotorVal(&rightPow);
		setDR(rightPow);
	} else {
		//89 inches = 2457 ticks : 2457.0/89.0 = 27.6067
		left->target = dist * 27.6067;
		int leftPow = updatePID(left);
		limMotorVal(&leftPow);
		setDL(leftPow);
		right->target = dist * 27.6067;
		int rightPow = updatePID(right);
		limMotorVal(&rightPow);
		setDR(rightPow);
	}
}
void setArm(int n) {//	set main 4 bar lift
	limMotorVal(&n);
	motorSet(M9_10, -n);
}
void setCB(int n) {//	set chain bar lift
	/*int maxPow = 0;
	int weakZone = 60;*/
	limMotorVal(&n);/*
	if(eCBGet() > CB_MAX - weakZone && n > maxPow) {
		n = maxPow;
	} else if(eCBGet() < CB_MIN + weakZone && n < -maxPow) {
		n = -maxPow;
	}*/
	motorSet(M7_8, n);
}
void pidCB(PidVars* cb_pid, double a) { // set chain-bar angle with PID
	cb_pid->target = a;
	cb_pid->sensVal = eCBGet();
	setCB(updatePID(cb_pid));
}
void pidArm(PidVars* arm_pid, double a) { // set arm angle with PID
	arm_pid->target = a;
	arm_pid->sensVal = eArmGet();
	setArm(updatePID(arm_pid));
}
//set chain bar and arm with PID to stack given cone
void stack(PidVars* arm_pid, PidVars* cb_pid, int cone) {
	pidCB(cb_pid, stackAngles[cone][CB]);
	pidArm(arm_pid, stackAngles[cone][ARM]);
}
//return lift to pick up cones
void returnLift(PidVars* arm_pid, PidVars* cb_pid) {
	pidCB(cb_pid, returnAngle[CB]);
	pidArm(arm_pid, returnAngle[ARM]);
}
void setClaw(int n) {//	set claw
	limMotorVal(&n);
	motorSet(M6, n);
}
void setMGL(int n) {//	set mobile goal lift
	limMotorVal(&n);
	motorSet(M11, -n);
}
//----- updates Arm and Chain-Bar -----//
void updateLift(PidVars* arm_pid, PidVars* cb_pid) {
	if(joystickGetDigital(1, 7, JOY_RIGHT)) {
		returnLift(arm_pid, cb_pid);
		return;
	}
	if(joystickGetDigital(1, 7, JOY_UP)) {
		// insert auto stack code here..............
		// .........................................
		return;
	}
	//----- update arm -----//
	const int t = 15;
	int j2 = joystickGetAnalog(1, 2);
	if(abs(j2) > t) {
		setArm(j2);
	} else {
		setArm(0);
	}
	//------ update chain bar -----//
	if(joystickGetDigital(1, 5, JOY_UP)) {
		setCB(-127);
	} else if(joystickGetDigital(1, 5, JOY_DOWN)) {
		setCB(127);
	} else {
		setCB(0);
	}
}
void printEnc() {
	printf("Arm: %d\tCB: %d\tDL: %d\tDR: %d\n", eArmGet(), eCBGet(), eDLGet(), eDRGet());
}
void printEnc_pidDrive(PidVars* DR_pid, PidVars* DL_pid, PidVars* DRturn_pid, PidVars* DLturn_pid) {
	printf("DL: %d/%d\tDR: %d/%d\tDLt: %d/%d\tDRt: %d/%d\n", (int)DL_pid->sensVal, (int)DL_pid->target, (int)DR_pid->sensVal, (int)DR_pid->target, (int)DLturn_pid->sensVal, (int)DLturn_pid->target, (int)DRturn_pid->sensVal, (int)DRturn_pid->target);
}
void printEnc_pidArmCB(PidVars* arm_pid, PidVars* cb_pid) {
	printf("arm: %d/%d\tcb: %d/%d\t", (int)arm_pid->sensVal, (int)arm_pid->target, (int)cb_pid->sensVal, (int)cb_pid->target);
}
/*
	pid1 = left pid for drive or drive turning
	pid2 = right pid for drive or drive turning
*/
int autonDrive(double dist, PidVars* pid1, PidVars* pid2, bool turning) {
	if((abs(pid1->sensVal - pid1->target) > pid1->DONE_ZONE || abs(pid2->sensVal - pid2->target) > pid2->DONE_ZONE) ||
	(pid1->doneTime + 300 < millis() && pid2->doneTime + 300 < millis())) {
		pidDrive(dist, pid1, pid2, turning);
		return 0;
	} else {
		pid1->doneTime = MASSIVE;
		pid2->doneTime = MASSIVE;
		return 1;
	}
}
void auton1(PidVars* DL_pid, PidVars* DR_pid, PidVars* DLturn_pid, PidVars* DRturn_pid, PidVars* arm_pid, PidVars* cb_pid) {
	//autonDrive(-10, DL_pid, DR_pid, false);
	unsigned long t0 = millis();
	int step = 0;
	while(true) {
		autonDrive(10, DLturn_pid, DRturn_pid, true);
		printEnc_pidDrive();
	}
	while(digitalRead(MGL_LIM)) {
		setMGL(-127);
	}
	resetMotors();
	resetDrive();
	t0 = millis();
	while (true) {
		if(!joystickGetDigital(1, 6, JOY_DOWN)) {
			resetMotors();
			resetDrive();
			t0 = millis();
			continue;
		}
		switch(step) {
			case 0:
				pidArm(arm_pid, 69);
				pidCB(cb_pid, 130);
				if(millis()-t0 < 2500) {
					setMGL(127);
				} else {
					setMGL(0);
					t0 = millis();
					step++;
				}
				break;
			case 1:
				step += autonDrive(-45, DL_pid, DR_pid, false);
				if(step == 2) {
					resetDrive();
				}
				if(millis()-t0 < 500) {
					setMGL(127);
				} else {
					setMGL(0);
				}
				setClaw(-20);
				break;
			case 2:
				step += autonDrive(10, DLturn_pid, DRturn_pid, true);
				if(step == 3) {
					resetDrive();
				}
			case 3:
				//then raise mobile goal lift
				if(digitalRead(MGL_LIM)) {
					setMGL(-127);
				} else {
					setMGL(0);
					step++;
				}
				break;/*
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
		printf("arm: %d/%d\tcb: %d/%d\tDL: %d/%d\tDR: %d/%d\tDLt: %d/%d\tDRt: %d/%d\n", (int)arm_pid->sensVal, (int)arm_pid->target, (int)cb_pid->sensVal, (int)cb_pid->target, (int)DL_pid->sensVal, (int)DL_pid->target, (int)DR_pid->sensVal, (int)DR_pid->target, (int)DLturn_pid->sensVal, (int)DLturn_pid->target, (int)DRturn_pid->sensVal, (int)DRturn_pid->target);
		delay(20);
	}
}
void operatorControl() {
	bool clawOpen = true;
	printf("test123");
	auton1(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid, &arm_pid, &cb_pid);
	while (true) {
		printEnc();
		updateLift(&arm_pid, &cb_pid);
		//----- mobile-goal lift -----//
		if(joystickGetDigital(1, 8, JOY_UP)) {
			if(digitalRead(MGL_LIM)) {
				setMGL(-127);
			} else {
				setMGL(0);
			}
		} else if(joystickGetDigital(1, 8, JOY_DOWN)) {
			setMGL(127);
		} else {
			setMGL(0);
		}
		//----- claw -----//
		if(joystickGetDigital(1, 6, JOY_DOWN)) {
			//close
			clawOpen = false;
			setClaw(-60);
		}	else if(joystickGetDigital(1, 6, JOY_UP)) {
			//open
			clawOpen = true;
			setClaw(60);
		} else {
			if(clawOpen == true) {
				setClaw(0);//+
			} else {
				setClaw(0);//-
			}
		}
		//----- drive -----//
		const int td = 15;
		int j3 = joystickGetAnalog(1, 3);
		int j4 = joystickGetAnalog(1, 4);
		if((abs(j3) > td) || (abs(j4) > td)) {
			setDL(j3 + j4);
			setDR(j3 - j4);
		} else {
			setDL(0);
			setDR(0);
		}

		delay(20);
	}
}
