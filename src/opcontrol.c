/** @file opcontrol.c
 * @brief File for operator control code
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"
#include "pid.h"
#include "encoders.h"
#include "math.h"

const int MAX_POWER = 127;
bool wtf = false;

void limMotorVal(int* n) {
	if(*n > MAX_POWER) *n = MAX_POWER;
	if(*n < -MAX_POWER) *n = -MAX_POWER;
}
int getLimMotorVal(int n) {
	if(n > MAX_POWER) return MAX_POWER;
	if(n < -MAX_POWER) return -MAX_POWER;
	else return n;
}
void setDL(int n) {
	limMotorVal(&n);
	motorSet(9, -n);
	motorSet(8, -n);
}
void setDR(int n) {
	limMotorVal(&n);
	motorSet(7, n);
	motorSet(6, n);
}
void setArm(int n) {
	limMotorVal(&n);
	motorSet(4, -n);
	motorSet(5, n);
}
void setCB(int n) {
	limMotorVal(&n);
	motorSet(3, n);
}
void setClaw(int n) {
	limMotorVal(&n);
	motorSet(1, n);
}
void setMGL(int n) {
	limMotorVal(&n);
	motorSet(2, n);
}
//----- Chain-Bar -----//
void updateManualCB() {
	if(joystickGetDigital(1, 5, JOY_UP)) {
		setCB(-80);
	} else if(joystickGetDigital(1, 5, JOY_DOWN)) {
		setCB(80);
	} else {
		//rubber bands cancel out gravity
		setCB(0);
	}
}
//----- updates Arm then Chain-Bar-----//
bool cbManualMode = true;
void updateArm(PidVars cb_pid) {
	//----- update toggle -----//
	if(joystickGetDigital(1, 7, JOY_UP)) {
		cbManualMode = true;
	} else if(joystickGetDigital(1, 7, JOY_DOWN)) {
		cbManualMode = false;
	}
	//----- manual arm -----//
	const int t = 15;
	int j2 = joystickGetAnalog(1, 2);
	if(abs(j2) > t) {
		setArm(j2);
	} else {
		//rubber bands cancel out gravity for the arm
		setArm(0);
	}
	if(!cbManualMode) {
		//----- proportional + integral control: CB angle -> arm angle -----//
		cb_pid.target = eArmGet();
		cb_pid.sensVal = eChainGet();
		setCB(getLimMotorVal(updatePID(cb_pid)));
	} else {
		updateManualCB();
	}
}
void printEnc() {
	printf("Arm Encoder: %d\tChain-Bar Encoder: %d\tmanual enabled: %d\t wtf: %d\n", eArmGet(), eChainGet(), cbManualMode, wtf);
}
void operatorControl() {
	//----- arm and chain-bar setup-----//
	PidVars cb_pid = pidDef;
	cb_pid.kp = 1.0;//1.75
	cb_pid.ki = 0.0;//0.0025
	cb_pid.kd = 0.0;//18.0
	cb_pid.INTEGRAL_ACTIVE_ZONE = 30;
	cb_pid.maxIntegral = 50;

	while (true) {
		updateArm(cb_pid);
		printEnc();

		//----- mobile-goal lift -----//
		if(joystickGetDigital(1, 8, JOY_UP)) {
			setMGL(-127);
		} else if(joystickGetDigital(1, 8, JOY_DOWN)) {
			setMGL(127);
		} else {
			setMGL(0);
		}
		//----- claw -----//
		if(joystickGetDigital(1, 6, JOY_UP)) {
			//open
			setClaw(-60);
		}	else if(joystickGetDigital(1, 6, JOY_DOWN)) {
			//close
			setClaw(60);
		} else {
			//rubber bands cancel out gravity
			setClaw(0);
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
