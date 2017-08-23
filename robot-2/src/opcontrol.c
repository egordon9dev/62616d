/** @file opcontrol.c
 * @brief File for operator control code
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"
#include "pid.h"
#include "encoders.h"
#include <math.h>

const int MAX_POWER = 127;

void limMotorVal(int* n) {
	if(*n > MAX_POWER) *n = MAX_POWER;
	if(*n < -MAX_POWER) *n = -MAX_POWER;
}
int getLimMotorVal(int n) {
	if(n > MAX_POWER) return MAX_POWER;
	if(n < -MAX_POWER) return -MAX_POWER;
	else return n;
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
void setClaw(bool b) {
	digitalWrite(2, b);
}
void setMGL(int n) {
	limMotorVal(&n);
	motorSet(6, n);
}
//----- Chain-Bar -----//
void updateManualCB(int grav_out) {
	if(joystickGetDigital(1, 5, JOY_UP)) {
		setCB(127);
	} else if(joystickGetDigital(1, 5, JOY_DOWN)) {
		setCB(-127);
	} else {
		setCB(grav_out);
	}
}
//----- updates Arm then Chain-Bar-----//
bool cbManualMode = true;
void updateArm(PidVars cb_pid, PidVars grav_pid) {
	//----- update toggle -----//
	if(joystickGetDigital(1, 6, JOY_UP)) {
		cbManualMode = true;
	} else if(joystickGetDigital(1, 6, JOY_DOWN)) {
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
	//----- proportional control: cancel out gravity for the Chain-Bar -----//
	grav_pid.sensVal = eChainGet();
	double grav_out = sin( (grav_pid.sensVal/360.0) * 2.0*M_PI ) * grav_pid.kp;
	if(!cbManualMode) {
		//----- proportional + integral control: CB angle -> arm angle -----//
		cb_pid.target = eArmGet();
		cb_pid.sensVal = eChainGet();
		setCB(getLimMotorVal(updatePI(cb_pid)) + grav_out);
	} else {
		updateManualCB(grav_out);
	}
}
void printEnc() {
	printf("Arm Encoder: %d\tChain-Bar Encoder: %d\tpid enabled: %d\n", eArmGet(), eChainGet(), cbManualMode);
}
void operatorControl() {
	//----- arm and chain-bar setup-----//
	PidVars cb_pid = pidDef;
	cb_pid.kp = 1.75;
	cb_pid.ki = 0.0025;
	cb_pid.kd = 20.0;
	cb_pid.INTEGRAL_ACTIVE_ZONE = 30;
	cb_pid.maxIntegral = 50;

	PidVars grav_pid = pidDef;
	grav_pid.kp = 22.0;
	grav_pid.target = 180;

	while (true) {
		updateArm(cb_pid, grav_pid);
		printEnc();

		//----- mobile-goal lift -----//
		if(joystickGetDigital(1, 7, JOY_UP)) {
			setMGL(-127);
		} else if(joystickGetDigital(1, 7, JOY_DOWN)) {
			setMGL(127);
		} else {
			setMGL(0);
		}
		//----- claw -----//
		if(joystickGetDigital(1, 8, JOY_UP)) {
			setClaw(HIGH);
		} else if(joystickGetDigital(1, 8, JOY_DOWN)) {
			setClaw(LOW);
		}

		delay(20);
	}
}
