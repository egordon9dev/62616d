#include "main.h"
#include "pid.h"
#include "encoders.h"
#include <math.h>
#include "ports.h"

const int MAX_POWER = 127;
bool wtf = false;

//angle settings for autonomous cone stacking
int armAngle[] = {
	57,//	0
	57,//	1
	57,//	2
};
int cbAngle[] = {
	90,//	0
	100,//	1
	110,//	2
};
//lift return angles for picking up cones
int cbRet = -110;
int armRet = 57;

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
void setArm(int n) {//	set main 4 bar lift
	limMotorVal(&n);
	motorSet(M9_10, n);
}
void setCB(int n) {//	set chain bar lift
	int max = 40;
	int zone = 60;
	limMotorVal(&n);
	if(eCBGet() > CB_MAX - zone && n < -max) {
		n = -max;
	} else if(eCBGet() < CB_MIN + zone && n > max) {
		n = max;
	}
	motorSet(M7_8, -n);
}
void setClaw(int n) {//	set claw
	limMotorVal(&n);
	motorSet(M6, n);
}
void setMGL(int n) {//	set mobile goal lift
	limMotorVal(&n);
	motorSet(M11, -n);
}
//set chain bar and arm with PID to stack given cone
void stack(PidVars* arm_pid, PidVars* cb_pid, int cone) {
	cb_pid->target = cbAngle[cone];
	cb_pid->sensVal = eCBGet();
	arm_pid->target = armAngle[cone];
	arm_pid->sensVal = eArmGet();
	setCB(updatePID(cb_pid));
	setArm(updatePID(arm_pid));
}
//return lift to pick up cones
void returnLift(PidVars* arm_pid, PidVars* cb_pid) {
	cb_pid->target = cbRet;
	cb_pid->sensVal = eCBGet();
	arm_pid->target = armRet;
	arm_pid->sensVal = eArmGet();
	setCB(updatePID(cb_pid));
	setArm(updatePID(arm_pid));
}
//----- updates Arm then Chain-Bar -----//
void updateLift(PidVars* cb_pid, PidVars* arm_pid) {
	//----- update arm -----//
	const int t = 15;
	int j2 = -joystickGetAnalog(1, 2);
	if(abs(j2) > t) {
		setArm(j2);
	} else {
		setArm(0);
	}
	//------ update chain bar -----//
	if(joystickGetDigital(1, 5, JOY_UP)) {
		setCB(127);
	} else if(joystickGetDigital(1, 5, JOY_DOWN)) {
		setCB(-127);
	} else {
		setCB(0);
	}
}
int stackH = 0;//	cone stack height
bool stacking = false, returning = false;
void updateStack(PidVars* arm_pid, PidVars* cb_pid) {
	if(joystickGetDigital(1, 7, JOY_UP)) {
		returning = false;
		stacking = true;
		if(stackH < sizeof(armAngle)/sizeof(armAngle[0]) - 1) {
			stackH++;
		} else {
			wtf = true;
			printf("error: ya dun stacked too high");
		}
	}
	if(joystickGetDigital(1, 7, JOY_RIGHT)) {
		returning = true;
		stacking = false;
	}

	if(stacking) {
		stack(arm_pid, cb_pid, stackH);
	} else if(returning) {
		returnLift(arm_pid, cb_pid); //return to intake position (low)
	} else {
		updateLift(cb_pid, arm_pid); //use manual lift control
	}
	if(joystickGetDigital(1, 7, JOY_LEFT)) { // enable manual lift control
		if(joystickGetDigital(1, 7, JOY_DOWN)) { // reset stack
			stackH = 0;
		}
		returning = false;
		stacking = false;
	}
}
void printEnc() {
	printf("Arm: %d\tCB: %d\tDL: %d\tDR: %d\n", eArmGet(), eCBGet(), eDLGet(), eDRGet());
}/*
void operatorControl() {
	while(true) {
		setMGL(50);
		delay(20);
	}
}*/
void operatorControl() {
	//----- arm and chain-bar setup-----//
	PidVars cb_pid = pidDef;
	cb_pid.kp = 2.475;//1.75
	cb_pid.ki = 0.00;//0.0025
	cb_pid.kd = 300.0;//18.0
	cb_pid.INTEGRAL_ACTIVE_ZONE = 20;
	cb_pid.maxIntegral = 50;
	cb_pid.prevTime = millis();
	PidVars arm_pid = pidDef;
	arm_pid.kp = 1.5;//1.75
	arm_pid.ki = 0.00;//0.0025
	arm_pid.kd = 300.0;//18.0
	arm_pid.INTEGRAL_ACTIVE_ZONE = 20;
	arm_pid.maxIntegral = 50;
	arm_pid.prevTime = millis();

	bool clawOpen = true;
	while (true) {
		printEnc();
		updateLift(&cb_pid, &arm_pid);
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
		if(joystickGetDigital(1, 6, JOY_UP)) {
			//open
			clawOpen = true;
			setClaw(-60);
		}	else if(joystickGetDigital(1, 6, JOY_DOWN)) {
			//close
			clawOpen = false;
			setClaw(60);
		} else {
			//rubber bands cancel out gravity
			if(clawOpen == true) {
				setClaw(-15);
			} else {
				setClaw(35);
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
