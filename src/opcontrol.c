#include "main.h"
#include "pid.h"
#include "encoders.h"
#include <math.h>
#include "setup.h"

bool wtf = false;
//----- updates Arm and Chain-Bar -----//
void updateLift(PidVars* arm_pid, PidVars* cb_pid) {
	if(joystickGetDigital(1, 7, JOY_RIGHT)) {
		returnLift(arm_pid, cb_pid);
		return;
	}
	if(joystickGetDigital(1, 7, JOY_UP)) {
		// insert auto stack code here..............
		//c .........................................
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
void printEnc_pidDrive(PidVars* DL_pid, PidVars* DR_pid, PidVars* DLturn_pid, PidVars* DRturn_pid) {
	printf("DL: %d/%d\tDR: %d/%d\tDLt: %d/%d\tDRt: %d/%d\tt: %ld\tdnR: %ld\tdnL: %ld\tdnRt: %ld\tdnLt: %ld\n", (int)DL_pid->sensVal, (int)DL_pid->target, (int)DR_pid->sensVal, (int)DR_pid->target, (int)DLturn_pid->sensVal, (int)DLturn_pid->target, (int)DRturn_pid->sensVal, (int)DRturn_pid->target, millis(), DL_pid->doneTime, DR_pid->doneTime, DLturn_pid->doneTime, DRturn_pid->doneTime);
}
void printEnc_pidArmCB(PidVars* arm_pid, PidVars* cb_pid) {
	printf("arm: %d/%d\tcb: %d/%d\t", (int)arm_pid->sensVal, (int)arm_pid->target, (int)cb_pid->sensVal, (int)cb_pid->target);
}

void operatorControl() {
	bool clawOpen = true;
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
