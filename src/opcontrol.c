#include "main.h"
#include "pid.h"
#include "setup.h"

//----- updates Arm and Chain-Bar -----//
void updateLift(PidVars* arm_pid, PidVars* cb_pid) {
	//----- update arm -----//
	if(!digitalRead(MGL_LIM)) {
		if(joystickGetDigital(1, 7, JOY_RIGHT)) {
			returnLift(arm_pid, cb_pid);
			return;
		}
		if(joystickGetDigital(1, 7, JOY_UP)) {
			// insert auto stack code here..............
			//c .........................................
			return;
		}
		const int t = 15;
		int j2 = joystickGetAnalog(1, 2);
		static unsigned long tLiftOff = 0;
		static double liftHoldAngle = 90;
		static bool pidRunning = false;
		if(abs(j2) > t) {
			tLiftOff = millis();
			setArm(j2);
		} else {
			if(millis() - tLiftOff > 300) {
				if(!pidRunning) {
					liftHoldAngle = armGet();
					pidRunning = true;
				}
				arm_pid->sensVal = armGet();
				arm_pid->target = liftHoldAngle;
				setArm(updatePID(arm_pid));
			} else {
				setArm(0);
				pidRunning = false;
			}
		}
		//------ update chain bar -----//
		if(joystickGetDigital(1, 5, JOY_UP)) {
			setCB(-127);
		} else if(joystickGetDigital(1, 5, JOY_DOWN)) {
			setCB(127);
		} else {
			setCB(0);
		}
	} else {
		pidArm(arm_pid, 72);
		pidCB(cb_pid, 150);
	}
}
void pidTest() {
	while(true) {
		pidDrive(90, &DLturn_pid, &DRturn_pid, true);
		printEnc_pidDrive(&DL_pid, &DR_pid, &DLturn_pid, &DRturn_pid);
		delay(20);
	}
}
void operatorControl() {
	bool clawOpen = false;
	long tClawOpen = millis();
	bool mglClose = false;
	bool mglUp = false;
	while(true) {
		printEnc();
		updateLift(&arm_pid, &cb_pid);
		//----- mobile-goal lift -----//
		if(joystickGetDigital(1, 8, JOY_RIGHT)) {
			if((millis()/200) % 2 == 1) {
			    setMGL(127);
			} else {
			    setMGL(-127);
			}
		} else if(joystickGetDigital(1, 8, JOY_UP)) {
			if(digitalRead(MGL_LIM)) {
				setMGL(-127);
				mglClose = false;
				mglUp = false;//true
			} else {
				setMGL(-20);
				mglClose = true;
				mglUp = false;
			}
		} else if(joystickGetDigital(1, 8, JOY_DOWN)) {
			setMGL(127);
			mglClose = false;
			mglUp = false;
		} else {
			if(mglClose) {
				setMGL(-25);// hold mobile goal in place
			} else if(mglUp) {
				setMGL(-127); // automatically continue lifting mobile goal
			} else {
				setMGL(0);
			}
		}
		//----- claw -----//
		if(joystickGetDigital(1, 6, JOY_DOWN)) {
			//close
			clawOpen = false;
			setClaw(-70);
		} else if(joystickGetDigital(1, 6, JOY_UP)) {
			//open
			clawOpen = true;
			tClawOpen = millis();
			setClaw(70);
		} else {
			if(clawOpen == true) {
				if(millis() - tClawOpen < 300) {
					setClaw(10);//+
				} else {
					setClaw(0);//+
				}
			} else {
				setClaw(-15);//-
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
