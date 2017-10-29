#include "setup.h"
#include "API.h"
#include "pid.h"

//////////////////////////////          MOTORS
const int MAX_POWER = 127;
void limMotorVal(int* n) {
    if (*n > MAX_POWER) *n = MAX_POWER;
    if (*n < -MAX_POWER) *n = -MAX_POWER;
}
int getLimMotorVal(int n) {
    if (n > MAX_POWER) return MAX_POWER;
    if (n < -MAX_POWER) return -MAX_POWER;
    return n;
}
void setDL(int n) {  //	set left drive motors
    limMotorVal(&n);
    motorSet(M0, n);
    motorSet(M1_2, n);
}
void setDR(int n) {  //	set right drive motors
    limMotorVal(&n);
    motorSet(M3, -n);
    motorSet(M4_5, -n);
}
void setArm(int n) {  //	set main 4 bar lift
    limMotorVal(&n);
    motorSet(M9_10, -n);
}
void setCB(int n) {  //	set chain bar lift
    /*int maxPow = 0;
    int weakZone = 60;*/
    limMotorVal(&n); /*
     if(cbGet() > CB_MAX - weakZone && n > maxPow) {
             n = maxPow;
     } else if(cbGet() < CB_MIN + weakZone && n < -maxPow) {
             n = -maxPow;
     }*/
    motorSet(M7_8, n);
}
void setClaw(int n) {  //	set claw
    limMotorVal(&n);
    motorSet(M6, -n);
}
void setMGL(int n) {  //	set mobile goal lift
    limMotorVal(&n);
    int hold = -20;
    if (n > hold || digitalRead(MGL_LIM)) {
        motorSet(M11, n);
        return;
    }
    motorSet(M11, hold);
}
void resetMotors() {
    for (int i = 1; i <= 10; i++) {
        motorSet(i, 0);
    }
}

void setupLCD() {
    lcdInit(LCD);
    lcdClear(LCD);
    lcdSetBacklight(LCD, true);
}
//////////////////////////////          ENCODERS
static Encoder eDL, eDR;
void setupEnc() {
    eDL = encoderInit(DRIVE_L_ENC_TOP, DRIVE_L_ENC_BOT, false);
    eDR = encoderInit(DRIVE_R_ENC_TOP, DRIVE_R_ENC_BOT, false);
    encoderReset(eDL);
    encoderReset(eDR);
}
#define POT_SENSITIVITY 0.06105006105
double armGet() {  //-
    return (3955 - analogRead(ARML_POT) + analogRead(ARMR_POT) - 121) * (POT_SENSITIVITY / 2.0) + 69;
}
double cbGet() {
    return analogRead(CB_POT) * POT_SENSITIVITY + 107.5;  // 107.5
}
int eDLGet() { return encoderGet(eDL); }
int eDRGet() { return encoderGet(eDR); }
void resetDriveEnc() {
    encoderReset(eDL);
    encoderReset(eDR);
}
void resetDrive(PidVars* DL_pid, PidVars* DR_pid, PidVars* DLturn_pid, PidVars* DRturn_pid) {
    resetDriveEnc();
    DL_pid->doneTime = LONG_MAX;
    DR_pid->doneTime = LONG_MAX;
    DLturn_pid->doneTime = LONG_MAX;
    DRturn_pid->doneTime = LONG_MAX;
    setDL(0);
    setDR(0);
}

void printEnc() { printf("Arm: %lf\tCB: %lf\tDL: %d\tDR: %d\n", armGet(), cbGet(), eDLGet(), eDRGet()); }
void printEnc_pidDrive(PidVars* DL_pid, PidVars* DR_pid, PidVars* DLturn_pid, PidVars* DRturn_pid) {
    printf(
        "DL: %d/%d\tDR: %d/%d\tDLt: %d/%d\tDRt: %d/%d\tt: %ld\tdnR: "
        "%ld\tdnL: %ld\tdnRt: %ld\tdnLt: %ld\n",
        (int)DL_pid->sensVal, (int)DL_pid->target, (int)DR_pid->sensVal, (int)DR_pid->target, (int)DLturn_pid->sensVal, (int)DLturn_pid->target, (int)DRturn_pid->sensVal, (int)DRturn_pid->target, millis(), DL_pid->doneTime, DR_pid->doneTime, DLturn_pid->doneTime, DRturn_pid->doneTime);
}
void printEnc_pidArmCB(PidVars* arm_pid, PidVars* cb_pid) { printf("arm: %d/%d\tcb: %d/%d\n", (int)arm_pid->sensVal, (int)arm_pid->target, (int)cb_pid->sensVal, (int)cb_pid->target); }

int autonMode = 0;
#define AUTO_MAX 5
#define nAutons 4
void autoSelect() {
    static int prevBtn = 0;
    int btn = lcdReadButtons(LCD);
    if (btn & LCD_BTN_CENTER) {
        lcdSetText(LCD, 1, "BATTERY STUFF");
    } else {
        if (btn & LCD_BTN_LEFT && !(prevBtn & LCD_BTN_LEFT)) {
            if (autonMode > 0) {
                autonMode--;
            }
        } else if (btn & LCD_BTN_RIGHT && !(prevBtn & LCD_BTN_RIGHT)) {
            if (autonMode < AUTO_MAX) {
                autonMode++;
            }
        }
        lcdClear(LCD);
        if (autonMode < nAutons) {
            lcdPrint(LCD, 1, "<   Auton  %d   >", autonMode);
            switch (autonMode) {
                case 0:
                    lcdSetText(LCD, 2, "MG + C 20pt LEFT");
                    break;
                case 1:
                    lcdSetText(LCD, 2, "MG + C 20pt RIGHT");
                    break;
                case 2:
                    lcdSetText(LCD, 2, "MG + C 10pt LEFT");
                    break;
                case 3:
                    lcdSetText(LCD, 2, "MG + C 10pt RIGHT");
                    break;
            }
        } else {
            lcdPrint(LCD, 1, "<   Skills %d   >", autonMode - nAutons);
        }
    }
    prevBtn = btn;
}
