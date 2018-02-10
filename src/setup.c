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
void setDL(int n) {  //	set right drive motors
    limMotorVal(&n);
    n = updateSlew(&DL_slew, n);
    motorSet(M3, n);
    motorSet(M4_5, n);
}
void setDR(int n) {  //	set left drive motors
    limMotorVal(&n);
    n = updateSlew(&DR_slew, n);
    motorSet(M0, -n);
    motorSet(M1_2, -n);
}
void setDRFB(int n) {  //	set main 4 bar lift
    limMotorVal(&n);
    int max = 50;
    int drfbA = drfbGet();
    if (drfbA > DRFB_MAX_HOLD_ANGLE && n > 0) n = 0;
    if ((drfbA < DRFB_MIN && n < 0) || (drfbA > DRFB_MAX1 && n > 0)) {
        if (drfbA < DRFB_MIN / 3 || drfbA > DRFB_MAX2) max /= 3;
        if (n > max) n = max;
        if (n < -max) n = -max;
    }
    n = updateSlew(&drfb_slew, n);
    motorSet(M8_9, n);
}

void setFB(int n) {
    limMotorVal(&n);
    int max = 20;
    int fbA = fbGet();
    if (fbA > FB_MAX && n > max) n = max;
    if (fbA < FB_MIN && n < -max) n = -max;
    if (fbA < FB_MIN_HOLD_ANGLE && n < 0) n = 0;
    n = updateSlew(&fb_slew, n);
    motorSet(M10, n);
}
void setRollers(int n) {  //	set rollers
    limMotorVal(&n);
    n = updateSlew(&roller_slew, n);
    motorSet(M11, n);
}
void setMGL(int n) {  //	set mobile goal lift
    limMotorVal(&n);
    int maxD = 20, maxU = 20;
    if (mglGet() > MGL_MAX) {
        if (n >= 0) n = maxD;
    }
    if (mglGet() < MGL_MIN) {
        if (n <= 0) n = -maxU;
    }
    n = updateSlew(&mgl_slew, n);
    motorSet(M6_7, n);
}
void resetMotors() {
    for (int i = 1; i <= 10; i++) { motorSet(i, 0); }
}
void setupLCD() {
    lcdInit(LCD);
    lcdClear(LCD);
    lcdSetBacklight(LCD, true);
}
Encoder eDL, eDR;
void resetDriveEnc() {
    encoderReset(eDL);
    encoderReset(eDR);
}
void setupSens() {
    eDL = encoderInit(DRIVE_L_ENC_TOP, DRIVE_L_ENC_BOT, false);
    eDR = encoderInit(DRIVE_R_ENC_TOP, DRIVE_R_ENC_BOT, false);
    encoderReset(eDL);
    encoderReset(eDR);
}
#define POT_SENSITIVITY 0.06105006105
int drfbGet() { return (-analogRead(DRFB_POT) + 2750) * POT_SENSITIVITY; }
int fbGet() { return (1840 - analogRead(FB_POT)) * POT_SENSITIVITY + 40; }
int mglGet() { return (4095 - analogRead(MGL_POT)) * POT_SENSITIVITY; }
int eDLGet() { return encoderGet(eDL); }
int eDRGet() { return encoderGet(eDR); }

void printEnc() { printf("dr4b: %d\tfb: %d\tmgl: %d\tDL: %d\tDR: %d\t\n", drfbGet(), fbGet(), mglGet(), eDLGet(), eDRGet()); }
void printEnc_pidDrive() {
    printf("DL: %d/%d\tDR: %d/%d\tDLt: %d/%d\tDRt: %d/%d\tt: %ld\t", (int)DL_pid.sensVal, (int)DL_pid.target, (int)DR_pid.sensVal, (int)DR_pid.target, (int)DLturn_pid.sensVal, (int)DLturn_pid.target, (int)DRturn_pid.sensVal, (int)DRturn_pid.target, millis());
    printf("driveCurve: %d/%d\tturnCurve: %d/%d\n", (int)driveCurve_pid.sensVal, (int)driveCurve_pid.target, (int)turnCurve_pid.sensVal, (int)turnCurve_pid.target);
}
void printEnc_pidDRFBFB() { printf("drfb: %d/%d\tfb: %d/%d\n", (int)drfb_pid_auto.sensVal, (int)drfb_pid_auto.target, (int)fb_pid_auto.sensVal, (int)fb_pid_auto.target); }
void printEnc_all() {
    printf("DL: %d/%d\tDR: %d/%d\tDLt: %d/%d\tDRt: %d/%d\tt: %ld\t", (int)DL_pid.sensVal, (int)DL_pid.target, (int)DR_pid.sensVal, (int)DR_pid.target, (int)DLturn_pid.sensVal, (int)DLturn_pid.target, (int)DRturn_pid.sensVal, (int)DRturn_pid.target, millis());
    printf("drfb: %d/%d\tfb: %d/%d\t", (int)drfb_pid_auto.sensVal, (int)drfb_pid_auto.target, (int)fb_pid_auto.sensVal, (int)fb_pid_auto.target);
    printf("mgl: %d/%d\n", (int)mgl_pid.sensVal, (int)mgl_pid.target);
}
int autonMode = 0, autonModeLen = 6;
void autoSelect() {
    static int prevBtn = 0;
    int btn = lcdReadButtons(LCD);
    if (btn & LCD_BTN_CENTER) {
        lcdSetText(LCD, 1, "THA EGOR SAYS HI");
    } else {
        if (btn & LCD_BTN_LEFT && !(prevBtn & LCD_BTN_LEFT)) {
            if (autonMode > 0) { autonMode--; }
        } else if (btn & LCD_BTN_RIGHT && !(prevBtn & LCD_BTN_RIGHT)) {
            if (autonMode < autonModeLen - 1) { autonMode++; }
        }
        lcdClear(LCD);
        lcdPrint(LCD, 1, "<   Auton  %d   >", autonMode);
        int i = 0;
        if (autonMode == i++) {
            lcdPrint(LCD, 1, "auto:  %d", autonMode);
            lcdSetText(LCD, 2, "--- NONE ---");
        } else if (autonMode == i++) {
            lcdPrint(LCD, 1, "auto:  %d", autonMode);
            lcdSetText(LCD, 2, "MG+1C:20 LEFT");
        } else if (autonMode == i++) {
            lcdPrint(LCD, 1, "auto:  %d", autonMode);
            lcdSetText(LCD, 2, "MG+1C:20 RIGHT");
        } else if (autonMode == i++) {
            lcdPrint(LCD, 1, "auto:  %d", autonMode);
            lcdSetText(LCD, 2, "MG+2C:20 LEFT");
        } else if (autonMode == i++) {
            lcdPrint(LCD, 1, "auto:  %d", autonMode);
            lcdSetText(LCD, 2, "MG+2C:20 RIGHT");
        } else if (autonMode == i++) {
            lcdPrint(LCD, 1, "auto skills: %d", autonMode);
            lcdSetText(LCD, 2, "LEFT");
        } else {
            lcdSetText(LCD, 1, "INVALID");
            lcdSetText(LCD, 2, "INVALID");
        }
    }
    prevBtn = btn;
}
