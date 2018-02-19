#include "setup.h"
#include "API.h"
#include "pid.h"

double fbUpP = 121;

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
int limInt(int n, int min, int max) {
    if (n > max) return max;
    if (n < min) return min;
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
    motorSet(M8_9, -n);
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
    printf("fb: %d", n);
}

void setRollers(int n) {  //	set rollers
    limMotorVal(&n);
    // n = updateSlew(&roller_slew, n);
    motorSet(M11, n);
}
void setMGL(int n) {  //	set mobile goal lift
    limMotorVal(&n);
    int maxD = 20, maxU = 25;
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
PidVars DL_brake, DR_brake;
void setupSens() {
    DL_brake = pidDef;
    DR_brake = pidDef;
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
void printDrv() {
    printf("DL: %d/%d\tDR: %d/%d\tDLt: %d/%d\tDRt: %d/%d\tt: %ld\t", (int)DL_pid.sensVal, (int)DL_pid.target, (int)DR_pid.sensVal, (int)DR_pid.target, (int)DLturn_pid.sensVal, (int)DLturn_pid.target, (int)DRturn_pid.sensVal, (int)DRturn_pid.target, millis());
    printf("driveCurve: %d/%d\tturnCurve: %d/%d\t", (int)driveCurve_pid.sensVal, (int)driveCurve_pid.target, (int)turnCurve_pid.sensVal, (int)turnCurve_pid.target);
}
void printEnc_pidDrive() {
    printDrv();
    printf("\n");
}
void printDRFBFB() { printf("drfb: %d/%d\tfb: %d/%d\t", (int)drfb_pid_auto.sensVal, (int)drfb_pid_auto.target, (int)fb_pid_auto.sensVal, (int)fb_pid_auto.target); }
void printEnc_pidDRFBFB() {
    printDRFBFB();
    printf("\n");
}
void printEnc_all() {
    printDrv();
    printDRFBFB();
    printf("\n");
}
int autonMode = 0, autonModeLen = 6;
void autoSelect() {
    static int prevBtn = 0;
    int btn = lcdReadButtons(LCD);
    if (btn & LCD_BTN_CENTER) {
        lcdSetText(LCD, 1, "Hi");
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
            lcdSetText(LCD, 2, "110");
        } else if (autonMode == i++) {
            lcdPrint(LCD, 1, "auto:  %d", autonMode);
            lcdSetText(LCD, 2, "120");
        } else if (autonMode == i++) {
            lcdPrint(LCD, 1, "auto:  %d", autonMode);
            lcdSetText(LCD, 2, "130");
        } else if (autonMode == i++) {
            lcdPrint(LCD, 1, "auto:  %d", autonMode);
            lcdSetText(LCD, 2, "010");
        } else if (autonMode == i++) {
            lcdPrint(LCD, 1, "auto skills: %d", autonMode);
            lcdSetText(LCD, 2, "020");
        } else if (autonMode == i++) {
            lcdPrint(LCD, 1, "auto skills: %d", autonMode);
            lcdSetText(LCD, 2, "030");
        } else {
            lcdSetText(LCD, 1, "INVALID");
            lcdSetText(LCD, 2, "INVALID");
        }
    }
    prevBtn = btn;
}

unsigned long dt = 0, prevT = 0;
int DL_brake_out = 0, DR_brake_out = 0;
void opctrlDrive() {
    DL_brake.kd = 20;
    DR_brake.kd = 20;

    const int td = 15;
    int drv = joystickGetAnalog(1, 3);
    int trn = joystickGetAnalog(1, 1) * DRIVE_TURN_MAX / 127.0;
    if (drv > DRIVE_DRIVE_MAX) drv = DRIVE_DRIVE_MAX;
    if (drv < -DRIVE_DRIVE_MAX) drv = -DRIVE_DRIVE_MAX;
    if (abs(drv) < td) drv = 0;
    if (abs(trn) < td) trn = 0;
    double lvel = 0.0, rvel = 0.0;
    if (dt != 0) {
        lvel = (DL_brake.prevSensVal - eDLGet()) / (double)dt;
        rvel = (DR_brake.prevSensVal - eDRGet()) / (double)dt;
    }
    DL_brake_out = lvel * DL_brake.kd;
    DR_brake_out = rvel * DR_brake.kd;
    setDL(drv + trn);
    setDR(drv - trn);
    if (drv == 0 && trn == 0) {
        int brakeMax = 100;
        if (DL_brake_out > brakeMax) DL_brake_out = brakeMax;
        if (DL_brake_out < -brakeMax) DL_brake_out = -brakeMax;
        if (DR_brake_out > brakeMax) DR_brake_out = brakeMax;
        if (DR_brake_out < -brakeMax) DR_brake_out = -brakeMax;

        setDL(DL_brake_out);
        setDR(DR_brake_out);
    }
    DL_brake.prevSensVal = eDLGet();
    DR_brake.prevSensVal = eDRGet();
    dt = millis() - prevT;
    if (dt > 300) dt = 20;
    prevT = millis();
}
