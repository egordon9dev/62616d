#include "setup.h"
#include "API.h"
#include "pid.h"

//////////////////////////////          MOTORS
const int MAX_POWER = 127;
bool progSkills = false;
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
    if (drfbA > DRFB_MAX_CUT && n > 0) n = 0;
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
    if (fbA < FB_MIN_CUT && n < 0) n = 0;
    n = updateSlew(&fb_slew, n);
    motorSet(M10, n);
}
void setRollers(int n) {  //	set rollers
    limMotorVal(&n);
    motorSet(M11, n);
}
void setMGL(int n) {  //	set mobile goal lift
    limMotorVal(&n);
    int maxD = 15, maxU = 20;
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
Gyro gyro;
Encoder eDL, eDR;
void setupSens() {
    gyro = gyroInit(GYRO, 200);
    eDL = encoderInit(DRIVE_L_ENC_TOP, DRIVE_L_ENC_BOT, false);
    eDR = encoderInit(DRIVE_R_ENC_TOP, DRIVE_R_ENC_BOT, false);
    encoderReset(eDL);
    encoderReset(eDR);
}
int yawGet() { return -gyroGet(gyro); }
#define POT_SENSITIVITY 0.06105006105
int drfbGet() {  //-s
    return (-analogRead(DRFB_POT) + 2750) * POT_SENSITIVITY;
}
int fbGet() { return (analogRead(FB_POT) - 1320) * POT_SENSITIVITY; }
int mglGet() { return (4095 - analogRead(MGL_POT)) * POT_SENSITIVITY; }
int eDLGet() { return encoderGet(eDL); }
int eDRGet() { return encoderGet(eDR); }
void resetDriveEnc() {
    encoderReset(eDL);
    encoderReset(eDR);
}
void resetDrive() {
    resetDriveEnc();
    DL_pid.doneTime = LONG_MAX;
    DR_pid.doneTime = LONG_MAX;
    turn_pid.doneTime = LONG_MAX;
    setDL(0);
    setDR(0);
}
void resetMGL() {
    mgl_pid.doneTime = LONG_MAX;
    setMGL(0);
}
void resetFB(bool auton) {
    if (auton) {
        fb_pid_auto.doneTime = LONG_MAX;
    } else {
    }
    setFB(0);
}
void resetDRFB(bool auton) {
    if (auton) {
        drfb_pid_auto.doneTime = LONG_MAX;
    } else {
        drfb_pid.doneTime = LONG_MAX;
    }
    setDRFB(0);
}

void printEnc() { printf("dr4b: %d\tfb: %d\tmgl: %d\tDL: %d\tDR: %d\tyaw: %d\n", drfbGet(), fbGet(), mglGet(), eDLGet(), eDRGet(), yawGet()); }
void printEnc_pidDrive() { printf("DL: %d/%d\tDR: %d/%d\tTurn: %d/%d\tt: %ld\tdnL: %ld\tdnR: %ld\tdnT: %ld\n", (int)DL_pid.sensVal, (int)DL_pid.target, (int)DR_pid.sensVal, (int)DR_pid.target, (int)turn_pid.sensVal, (int)turn_pid.target, millis(), DL_pid.doneTime, DR_pid.doneTime, turn_pid.doneTime); }
void printEnc_pidDRFBFB() { printf("drfb: %d/%d\tfb: %d/%d\n", (int)drfb_pid_auto.sensVal, (int)drfb_pid_auto.target, (int)fb_pid_auto.sensVal, (int)fb_pid_auto.target); }
void printEnc_all() {
    printf("DL: %d/%d\tDR: %d/%d\tTurn: %d/%d\tt: %ld\t", (int)DL_pid.sensVal, (int)DL_pid.target, (int)DR_pid.sensVal, (int)DR_pid.target, (int)turn_pid.sensVal, (int)turn_pid.target, millis());
    printf("drfb: %d/%d\tfb: %d/%d\t", (int)drfb_pid_auto.sensVal, (int)drfb_pid_auto.target, (int)fb_pid_auto.sensVal, (int)fb_pid_auto.target);
    printf("mgl: %d/%d\n", (int)mgl_pid.sensVal, (int)mgl_pid.target);
}
int autonMode = 0;
void autoSelect() {
    static int prevBtn = 0;
    int btn = lcdReadButtons(LCD);
    if (btn & LCD_BTN_CENTER) {
        lcdSetText(LCD, 1, "BATTERY STUFF");
    } else {
        if (btn & LCD_BTN_LEFT && !(prevBtn & LCD_BTN_LEFT)) {
            if (autonMode > 0) { autonMode--; }
        } else if (btn & LCD_BTN_RIGHT && !(prevBtn & LCD_BTN_RIGHT)) {
            if (autonMode < nAutons + nSkills) { autonMode++; }
        }
        lcdClear(LCD);
        if (autonMode < nAutons) {
            lcdPrint(LCD, 1, "<   Auton  %d   >", autonMode);
            switch (autonMode) {
                case 0: lcdSetText(LCD, 2, "MG + C 20pt LEFT"); break;
                case 1: lcdSetText(LCD, 2, "MG + C 20pt RIGHT"); break;
                case 2: lcdSetText(LCD, 2, "MG + C 10pt LEFT"); break;
                case 3: lcdSetText(LCD, 2, "MG + C 10pt RIGHT"); break;
                case 4: lcdSetText(LCD, 2, "MG + C 5pt ANY"); break;
            }
        } else if (autonMode <= nAutons + nSkills) {
            switch (autonMode) {
                case 5:
                    lcdSetText(LCD, 1, "Prog Skills");
                    lcdSetText(LCD, 2, "MGs 5.5in L side");
                    break;
                case 6:
                    lcdSetText(LCD, 1, "Driver Skills");
                    lcdSetText(LCD, 2, "MG+C LEFT,OP= 8D");
                    break;
            }
        } else {
            lcdSetText(LCD, 1, "--- INVALID ---");
            lcdSetText(LCD, 2, "--- INVALID ---");
        }
    }
    prevBtn = btn;
}
