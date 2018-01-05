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
    if (progSkills) {
        n = updateSlew(&DL_slew_auto, n);
    } else {
        n = updateSlew(&DL_slew, n);
    }
    printf("DL: %d\t", n);
    motorSet(M3, n);
    motorSet(M4_5, n);
}
void setDR(int n) {  //	set left drive motors
    limMotorVal(&n);
    if (progSkills) {
        n = updateSlew(&DR_slew_auto, n);
    } else {
        n = updateSlew(&DR_slew, n);
    }
    printf("DR: %d\t", n);
    motorSet(M0, -n);
    motorSet(M1_2, -n);
}
void setDRFB(int n) {  //	set main 4 bar lift
    limMotorVal(&n);
    int max = 50;
    if ((drfbGet() > DRFB_MAX && n > 0) || (drfbGet() < DRFB_MIN && n < 0)) {
        if (drfbGet() < DRFB_MIN / 3) max /= 3;
        if (n > max) n = max;
        if (n < -max) n = -max;
    }
    if (drfbGet() > DRFB_MAX_CUT && n > 0) {
        n = 0;
    }
    n = updateSlew(&drfb_slew, n);
    motorSet(M8_9, n);
}

void setFB(int n) {
    limMotorVal(&n);
    int max = 15;
    if ((fbGet() > FB_MAX && n > 0) || (fbGet() < FB_MIN && n < 0)) {
        if (n > max) n = max;
        if (n < -max) n = -max;
    }
    if (fbGet() < FB_MIN_CUT - drfbGet() && n < 0) n = 0;
    if (fbGet() > FB_MAX_CUT + drfbGet() / 6 && n > 0) n = 0;
    n = updateSlew(&fb_slew, n);
    motorSet(M10, n);
}
void setRollers(int n) {  //	set rollers
    limMotorVal(&n);
    motorSet(M11, n);
}
void setMGL(int n) {  //	set mobile goal lift
    limMotorVal(&n);
    int max = 15;
    if ((mglGet() > MGL_MAX && n > 0) || (mglGet() < MGL_MIN && n < 0)) {
        if (n > max) n = max;
        if (n < -max) n = -max;
    }
    lcdPrint(LCD, 1, "mgl: %d\n", n);
    n = updateSlew(&mgl_slew, n);
    motorSet(M6_7, n);
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
int mglGet() { return (analogRead(MGL_POT) - 1500) * POT_SENSITIVITY; }
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
void resetFB() {
    fb_pid.doneTime = LONG_MAX;
    setFB(0);
}
void resetDRFB() {
    drfb_pid_auto.doneTime = LONG_MAX;
    setDRFB(0);
}

void printEnc() { printf("dr4b: %d\tfb: %d\tmgl: %d\tDL: %d\tDR: %d\tyaw: %d\n", drfbGet(), fbGet(), mglGet(), eDLGet(), eDRGet(), yawGet()); }
void printEnc_pidDrive() { printf("DL: %d/%d\tDR: %d/%d\tTurn: %d/%d\tt: %ld\tdnL: %ld\tdnR: %ld\tdnT: %ld\n", (int)DL_pid.sensVal, (int)DL_pid.target, (int)DR_pid.sensVal, (int)DR_pid.target, (int)turn_pid.sensVal, (int)turn_pid.target, millis(), DL_pid.doneTime, DR_pid.doneTime, turn_pid.doneTime); }
void printEnc_pidDRFBFB() { printf("drfb: %d/%d\tfb: %d/%d\n", (int)drfb_pid_auto.sensVal, (int)drfb_pid_auto.target, (int)fb_pid.sensVal, (int)fb_pid.target); }

int autonMode = 0;
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
            if (autonMode < nAutons + nSkills) {
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
                case 4:
                    lcdSetText(LCD, 2, "MG + C 5pt ANY");
                    break;
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
