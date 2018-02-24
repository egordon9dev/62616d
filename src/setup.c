#include "setup.h"
#include "API.h"
#include "pid.h"

double fbUpP = 121;

int DRIVE_DRIVE_MAX = 110, DRIVE_TURN_MAX = 90;

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
double limDouble(double n, double min, double max) {
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
    printf("(fb:%d)", n);
}

void setRollers(int n) {  //	set rollers
    limMotorVal(&n);
    motorSet(M11, n);
}
void setRollersSlew(int n) {
    limMotorVal(&n);
    n = updateSlew(&roller_slew, n);
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
    eDL = encoderInit(DRIVE_L_ENC_T, DRIVE_L_ENC_B, false);
    eDR = encoderInit(DRIVE_R_ENC_B, DRIVE_R_ENC_T, false);
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
    printf("DLR: %d/%d, %d/%d\tDLRt: %d/%d, %d/%d\tt: %ld\t", (int)DL_pid.sensVal, (int)DL_pid.target, (int)DR_pid.sensVal, (int)DR_pid.target, (int)DLturn_pid.sensVal, (int)DLturn_pid.target, (int)DRturn_pid.sensVal, (int)DRturn_pid.target, millis());
    printf("dCrv: %d/%d, tCrv: %d/%d\t", (int)driveCurve_pid.sensVal, (int)driveCurve_pid.target, (int)turnCurve_pid.sensVal, (int)turnCurve_pid.target);
}
void printEnc_pidDrive() {
    printDrv();
    printf("\n");
}
void printDRFBFB() { printf("drfb: %d/%d, fb: %d/%d\t", (int)drfb_pid_auto.sensVal, (int)drfb_pid_auto.target, (int)fb_pid_auto.sensVal, (int)fb_pid_auto.target); }
void printMGL() { printf("mgl: %d/%d\t", (int)mgl_pid.sensVal, (int)mgl_pid.target); }
void printEnc_pidDRFBFB() {
    printDRFBFB();
    printf("\n");
}
void printEnc_all() {
    printDrv();
    printMGL();
    printDRFBFB();
    printf("\n");
}
AutoSel autoSel = {.stackH = 1, .zone = 5, .nAuton = 0, .leftSide = true, .loaderSide = true};

void autoSelect() {
    static int prevBtn = 0;

    int btn = lcdReadButtons(LCD);
    if (btn == LCD_BTN_LEFT && !(prevBtn & LCD_BTN_LEFT)) {
        autoSel.leftSide = !autoSel.leftSide;
    } else if (btn == LCD_BTN_CENTER && !(prevBtn & LCD_BTN_CENTER)) {
        autoSel.stackH++;
        if (autoSel.nAuton == 1) {
            if (autoSel.stackH > 3) autoSel.stackH = 1;
        } else if (autoSel.nAuton == 2) {
            if (autoSel.zone == 20 && autoSel.stackH > 3) autoSel.stackH = 1;
            if (autoSel.zone == 10 && autoSel.stackH > 4) autoSel.stackH = 1;
            if (autoSel.zone == 5 && autoSel.stackH > 5) autoSel.stackH = 1;
        }
    } else if (btn == LCD_BTN_RIGHT && !(prevBtn & LCD_BTN_RIGHT)) {
        autoSel.loaderSide = !autoSel.loaderSide;
    }
    if (btn == (LCD_BTN_LEFT | LCD_BTN_CENTER) && prevBtn != (LCD_BTN_LEFT | LCD_BTN_CENTER)) {
        autoSel.nAuton++;
        if (autoSel.nAuton > 2) autoSel.nAuton = 0;
    }
    if (btn == (LCD_BTN_RIGHT | LCD_BTN_CENTER) && prevBtn != (LCD_BTN_RIGHT | LCD_BTN_CENTER)) {
        if (autoSel.zone == 5) {
            autoSel.zone = 10;
        } else if (autoSel.zone == 10) {
            autoSel.zone = 20;
        } else {
            autoSel.zone = 5;
        }
    }
    lcdClear(LCD);
    if (autoSel.nAuton == 0) {
        lcdPrint(LCD, 1, "AUTON SELECT");
        lcdSetText(LCD, 2, "--- NONE ---");
    } else {
        lcdPrint(LCD, 1, "A %d:%d %s", autoSel.nAuton, autoSel.zone, autoSel.nAuton == 1 ? "Field" : "Loader");
        lcdPrint(LCD, 2, " %s    C:%d    L:%s", autoSel.leftSide ? "L" : "R", autoSel.stackH, autoSel.loaderSide ? "Y" : "N");
    }
    prevBtn = btn;
}
int drfba[][2] = {{15, 4}, {18, 13}, {27, 22}, {37, 29}, {44, 36}, {52, 44}, {59, 52}, {67, 58}, {76, 67}, {85, 76}, {93, 85}, {103, 94}, {119, 108}};
int drfbDownA[] = {0, 0, 15, 21, 31, 40, 47, 55, 61, 69, 77, 84, 96};
int ldrGrabI = 0, ldrStackI = 0;
/* PRECONDITIONS:
    DRFB+FB just stacked a cone,
    ldrHvrI is set to 0 when function is first called
    a1: height to lift drfb above cone
*/
bool loaderGrab(double a1) {
    int j = 0;
    static double drfba1, fb0;
    if (ldrGrabI == j++) {
        fb0 = fbGet();
        ldrGrabI++;
    } else if (ldrGrabI == j++) {  // release cone
        drfba1 = a1;
        if (drfba1 < DRFB_LDR_DOWN) drfba1 = DRFB_LDR_DOWN;
        if (drfbGet() > drfba1 - 2) {
            ldrGrabI++;
        } else {
            pidDRFB(drfba1 + 3, 999999, true);
            setRollers(-80);
            pidFB(fb0, 999999, true);
        }
    } else if (ldrGrabI == j++) {  // grab cone
        pidFB(FB_MID_POS, 999999, true);
        if (fbGet() < FB_CLEAR_OF_STACK) {
            setRollers(80);
            pidDRFB(DRFB_LDR_DOWN, 999999, true); /*
             if (drfbGet() < DRFB_LDR_DOWN + 14) {
                 setDRFB(0);
             } else {
                 setDRFB(-127);
             }*/
        } else {
            pidDRFB(drfba1, 999999, true);
        }
        if (drfbGet() < DRFB_LDR_DOWN + 14 && fbGet() < FB_MID_POS + 9) {  // 7, 5
            setRollers(60);
            return true;
        }
    }
    return false;
}

/* PRECONDITIONS:
    DRFB+FB grab
    ldrStackI is set to 0 when function is first called
    a1: drfb upper position: cone above stack
    a2: drfb lower position: cone on stack
*/
bool loaderStack(double a1, double a2) {
    int fbUp = FB_UP_POS - 7;
    int j = 0;
    static double drfba1;
    static unsigned long t0 = 0, lastT = 0;
    if (millis() - lastT > 200) t0 = millis();
    lastT = millis();
    if (ldrStackI == j++) {  // lift cone
        drfba1 = a1;
        if (drfba1 < DRFB_LDR_DOWN) drfba1 = DRFB_LDR_DOWN;
        pidDRFB(drfba1, 999999, true);
        setRollers(millis() - t0 > 100 ? 25 : 80);
        if (drfbGet() > drfba1 - 5) {
            pidFB(fbUp, 999999, true);
            if (fbGet() > fbUp - 10) ldrStackI++;
        } else {
            pidFB(FB_MID_POS, 999999, true);
        }
    } else if (ldrStackI == j++) {  // stack cone
        setRollers(25);
        pidFB(fbUp, 999999, true);
        pidDRFB(a2, 999999, true);
        if (drfbGet() < a2 + 5) return true;
    }
    return false;
}
/*
PRECONDITION: autoStacking = false
range: start to end (inclusive)
valid numbers: 1 to 13
*/
bool autoStacking = false;
bool autoStack(int start, int end) {
    static int q, u, prevU;
    static unsigned long autoStackT0, lastT;
    if (autoStacking == false) {
        q = start - 1;
        u = 0;
        prevU = u;
        autoStacking = true;
        autoStackT0 = millis();
        lastT = millis();
    }
    if (q < end) {
        if (u != prevU) lastT = millis();
        // safety first
        if (millis() - lastT > 2500) {
            resetMotors();
            return false;
        }
        prevU = u;
        int h = 0;
        if (u == h++) {
            ldrGrabI = 0;
            u++;
        } else if (u == h++) {
            if (loaderGrab(drfba[q][0])) {
                ldrStackI = 0;
                u++;
            }
        } else if (u == h++) {
            if (loaderStack(drfba[q][0], drfba[q][1])) u++;
        } else if (u == h++) {
            u = 0;
            q++;
            printf("\n\ncone %d\n\n", q + 1);
        }
    } else {
        printf("\n\n\nSTACK TIME:\t%ld\n\n\n", millis() - autoStackT0);
        return true;
    }
    return false;
}

unsigned long dt = 0, prevT = 0;
int DL_brake_out = 0, DR_brake_out = 0;
void opctrlDrive() {
    DL_brake.kd = 0;
    DR_brake.kd = 0;

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
