#include "setup.h"
#include "API.h"
#include "pid.h"

double fbUpP = FB_UP_P0;

int DRIVE_DRIVE_MAX = 110, DRIVE_TURN_MAX = 90;

/*
##     ##  #######  ########  #######  ########   ######
###   ### ##     ##    ##    ##     ## ##     ## ##    ##
#### #### ##     ##    ##    ##     ## ##     ## ##
## ### ## ##     ##    ##    ##     ## ########   ######
##     ## ##     ##    ##    ##     ## ##   ##         ##
##     ## ##     ##    ##    ##     ## ##    ##  ##    ##
##     ##  #######     ##     #######  ##     ##  ######
*/

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
    double drfbA = drfbGet();
    if (drfbA > DRFB_MAX_HOLD_ANGLE && n > 0) n = 0;
    if ((drfbA < DRFB_MIN && n < 0) || (drfbA > DRFB_MAX1 && n > 0)) {
        if (drfbA < DRFB_MIN / 2 || drfbA > DRFB_MAX2) max /= 2;
        if (n > max) n = max;
        if (n < -max) n = -max;
    }
    n = updateSlew(&drfb_slew, n);
    n *= -1;
    motorSet(M8_9, n);
}

void setFB(int n) {
    limMotorVal(&n);
    int max = 20;
    double fbA = fbGet();
    if (fbA > FB_MAX && n > max) n = max;
    if (fbA < FB_MIN && n < -max) n = -max;
    if (fbA < FB_MIN_HOLD_ANGLE && n < 0) n = 0;
    n = updateSlew(&fb_slew, n);
    motorSet(M10, -n);
}
void setMGL(int n) {  //	set mobile goal lift
    limMotorVal(&n);
    // when drfb is down only allow holding MG up
    if (drfbGet() < 19 && !(mglGet() < MGL_MIN && n < 0)) n = 0;
    int maxD = 22, maxU = 22;
    if (mglGet() > MGL_MAX) {
        if (n >= 0) n = maxD;
    }
    if (mglGet() < MGL_MIN) {
        if (n <= 0) n = -maxU;
    }
    n = updateSlew(&mgl_slew, n);
    motorSet(M6_7, n);
}
void stopMGL() { motorSet(M6_7, updateSlew(&mgl_slew, 0)); }

void resetMotors() {
    for (int i = 1; i <= 10; i++) { motorSet(i, 0); }
}
/*
 ######  ######## ##    ##  ######   #######  ########   ######
##    ## ##       ###   ## ##    ## ##     ## ##     ## ##    ##
##       ##       ####  ## ##       ##     ## ##     ## ##
 ######  ######   ## ## ##  ######  ##     ## ########   ######
      ## ##       ##  ####       ## ##     ## ##   ##         ##
##    ## ##       ##   ### ##    ## ##     ## ##    ##  ##    ##
 ######  ######## ##    ##  ######   #######  ##     ##  ######
*/

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
Ultrasonic us1;
void setupSens() {
    DL_brake = pidDef;
    DR_brake = pidDef;
    eDL = encoderInit(DRIVE_L_ENC_T, DRIVE_L_ENC_B, false);
    eDR = encoderInit(DRIVE_R_ENC_B, DRIVE_R_ENC_T, false);
    encoderReset(eDL);
    encoderReset(eDR);
    us1 = ultrasonicInit(US1_OUT, US1_IN);
}
void shutdownSens() { ultrasonicShutdown(us1); }
#define POT_SENSITIVITY 0.06105006105
double drfbGet() { return (2727 - analogRead(DRFB_POT)) * POT_SENSITIVITY; }
double fbGet() { return (503 - analogRead(FB_POT)) * POT_SENSITIVITY + 140; }
double mglGet() { return (4095 - analogRead(MGL_POT)) * POT_SENSITIVITY; }
int eDLGet() { return encoderGet(eDL); }
int eDRGet() { return encoderGet(eDR); }
int us1Get() { return ultrasonicGet(us1); }

void printEnc() { printf("dr4b: %d\tfb: %d\tmgl: %d\tDL: %d\tDR: %d\tus1:%d\t\n", (int)drfbGet(), (int)fbGet(), (int)mglGet(), eDLGet(), eDRGet(), us1Get()); }
void printDrv() {
    printf("d %d/%d %d/%d dt %d/%d %d/%d t %ld ", (int)DL_pid.sensVal, (int)DL_pid.target, (int)DR_pid.sensVal, (int)DR_pid.target, (int)DLturn_pid.sensVal, (int)DLturn_pid.target, (int)DRturn_pid.sensVal, (int)DRturn_pid.target, millis());
    printf("dCv %d/%d tCv %d/%d ", (int)driveCurve_pid.sensVal, (int)driveCurve_pid.target, (int)turnCurve_pid.sensVal, (int)turnCurve_pid.target);
}
void printEnc_pidDrive() {
    printDrv();
    printf("\n");
}
void printDRFBFB() { printf("drfb %d/%d fb %d/%d ", (int)drfb_pid_auto.sensVal, (int)drfb_pid_auto.target, (int)fb_pid_auto.sensVal, (int)fb_pid_auto.target); }
void printMGL() { printf("mgl: %d/%d\t", (int)mgl_pid.sensVal, (int)mgl_pid.target); }
void printEnc_pidDRFBFB() {
    printDRFBFB();
    printf("\n");
}
void printEnc_all() {
    printDrv();
    printMGL();
    printDRFBFB();
    printf("us %d\n", us1Get());
}
/*
uses linear approximation to estimate true ultrasound sensor reading
this is meant to compensate for the 50 ms gap between us sensor updates

PRECONDITION: usPredicted set to 0 before first function call
*/
unsigned long usPredicted = 0;
double usPredict() {
    static int prevUs1;
    static unsigned long prevT;

    int curUs1 = us1Get();
    unsigned long curT = millis();

    static double lpfSlope = 0.0;
    if (usPredicted > 0) {
        double curSlope = (double)(curUs1 - prevUs1) / (double)(curT - prevT);
        if (usPredicted == 1) {
            lpfSlope = curSlope;
        } else if ((curUs1 != prevUs1 || curT - prevT > 50) && curUs1 != 0) {
            lpfSlope = /*(0.5 * lpfSlope) + (0.5 * */ curSlope /*)*/;
            prevUs1 = curUs1;
            prevT = curT;
        }
    } else {
        prevUs1 = curUs1;
        prevT = curT;
    }
    usPredicted++;
    return prevUs1 + lpfSlope * (curT - prevT);
}
/*
   ###    ##     ## ########  #######      ######  ######## ##       ########  ######  ########
  ## ##   ##     ##    ##    ##     ##    ##    ## ##       ##       ##       ##    ##    ##
 ##   ##  ##     ##    ##    ##     ##    ##       ##       ##       ##       ##          ##
##     ## ##     ##    ##    ##     ##     ######  ######   ##       ######   ##          ##
######### ##     ##    ##    ##     ##          ## ##       ##       ##       ##          ##
##     ## ##     ##    ##    ##     ##    ##    ## ##       ##       ##       ##    ##    ##
##     ##  #######     ##     #######      ######  ######## ######## ########  ######     ##
*/
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
            if (autoSel.zone == 20 && autoSel.stackH > 4) autoSel.stackH = 1;
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

/*
   ###    ##     ## ########  #######      ######  ########    ###     ######  ##    ##
  ## ##   ##     ##    ##    ##     ##    ##    ##    ##      ## ##   ##    ## ##   ##
 ##   ##  ##     ##    ##    ##     ##    ##          ##     ##   ##  ##       ##  ##
##     ## ##     ##    ##    ##     ##     ######     ##    ##     ## ##       #####
######### ##     ##    ##    ##     ##          ##    ##    ######### ##       ##  ##
##     ## ##     ##    ##    ##     ##    ##    ##    ##    ##     ## ##    ## ##   ##
##     ##  #######     ##     #######      ######     ##    ##     ##  ######  ##    ##
*/

int drfba[][2] = {{20, 0}, {27, 6}, {36, 18}, {45, 28}, {52, 37}, {61, 45}, {68, 53}, {77, 62}, {84, 69}, {92, 77}, {102, 85}, {115, 94}, {124, 103}};
/* PRECONDITIONS:
-DRFB up
-fb at FB_UP_POS
*/
bool stackConeQ(int q) {
    double a2 = drfba[q][1];
    if (drfbGet() > a2) {
        setDRFB(-127);
    } else {
        setDRFB(30);
    }
    if (drfbGet() < a2 + 4) {
        pidFB(FB_MID_POS, 999999, true);
        if (fbGet() < FB_MID_POS + 25) return true;
    } else {
        pidFB(FB_UP_POS, 999999, true);
    }
    return false;
}
/* PRECONDITIONS:
    - DRFB+FB just stacked a cone     or      robot is lined up for first cone
    - asi set to 0
    - ** drive encoders should not move or be reset in between function calls **
*/
int asi;  // auto stack index
int loaderGrabAndStack(int q, bool firstCone) {
    static int u, prevU, prevAsi;  // auto stack sub-index
    static unsigned long prevT;
    int driveT = 200;
    unsigned long t0 = 0;
    double driveDist = 7.0;
    // allows code to progress to next step immediately rather than waiting for the next task iteration
    bool allowRepeat = true;
    while (allowRepeat) {
        allowRepeat = false;
        int j = 0;
        if (asi == j++) {
            u = 0;
            prevT = millis();
            prevU = u;
            prevAsi = asi;
            asi++;
        } else if (asi == j++) {  // release cone
            if (firstCone) {
                asi++;
            } else {
                int h = 0;
                if (u == h++) {  // stack cone and finish driving back
                    bool driveDone = false;
                    if (q < AUTO_STACK_STATIONARY) {
                        driveDone = pidDrive(-driveDist, driveT, false);
                    } else {
                        driveDone = true;
                    }
                    if (stackConeQ(q) && driveDone) {
                        DL_pid.doneTime = LONG_MAX;
                        DR_pid.doneTime = LONG_MAX;
                        resetDriveEnc();
                        u++;
                    }
                } else if (u == h++) {  // drive forward, hover over cone
                    if (q >= AUTO_STACK_STATIONARY) {
                        u++;
                    } else {
                        pidFB(FB_MID_POS + 25, 999999, true);
                        pidDRFB(DRFB_LDR_UP, 999999, true);
                        if (drfbGet() > DRFB_LDR_UP - 6 && pidDrive(driveDist, driveT, false)) u++;
                    }
                } else if (u == h++) {
                    asi++;
                }
            }
        } else if (asi == j++) {
            t0 = LONG_MAX;
            asi++;
        } else if (asi == j++) {  // grab cone
            if (fbGet() < FB_MID_POS + 3 && t0 == LONG_MAX) t0 = millis();
            if (millis() - t0 > 200) {
                pidFB(FB_MID_POS, 999999, true);
            } else {
                if (t0 == LONG_MAX) {
                    setFB(-127);
                } else {
                    setFB(-50);
                }
            }
            pidDRFB(DRFB_LDR_DOWN, 999999, true);
            if (fabs(FB_MID_POS - fbGet()) < 6 && fabs(DRFB_LDR_DOWN - drfbGet()) < 4) {
                resetDriveEnc();
                DL_pid.doneTime = LONG_MAX;
                DR_pid.doneTime = LONG_MAX;
                asi++;
            }
        } else if (asi == j++) {  // stack cone, drive back
            double a1 = drfba[q][0];
            if (a1 < DRFB_LDR_DOWN) a1 = DRFB_LDR_DOWN;
            pidDRFB(a1 + 5, 999999, true);
            if (q < AUTO_STACK_STATIONARY) pidDrive(-driveDist, 999999, false);
            if (drfbGet() > a1) {
                pidFB(FB_UP_POS, 999999, true);
                if (fbGet() > FB_UP_POS - 6) { asi++; }
            } else {
                pidFB(FB_MID_POS, 999999, true);
            }
        } else if (asi == j++) {
            return 1;
        }
        if (u != prevU || asi != prevAsi) {
            prevT = millis();
            allowRepeat = true;
        }
        // safety first (ptc tripped or robot got stuck)
        if (millis() - prevT > 2500) {
            resetMotors();
            return -1;
        }
        prevU = u;
        prevAsi = asi;
    }
    return 0;
}
/*
PRECONDITION: autoStacking = false
range: start to end (inclusive)
valid numbers: 1 to 13
*/
bool autoStacking = false;
bool autoStack(int start, int end) {
    static int q, u;
    static unsigned long autoStackT0;
    if (autoStacking == false) {
        q = start - 1;
        u = 0;
        autoStacking = true;
        autoStackT0 = millis();
    }
    if (q < end) {
        int h = 0;
        if (u == h++) {
            asi = 0;
            u++;
        }
        int n = loaderGrabAndStack(q, q == start - 1);
        if (n == 1) {
            u = 0;
            q++;
            printf("\n\nSTART: cone %d\n\n", q + 1);
        } else if (n == -1) {
            printf("\n\nTIMEOUT\n\n");
            return true;
        }
    } else {
        printf("\n\n\nSTACK TIME:\t%ld\n\n\n", millis() - autoStackT0);
        resetMotors();
        return true;
    }
    return false;
}
/*
 #######  ########   ######  ######## ########  ##          ########  ########  #### ##     ## ########
##     ## ##     ## ##    ##    ##    ##     ## ##          ##     ## ##     ##  ##  ##     ## ##
##     ## ##     ## ##          ##    ##     ## ##          ##     ## ##     ##  ##  ##     ## ##
##     ## ########  ##          ##    ########  ##          ##     ## ########   ##  ##     ## ######
##     ## ##        ##          ##    ##   ##   ##          ##     ## ##   ##    ##   ##   ##  ##
##     ## ##        ##    ##    ##    ##    ##  ##          ##     ## ##    ##   ##    ## ##   ##
 #######  ##         ######     ##    ##     ## ########    ########  ##     ## ####    ###    ########
*/
unsigned long dt = 0, prevT = 0;
int DL_brake_out = 0, DR_brake_out = 0;
void opctrlDrive() {
    DL_brake.kd = 0;
    DR_brake.kd = 0;

    const int td = 17;
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
