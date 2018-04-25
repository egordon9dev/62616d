#include "setup.h"
#include "pid.h"
#include "pros_src/myAPI.h"

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
    static unsigned long t0;
    static int prevN = -999;
    DL_pid.sensVal = eDLGet();
    updatePID(&DL_pid);
    if (abs(n - prevN) > 10) {
        t0 = millis();
        prevN = n;
    }
    bool stall = (isAutonomous() ? true : !joystickGetDigital(2, 8, JOY_UP)) && millis() - t0 > 800 && fabs((DL_pid.deriv) / (DL_pid.kd)) < 0.2;
    int max = stall ? 45 : 127;
    n = limInt(n, -max, max);
    n = updateSlew(&DL_slew, n);
    printf("L%d ", n);
    motorSet(M3, n);
    motorSet(M4_5, n);
}
void setDR(int n) {  //	set left drive motors
    static unsigned long t0;
    static int prevN = -999;
    DR_pid.sensVal = eDRGet();
    updatePID(&DR_pid);
    if (abs(n - prevN) > 10) {
        t0 = millis();
        prevN = n;
    }
    bool stall = (isAutonomous() ? true : !joystickGetDigital(2, 8, JOY_UP)) && millis() - t0 > 800 && fabs((DR_pid.deriv) / (DR_pid.kd)) < 0.2;
    int max = stall ? 45 : 127;
    n = limInt(n, -max, max);
    n = updateSlew(&DR_slew, n);
    printf("R%d ", n);
    motorSet(M0, -n);
    motorSet(M1_2, -n);
}
void setDRFB(int n) {  //	set main drfb lift
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
    motorSet(M8_9, -n);
}
void setDRFBUnlim(int n) {  //	set main drfb lift
    limMotorVal(&n);
    int max = 50;
    double drfbA = drfbGet();
    if ((drfbA < DRFB_MIN / 2.0 && n < 0) || (drfbA > DRFB_MAX1 && n > 0)) {
        if (n > max) n = max;
        if (n < -max) n = -max;
    }
    n = updateSlew(&drfb_slew, n);
    motorSet(M8_9, -n);
}

void setFB(int n) {
    static unsigned long t0;
    static int prevN = -999;
    fb_pid_auto.sensVal = fbGet();
    updatePID(&fb_pid_auto);
    if (abs(n - prevN) > 10) {
        t0 = millis();
        prevN = n;
    }
    double fbVel = -(fb_pid_auto.deriv) / (fb_pid_auto.kd);
    bool stall = millis() - t0 > 400 && ((n < 0 && fbVel > -0.12) || (n > 0 && fbVel < 0.12));
    int maxStall = stall ? 30 : 127;
    n = limInt(n, -maxStall, maxStall);
    int max = 20;
    lcdPrint(LCD, 1, "f%d ", n);
    double fbA = fbGet();
    if (fbA > FB_MAX && n > max) n = max;
    if (fbA < FB_MIN && n < -max) n = -max;
    // if (fbA < FB_MIN_HOLD_ANGLE && n < 0) n = 0;
    n = updateSlew(&fb_slew, n);
    printf("f%d ", n);
    motorSet(M10, -n);
}
void setMGL(int n) {  //	set mobile goal lift
    // when drfb is down limit mgl movement to certain cases
    if (drfbGet() < DRFB_MGL_ACTIVE && !(mglGet() < MGL_MIN && n < 0) && !(mglGet() > MGL_ACTIVE2 && drfbGet() > DRFB_MGL_ACTIVE2 && n > 0) && (mglGet() < MGL_ACTIVE3)) n = 0;

    static unsigned long t0;
    static int prevN = -999;
    mgl_pid.sensVal = mglGet();
    updatePID(&mgl_pid);
    if (abs(n - prevN) > 10) {
        t0 = millis();
        prevN = n;
    }
    double mglVel = -(mgl_pid.deriv) / (mgl_pid.kd);
    bool stall = millis() - t0 > 350 && ((n < 0 && mglVel > -0.06) || (n > 0 && mglVel < 0.06));
    int maxStall = stall ? 30 : 127;
    n = limInt(n, -maxStall, maxStall);

    int maxD = 24, maxU = 22;
    if (mglGet() > MGL_MAX) {
        if (n >= 0) n = maxD;
    }
    if (mglGet() < MGL_MIN) {
        if (n <= 0) n = -maxU;
    }
    n = updateSlew(&mgl_slew, n);
    printf("m%d ", n);
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
Ultrasonic us;
void setupSens() {
    DL_brake = pidDef;
    DR_brake = pidDef;
    eDL = encoderInit(DRIVE_L_ENC_T, DRIVE_L_ENC_B, false);
    eDR = encoderInit(DRIVE_R_ENC_B, DRIVE_R_ENC_T, false);
    encoderReset(eDL);
    encoderReset(eDR);
    us = myUltrasonicInit(US_OUT, US_IN);
    analogCalibrate(LT1);
    analogCalibrate(LT2);
}
#define POT_SENSITIVITY 0.06105006105
double drfbGet() { return (2727 - analogRead(DRFB_POT)) * POT_SENSITIVITY; }
double fbGet() { return (503 - analogRead(FB_POT)) * POT_SENSITIVITY + 140; }
double mglGet() { return (2550 - analogRead(MGL_POT)) * POT_SENSITIVITY; }
int eDLGet() { return encoderGet(eDL); }
int eDRGet() { return encoderGet(eDR); }
int usGet() { return myUltrasonicGet(us); }
int lt1Get() { return analogReadCalibrated(LT1); }
int lt2Get() { return analogReadCalibrated(LT2); }

void printEnc() { printf("dr4b %d fb %d mgl %d dDst %d %d dAng %d %d us %d lt %d %d t %ld\n", (int)drfbGet(), (int)fbGet(), (int)mglGet(), (int)(eDLGet() / DRIVE_TICKS_PER_IN), (int)(eDRGet() / DRIVE_TICKS_PER_IN), (int)(eDLGet() / DRIVE_TICKS_PER_DEG), (int)(eDRGet() / DRIVE_TICKS_PER_DEG), usPredict(), lt1Get(), lt2Get(), millis()); }
void printDrv() {
    printf("d %d/%d %d/%d dt %d/%d %d/%d ", (int)DL_pid.sensVal, (int)DL_pid.target, (int)DR_pid.sensVal, (int)DR_pid.target, (int)DLturn_pid.sensVal, (int)DLturn_pid.target, (int)DRturn_pid.sensVal, (int)DRturn_pid.target);
    printf("ds %d/%d %d/%d t %ld ", (int)DLshort_pid.sensVal, (int)DLshort_pid.target, (int)DRshort_pid.sensVal, (int)DRshort_pid.target, millis());
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
    printf("us %d lt %d\n", (int)usPredict(), lt1Get());
}
/*
##     ##  ######     ########  ########  ######## ########  ####  ######  ########
##     ## ##    ##    ##     ## ##     ## ##       ##     ##  ##  ##    ##    ##
##     ## ##          ##     ## ##     ## ##       ##     ##  ##  ##          ##
##     ##  ######     ########  ########  ######   ##     ##  ##  ##          ##
##     ##       ##    ##        ##   ##   ##       ##     ##  ##  ##          ##
##     ## ##    ##    ##        ##    ##  ##       ##     ##  ##  ##    ##    ##
 #######   ######     ##        ##     ## ######## ########  ####  ######     ##
*/

/*
uses linear approximation to estimate true ultrasound sensor reading
this is meant to compensate for the 50 ms gap between us sensor updates

PRECONDITION: usPredicted set to 0 before first function call
*/
int usPredict() {
    static int prevSens = 0;
    int curSens = usGet();
    if (curSens > 0) prevSens = curSens;
    return prevSens;
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
        autoSel.nAuton = 4;
    } else if (btn == LCD_BTN_RIGHT && !(prevBtn & LCD_BTN_RIGHT)) {
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
        lcdPrint(LCD, 1, "A %d:%d", autoSel.nAuton, autoSel.zone);
        lcdPrint(LCD, 2, " %s", autoSel.leftSide ? "L" : "R");
    }
    prevBtn = btn;
}
/*
########  #### ########  ######## ########  ########  #### ##     ## ########
##     ##  ##  ##     ## ##       ##     ## ##     ##  ##  ##     ## ##
##     ##  ##  ##     ## ##       ##     ## ##     ##  ##  ##     ## ##
########   ##  ########  ######   ##     ## ########   ##  ##     ## ######
##         ##  ##        ##       ##     ## ##   ##    ##   ##   ##  ##
##         ##  ##        ##       ##     ## ##    ##   ##    ## ##   ##
##        #### ##        ######## ########  ##     ## ####    ###    ########
*/

unsigned long pipeDriveT0 = 0;
bool pipeDrive() {
    int pwr = 45 + usPredict();
    if (usPredict() > 5) pipeDriveT0 = millis();
    if (millis() - pipeDriveT0 < 100) {
        setDL(pwr);
        setDR(pwr);
        return false;
    } else {
        setDL(pwr);
        setDR(pwr);
        return true;
    }
}
unsigned long pipeDriveFastT0 = 0;
bool pipeDriveFast() {
    if (usPredict() > 5) pipeDriveFastT0 = millis();
    if (millis() - pipeDriveFastT0 < 100) {
        setDL(127);
        setDR(127);
        return false;
    } else {
        setDL(50);
        setDR(50);
        return true;
    }
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

int drfba[][2] = {{23, 0}, {28, 10}, {38, 19}, {47, 29}, {55, 39}, {62, 47}, {70, 55}, {77, 62}, {86, 70}, {95, 78}, {105, 86}, {118, 95}, {124, 105}};
/* PRECONDITIONS:
-DRFB up
-fb at FB_UP_POS
*/
bool stackConeQ(int q) {
    double a2 = drfba[q][1];
    double da = drfbGet();
    if (da > a2 + 8) {
        setDRFB(-127);
    } else if (da > a2 + 4) {
        setDRFB(-50);
    } else {
        if (q < 3) {
            pidDRFB(a2, 999999, true);
        } else {
            setDRFB(-20);
        }
    }
    if (drfbGet() < a2 + 4) {
        pidFB(FB_MID_POS, 999999, true);
        if (fbGet() < FB_MID_POS + 30) return true;
    } else {
        pidFB(FB_UP_POS, 999999, true);
    }
    return false;
}
double myAsin(double d) { return asin(limDouble(d, -1.0, 1.0)); }
bool liftConeQ(int q) {
    double a1 = drfba[q][0];
    if (fabs(fbGet() - FB_MID_POS) < 4 || drfbGet() > 8) {
        pidDRFB(a1 + 7, 999999, true);
    } else {
        setDRFB(0);
    }
    if (drfbGet() > a1 - 3) {
        pidFB(FB_UP_POS, 999999, true);
        if (fbGet() > FB_UP_POS - 6) return true;
    } else if (drfbGet() > 28) {
        pidFB(FB_HALF_UP_POS - 12, 999999, true);
    } else {
        pidFB(FB_MID_POS, 999999, true);
    }
    return false;
}
/* PRECONDITIONS:
    - DRFB+FB just stacked a cone     or      robot is lined up for first cone
    - asi set to 0
    - ** drive encoders should not move or be reset in between function calls **
*/
int asi;  // auto stack index
int loaderGrabAndStack(int q, bool firstCone, bool lastCone) {
    static int u, prevU, prevAsi;  // auto stack sub-index

    static unsigned long prevT;
    int driveT = 200;
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
                        driveDone = pidDriveShort(-driveDist, driveT);
                    } else {
                        driveDone = true;
                    }
                    if (stackConeQ(q) && driveDone) {
                        DLshort_pid.doneTime = LONG_MAX;
                        DRshort_pid.doneTime = LONG_MAX;
                        resetDriveEnc();
                        u++;
                    }
                } else if (u == h++) {  // drive forward, hover over cone
                    if (q >= AUTO_STACK_STATIONARY) {
                        u++;
                    } else {
                        pidFB(FB_MID_POS + 25, 999999, true);
                        pidDRFB(DRFB_LDR_UP, 999999, true);
                        if (drfbGet() > DRFB_LDR_UP - 6 && pidDriveShort(driveDist, driveT)) u++;
                    }
                } else if (u == h++) {
                    asi++;
                }
            }
        } else if (asi == j++) {  // grab cone
            // fix this, angles are guessed
            if (drfbGet() > DRFB_LDR_DOWN + 12) {
                pidFB(FB_MID_POS + 10, 999999, true);
            } else if (fbGet() > FB_MID_POS) {
                setFB(-127);
            } else {
                setFB(-50);
            }
            pidDRFB(DRFB_LDR_DOWN - 2, 999999, true);
            if (fbGet() < FB_MID_POS + 4 && drfbGet() < DRFB_LDR_DOWN) {
                resetDriveEnc();
                DLshort_pid.doneTime = LONG_MAX;
                DRshort_pid.doneTime = LONG_MAX;
                asi++;
            }
        } else if (asi == j++) {  // stack cone, drive back
            if (q < AUTO_STACK_STATIONARY && !lastCone) pidDriveShort(-driveDist, 999999);
            if (liftConeQ(q)) asi++;
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
        int n = loaderGrabAndStack(q, q == start - 1, q == end - 1);
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
bool pipeDriving = false;
bool curSetDownStack = false;
int prevDrv = 0;
void opctrlDrive() {
    if (joystickGetDigital(1, 7, JOY_DOWN)) { pipeDriving = true; }
    if (pipeDriving) pipeDrive();
    DL_brake.kd = 0;
    DR_brake.kd = 0;

    int drv = joystickGetAnalog(1, 3);
    prevDrv = drv;
    int trn = joystickGetAnalog(1, 1);

    if (abs(drv) < 70) trn *= DRIVE_TURN_MAX / 127.0;
    drv = limInt(drv, -DRIVE_DRIVE_MAX, DRIVE_DRIVE_MAX);
    if (abs(drv) < JOY_THRESHOLD) drv = 0;
    if (abs(trn) < JOY_THRESHOLD) trn = 0;
    if (drv != 0 || trn != 0) pipeDriving = false;
    /*
double lvel = 0.0, rvel = 0.0;
if (dt != 0) {
lvel = (DL_brake.prevSensVal - eDLGet()) / (double)dt;
rvel = (DR_brake.prevSensVal - eDRGet()) / (double)dt;
}
DL_brake_out = lvel * DL_brake.kd;
DR_brake_out = rvel * DR_brake.kd;*/
    if (!pipeDriving) {
        setDL(drv + trn);
        setDR(drv - trn);
    } /*
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
    prevT = millis();*/
}
///////////////////////////////////
