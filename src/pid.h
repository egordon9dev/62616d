#ifndef PID_H
#define PID_H
#include <stdbool.h>
typedef struct PidVars {
    double unwind, DONE_ZONE, maxIntegral, iActiveZone, target, sensVal, prevSensVal, prevErr, errTot, kp, ki, kd;
    unsigned long prevTime, doneTime;
} PidVars;
typedef struct Slew {
    double a, out;
    unsigned long prevTime;
} Slew;
extern Slew fb_slew, drfb_slew, mgl_slew, DL_slew, DR_slew, DL_slew_auto, DR_slew_auto;
extern PidVars pidDef, drfb_pid, drfb_pid_auto, fb_pid_auto, mgl_pid, DL_pid, DR_pid, turn_pid;
#define LONG_MAX 2147483647
#define DBL_MAX 999999999.999999
void resetDone(PidVars *pidVars);

double updateSlew(Slew *slew, double in);

//----- proportional + integral + derivative control feedback -----//
double updatePID(PidVars *pidVars);

/*
    returns true if pid (p) is closer than distance (d) and slower than speed (s)
    otherwise returns false
*/
bool killPID(int d, int s, PidVars *p);

// set chain-bar angle with PID
bool pidFB(double a, unsigned long wait, bool auton);

// set arm angle with PID
bool pidDRFB(double a, unsigned long wait, bool auton);

bool pidMGL(double a, unsigned long wait);

// set chain bar and arm with PID to stack given cone
int getDRFB(int cone);
int getFB(int cone);
// return lift to pick up cones (returns true if lift is returned)
bool contReturnLift(bool auton, unsigned long wait);
void startReturnLift(bool auton);

// dist: inches
bool pidDrive(double dist, unsigned long wait);
// angle: degrees
bool pidTurn(double angle, unsigned long wait);

#endif
