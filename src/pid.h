#ifndef PID_H
#define PID_H
#include <stdbool.h>
typedef struct PidVars {
    double unwind, DONE_ZONE, maxIntegral, iActiveZone, target, sensVal, prevSensVal, prevErr, errTot, kp, ki, kd, deriv;
    unsigned long prevTime, doneTime, prevDUpdateTime;
} PidVars;
typedef struct Slew {
    double a, out;
    unsigned long prevTime;
} Slew;
extern Slew fb_slew, drfb_slew, mgl_slew, DL_slew, DR_slew;
extern PidVars drfb_pid, drfb_pid_auto, fb_pid, fb_pid_auto, mgl_pid, DL_pid, DR_pid, DLshort_pid, DRshort_pid, DLturn_pid, DRturn_pid, turnUsCone_pid, driveCurve_pid, turnCurve_pid, pidDef;
#define LONG_MAX 2147483647
#define DBL_MAX 999999999.999999

double updateSlew(Slew *slew, double in);

//----- proportional + integral + derivative control feedback -----//
double updatePID(PidVars *pidVars);

/*
    returns true if pid (p) is closer than distance (d) and slower than speed (s)
    otherwise returns false
*/
bool killPID(int d, int s, PidVars *p);

// set fb angle with PID
bool pidFB(double a, unsigned long wait, bool auton);

// set arm angle with PID
bool pidDRFB(double a, unsigned long wait, bool auton);

void syncDRFBFB();

bool pidMGL(double a, unsigned long wait);
bool pidMGLSubD(double a, unsigned long wait, bool subD);

// dist: inches
bool pidDumbDrive(double dist, unsigned long wait);
bool pidDrive(double dist, unsigned long wait);
bool pidDriveShort(double dist, unsigned long wait);
// angle: degrees
bool pidTurn(double angle, unsigned long wait);
bool pidTurnSweep(double angleL, double angleR, bool activeL, bool activeR, bool driveShort, unsigned long wait);
extern bool settingDownStack;
bool setDownStack();
// void setDownStack();

#endif
