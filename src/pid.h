#ifndef PID_H
#define PID_H
#include <stdbool.h>
typedef struct PidVars {
    double unwind, DONE_ZONE, maxIntegral, target, sensVal, prevSensVal, prevErr, errTot, kp, ki, kd;
    unsigned long prevTime, doneTime;
} PidVars;
typedef struct LPF {
    double a, out;
} LPF;
extern LPF fb_lpf, drfb_lpf, mgl_lpf, DL_lpf, DR_lpf, DL_lpf_auto, DR_lpf_auto;
extern PidVars pidDef, drfb_pid, fb_pid, mgl_pid, DL_pid, DR_pid, turn_pid;
#define LONG_MAX 2147483647
#define DBL_MAX 999999999.999999
void resetDone(PidVars *pidVars);

double updateLPF(LPF *lpf, double in);

//----- proportional + integral + derivative control feedback -----//
double updatePID(PidVars *pidVars);

/*
    returns 1 if pid (p) is closer than distance (d) and slower than speed (s)
    otherwise returns 0
*/
int killPID(int d, int s, PidVars *p);

// set chain-bar angle with PID
int pidFB(double a, int wait);

// set arm angle with PID
int pidDRFB(double a, int wait);

int pidMGL(double a, int wait);

// set chain bar and arm with PID to stack given cone
int stack(int cone);
int getDRFB(int cone);
int getFB(int cone);
// return lift to pick up cones
void returnLift();

// dist: inches
void pidDrive(double dist);
// angle: degrees
void pidTurn(double angle);

#endif
