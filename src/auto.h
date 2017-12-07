#ifndef AUTO_H
#define AUTO_H
#include "pid.h"
void auton1(PidVars *DL_pid, PidVars *DR_pid, PidVars *DLturn_pid, PidVars *DRturn_pid, PidVars *drfb_pid, PidVars *fb_pid, bool right, bool skills);
#endif  // AUTO_H
