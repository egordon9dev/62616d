#ifndef AUTO_H
#define AUTO_H
#include "pid.h"
bool scoreMG(bool leftSide, int zone);
void auton2(bool leftSide, int stackH, int zone);
void auton3(bool leftSide, int stackH, bool loaderSide, int zone);
void auton4(bool leftSide, bool loaderSide, int zone);
void autonSkills();
#endif  // AUTO_H
