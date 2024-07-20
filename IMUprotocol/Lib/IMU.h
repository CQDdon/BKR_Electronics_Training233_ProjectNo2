#ifndef IMU_H
#define IMU_H

#include "main.h"

extern float axisAcc[3];
extern float axisAngVel[3];
extern float axisAng[3];

void dataHandle(uint8_t buffer[]);

#endif // IMU_H