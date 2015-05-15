#ifndef COORDINATES_H
#define COORDINATES_H

#include <pthread.h>
#include <time.h>
#include "robo_control.h"

bool StartCoordCalibration(RoboControl *robot1, RoboControl *robot2);
bool StopCoordCalibration();
bool WaitForCoordCalibrationEnd();
bool GetCoordCalibrationResults(double *xMax, double *yMax, double *xMin, double *yMin);

#endif // COORDINATES_H
