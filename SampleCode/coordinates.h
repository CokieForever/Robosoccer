#ifndef COORDINATES_H
#define COORDINATES_H

#include <pthread.h>
#include <time.h>
#include "robo_control.h"

bool SetManualCoordCalibration(Position a, Position b, Position c, Position d);
Position NormalizePosition(Position pos);
Position UnnormalizePosition(Position pos);

bool StartCoordCalibration(RoboControl *robot1, RoboControl *robot2);
bool WaitForCoordCalibrationEnd(bool stopNow = false);

bool GetCoordCalibrationResults(double *tx, double *ty, double *theta, double *kx, double *ky);

#endif // COORDINATES_H
