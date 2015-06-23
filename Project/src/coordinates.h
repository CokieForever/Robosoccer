#ifndef COORDINATES_H
#define COORDINATES_H

#include <pthread.h>
#include <time.h>
#include "robo_control.h"

class CoordinatesCalibrer
{

public:
    CoordinatesCalibrer();
    CoordinatesCalibrer(Position a, Position b, Position c, Position d);

    bool SetManualCoordCalibration(Position a, Position b, Position c, Position d);
    Position NormalizePosition(Position pos) const;
    Position UnnormalizePosition(Position pos) const;

    bool StartCoordCalibration(RoboControl *robot1, RoboControl *robot2);
    bool WaitForCoordCalibrationEnd(bool stopNow = false);

    bool GetCoordCalibrationResults(double *tx, double *ty, double *theta, double *kx, double *ky) const;
    bool GetCoordCalibrationResults(double *xMax, double *xMin, double *yMax, double *yMin) const;

private:
    static void* CalibrationFn(void *data);

    double m_tx, m_ty, m_cosTheta, m_sinTheta, m_kx, m_ky;
    bool m_isCalibrating;
    bool m_calibrationSuccessful;
    bool m_stopCalibrating;
    pthread_t m_calibrationThread;
    RoboControl *m_robots[2];

    void Init();



};

#endif // COORDINATES_H
