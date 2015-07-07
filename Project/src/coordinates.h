#ifndef COORDINATES_H
#define COORDINATES_H

#include <pthread.h>
#include <time.h>
#include "newrobocontrol.h"

class CoordinatesCalibrer
{

public:
    CoordinatesCalibrer();
    CoordinatesCalibrer(Position a, Position b, Position c, Position d);

    bool SetManualCoordCalibration(Position a, Position b, Position c, Position d);

    Position NormalizePosition(Position pos) const;
    Position UnnormalizePosition(Position pos) const;
    double NormalizeAngle(double angle) const;
    double UnnormalizeAngle(double angle) const;

    bool StartCoordCalibration(NewRoboControl *robot1, NewRoboControl *robot2);
    bool WaitForCoordCalibrationEnd(bool stopNow = false);

    bool GetCoordCalibrationResults(double *tx, double *ty, double *theta, double *kx, double *ky) const;
    bool GetCoordCalibrationResults(double *xMax, double *xMin, double *yMax, double *yMin) const;

private:
    static void* CalibrationFn(void *data);

    double m_tx;                    /**< TODO */
    double m_ty;                    /**< TODO */
    double m_cosTheta;              /**< TODO */
    double m_sinTheta;              /**< TODO */
    double m_theta;                 /**< TODO */
    double m_kx;                    /**< TODO */
    double m_ky;                    /**< TODO */
    bool m_isCalibrating;           /**< TODO */
    bool m_calibrationSuccessful;   /**< TODO */
    bool m_stopCalibrating;         /**< TODO */
    pthread_t m_calibrationThread;  /**< TODO */
    NewRoboControl *m_robots[2];    /**< TODO */

    void Init();



};

#endif // COORDINATES_H
