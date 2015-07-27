#ifndef COORDINATES_H
#define COORDINATES_H

#include <pthread.h>
#include <time.h>
#include "newrobocontrol.h"

/**
 * @brief Class used to convert real field coordinates to normalized coordinates and vice-versa.
 *
 * This class is intended to be used as a wrapper for Positions given by the Robosoccer system.
 * After being instantiated, the calibration data can be computed automatically by calling the
 * @ref CoordinatesCalibrer::StartCoordCalibration() function, which will move the provided robots
 * to measure the field. The calibration data can also be entered manually by calling the
 * @ref CoordinatesCalibrer::SetManualCoordCalibration() function.
 * Once the calibration data is set, use the @ref CoordinatesCalibrer::NormalizePosition() and
 * @ref CoordinatesCalibrer::UnnormalizePosition() to switch between normalized and real field
 * coordinates.
 * The calibrator uses a rotation, a translation and a scaling transform to convert to normalized coordinates.
 * It does not support non linear transformations (e.g. optic transformations).
 */
class CoordinatesCalibrer
{

public:
    CoordinatesCalibrer();
    CoordinatesCalibrer(Position a, Position b, Position c, Position d);

    bool SetManualCoordCalibration(Position a, Position b, Position c, Position d);

    Position NormalizePosition(Position pos) const;
    Position NormalizePosition(Position pos, double phi) const;
    Position UnnormalizePosition(Position pos) const;
    Position UnnormalizePosition(Position pos, double phi) const;
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
    bool m_isCalibrating;           /**< Indicates if the automatic calibration is still going on. */
    bool m_calibrationSuccessful;   /**< Indicates if the automatic calibration is successful. */
    bool m_stopCalibrating;         /**< Indicates if the automatic calibration should be stopped. */
    pthread_t m_calibrationThread;  /**< The thread of automatic calibration. */
    NewRoboControl *m_robots[2];    /**< The robots used for the automatic calibration. */

    void Init();



};

#endif // COORDINATES_H
