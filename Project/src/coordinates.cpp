#include "coordinates.h"
#include "geometry.h"


/**
 * @brief Default empty constructor. The calibration data must be set either with @ref CoordinatesCalibrer::SetManualCoordCalibration()
 * or @ref CoordinatesCalibrer::StartCoordCalibration() before using the instance.
 *
 */
CoordinatesCalibrer::CoordinatesCalibrer()
{
    Init();
}

/**
 * @brief Constructor with calibration data (see @ref CoordinatesCalibrer::SetManualCoordCalibration() for details).
 *
 * @param a First position
 * @param b Second position
 * @param c Third position
 * @param d Fourth position
 */
CoordinatesCalibrer::CoordinatesCalibrer(Position a, Position b, Position c, Position d)
{
    Init();
    SetManualCoordCalibration(a, b, c, d);
}

/**
 * @brief Initializes the parameters for a CoordinatesCalibrer instance.
 *
 */
void CoordinatesCalibrer::Init()
{
    m_tx = 0;
    m_ty = 0;
    m_cosTheta = 1;
    m_sinTheta = 0;
    m_kx = 1;
    m_ky = 1;

    m_isCalibrating = false;
    m_calibrationSuccessful = false;
    m_stopCalibrating = false;
    m_calibrationThread = NULL;

    memset(m_robots, 0, sizeof(NewRoboControl*)*2);
}

/**
 * @brief Sets manually the calibration data based on 4 positions
 *
 @verbatim

 ********** A **********
 *                     *
 D          O          B
 *                     *
 ********** C **********
 @endverbatim
 *
 * @param a First position
 * @param b Second position
 * @param c Third position
 * @param d Fourth position
 * @return bool
 */
bool CoordinatesCalibrer::SetManualCoordCalibration(Position a, Position b, Position c, Position d)
{
    m_tx = -(a.GetX() + c.GetX()) / 2;
    m_ty = -(a.GetY() + c.GetY()) / 2;
    double dist = d.DistanceTo(b);
    m_cosTheta = (b.GetX() - d.GetX()) / dist;
    m_sinTheta = (d.GetY() - b.GetY()) / dist;
    m_kx = 2 / dist;
    m_ky = 2 / a.DistanceTo(c);

    m_theta = acos(m_cosTheta);
    if (m_sinTheta < 0)
        m_theta = -m_theta;

    m_calibrationSuccessful = true;
    return true;
}

/**
 * @brief It normalizes a position (real field -> normalized).
 * @deprecated Not supported anymore. Use CoordinatesCalibrer::NormalizePosition(Position) const instead.
 *
 * Normalized coordinates are in [-1 ; 1].
 *
 * @param pos The real field position.
 * @param phi The angle of the robot associated with the position, as returned by the GetPhi() function.
 * @return Position The normalized position.
 */
Position CoordinatesCalibrer::NormalizePosition(Position pos, double phi) const
{
    double x, y;
    phi = phi >= 0 ? phi - M_PI : phi + M_PI;
    ComputeVectorEnd(pos.GetX(), pos.GetY(), phi, 0.025, &x, &y);

    return NormalizePosition(Position(x,y));
}

/**
 * @brief It normalizes a position (real field -> normalized).
 *
 * Normalized coordinates are in [-1 ; 1].
 *
 * @param pos The real field position.
 * @return Position The normalized position.
 */
Position CoordinatesCalibrer::NormalizePosition(Position pos) const
{
    double x = pos.GetX(), y = pos.GetY();

    //Translation
    x += m_tx;
    y += m_ty;

    //Rotation
    x = x * m_cosTheta - y * m_sinTheta;
    y = x * m_sinTheta + y * m_cosTheta;

    //Dilatation
    x *= m_kx;
    y *= m_ky;

    return Position(x, y);
}

/**
 * @brief It unnormalizes a position (normalized -> real field).
 * @deprecated Not supported anymore. Use CoordinatesCalibrer::UnnormalizePosition(Position) const instead.
 *
 * @param pos The normalized position. Coordinates outside the range  [-1,1] will be converted into real positions outside the field.
 * @param phi UNNORMALIZED orientation of the robot, typically given by RoboControl::GetPhi().
 * @return Position The unnormalized position.
 */
Position CoordinatesCalibrer::UnnormalizePosition(Position pos, double phi) const
{
    pos = UnnormalizePosition(pos);

    double x, y;
    ComputeVectorEnd(pos.GetX(), pos.GetY(), phi, 0.025, &x, &y);

    return Position(x, y);
}

/**
 * @brief It unnormalizes a position (normalized -> real field).
 *
 * @param pos The normalized position. Coordinates outside the range  [-1,1] will be converted into real positions outside the field.
 * @return Position The unnormalized position.
 */
Position CoordinatesCalibrer::UnnormalizePosition(Position pos) const
{
    double x = pos.GetX(), y = pos.GetY();

    //Dilatation
    x /= m_kx;
    y /= m_ky;

    //Rotation
    x = x * m_cosTheta + y * m_sinTheta;
    y = -x * m_sinTheta + y * m_cosTheta;

    //Translation
    x -= m_tx;
    y -= m_ty;

    return Position(x, y);
}

/**
 * @brief Normalizes an angle.
 *
 * As the coordinates normalization involves scaling and rotation, the angles values might change when switching between the coordinates systems.
 * This function converts an angle computed in real field coordinates to the same angle in normalized coordinates.
 *
 * @param angle Angle to be normalized.
 * @return double Normalized Angle.
 */
double CoordinatesCalibrer::NormalizeAngle(double angle) const
{
    //Rotation
    angle -= m_theta;
    if (angle < -M_PI)
        angle += 2*M_PI;
    else if (angle > M_PI)
        angle -= 2*M_PI;

    //Dilatation
    double c = cos(angle);
    double y = 1 / (c*c) - 1;
    double c2 = 1 / sqrt(1 + (m_ky/m_kx)*y * (m_ky/m_kx)*y);
    double s2 = y / sqrt((m_kx/m_ky)*(m_kx/m_ky) + y*y);
    angle = acos(c2);
    if (s2 < 0)
        angle = -angle;

    return angle;
}

/**
 * @brief It unnormalizes an angle.
 *
 * Reverse of @ref CoordinatesCalibrer::NormalizeAngle().
 *
 * @param angle Angle to be unnormalized.
 * @return double The unnormalized angle.
 */
double CoordinatesCalibrer::UnnormalizeAngle(double angle) const
{
    //Dilatation
    double c = cos(angle);
    double y = 1 / (c*c) - 1;
    double c2 = 1 / sqrt(1 + (m_kx/m_ky)*y * (m_kx/m_ky)*y);
    double s2 = y / sqrt((m_ky/m_kx)*(m_ky/m_kx) + y*y);
    angle = acos(c2);
    if (s2 < 0)
        angle = -angle;

    //Rotation
    angle += m_theta;
    if (angle < -M_PI)
        angle += 2*M_PI;
    else if (angle > M_PI)
        angle -= 2*M_PI;

    return angle;
}

/**
 * @brief This function starts the automatic coordinates calibration
 *
 * This function will move the robots on the field to determine the field's limits.
 * You should not give order to the robots while the process is in progress.
 * You can call the @ref CoordinatesCalibrer::WaitForCoordCalibrationEnd() function to wait for the end of the process.
 *
 * @param robot1 First robot to use for the calibration. NULL can possibly lead to segfault.
 * @param robot2 Second robot to use for the calibration. NULL can possibly lead to segfault.
 * @return bool True if the coordinates calibration was started. False if not.
 */
bool CoordinatesCalibrer::StartCoordCalibration(NewRoboControl *robot1, NewRoboControl *robot2)
{
    if (robot1)
        m_robots[0] = robot1;
    if (robot2)
        m_robots[1] = robot2;
    pthread_create(&m_calibrationThread, NULL, CalibrationFn, this);

    usleep(0.1e6);
    return true;
}

/**
 * @brief It waits for the end of the automatic coordinates calibration
 *
 * @param stopNow True if the coordinates calbration should stop immediately. False if not.
 * @return bool     True if the coordinates caliration is still not finished. False if it is stopped.
 */
bool CoordinatesCalibrer::WaitForCoordCalibrationEnd(bool stopNow)
{
    if (!m_isCalibrating)
        return false;
    if (stopNow)
        m_stopCalibrating = true;
    pthread_join(m_calibrationThread, NULL);
    return true;
}

/**
 * @brief It gets the current calibration data. A handier version is CoordinatesCalibrer::GetCoordCalibrationResults(double*, double*, double*, double*) const.
 *
 * @param tx Pointer to receive the X translation setting. Can be NULL.
 * @param ty Pointer to receive the Y translation setting. Can be NULL.
 * @param theta Pointer to receive the rotation setting (in radians). Can be NULL.
 * @param kx Pointer to receive the X scaling setting. Can be NULL.
 * @param ky Pointer to receive the Y scaling setting. Can be NULL.
 * @return bool True if the calibration was actually done, False if not (in this case data stays unchanged).
 */
bool CoordinatesCalibrer::GetCoordCalibrationResults(double *tx, double *ty, double *theta, double *kx, double *ky) const
{
    if (!m_calibrationSuccessful)
        return false;

    if (tx)
        *tx = m_tx;
    if (ty)
        *ty = m_ty;
    if (theta)
        *theta = asin(m_sinTheta);
    if (kx)
        *kx = m_kx;
    if (ky)
        *ky = m_ky;

    return true;
}

/**
 * @brief This function gets the current calibration data. More readable alternative to CoordinatesCalibrer::GetCoordCalibrationResults(double*, double*, double*, double*, double*) const.
 *
 * @param xMax A pointer to receive the maximum value for the x coordinate. Can be NULL.
 * @param xMin A pointer to receive the minimum value for the x coordinate. Can be NULL.
 * @param yMax A pointer to receive the maximum value for the y coordinate. Can be NULL.
 * @param yMin A pointer to receive the minimum value for the y coordinate. Can be NULL.
 * @return bool True if the calibration was actually done, False if not (in this case data stays unchanged).
 */
bool CoordinatesCalibrer::GetCoordCalibrationResults(double *xMax, double *xMin, double *yMax, double *yMin) const
{
    if (!m_calibrationSuccessful)
        return false;

    Position lowerRight(1,1), upperLeft(-1,-1);
    upperLeft = NormalizePosition(upperLeft);
    lowerRight = NormalizePosition(lowerRight);

    if (xMax)
        *xMax = lowerRight.GetX();
    if (xMin)
        *xMin = upperLeft.GetX();
    if (yMax)
        *yMax = lowerRight.GetY();
    if (yMin)
        *yMin = upperLeft.GetY();

    return true;
}

#define exit_cal() do {calibrer->m_calibrationSuccessful = !calibrer->m_stopCalibrating; calibrer->m_isCalibrating = false; return NULL;} while (0)
#define usleep_cal(t) do { if (calibrer->m_stopCalibrating) exit_cal(); usleep(t); if (calibrer->m_stopCalibrating) exit_cal(); } while (0)
/**
 * @brief Function called to do the automatic coordinates calibration.
 *
 * @param data Pointer to the associated @ref CoordinatesCalibrer instance.
 */
void* CoordinatesCalibrer::CalibrationFn(void *data)
{
    CoordinatesCalibrer *calibrer = (CoordinatesCalibrer*)data;

    double xMax=0, xMin=0, yMax=0, yMin=0;
    calibrer->m_isCalibrating = true;
    calibrer->m_stopCalibrating = false;

    calibrer->m_robots[0]->GotoXY(0, 0.2, 80, true);
    calibrer->m_robots[1]->GotoXY(0, -0.2, 80, true);
    usleep_cal(7e6);

    calibrer->m_robots[0]->GotoXY(0, 10, 80, false);
    calibrer->m_robots[1]->GotoXY(0, -10, 80, false);
    usleep_cal(2e6);

    bool watch0 = true, watch1 = true;
    double prevY_0 = 0.2, prevY_1 = -0.2;
    while (watch1 || watch0)
    {
        usleep(1e6);

        if (watch0)
        {
            double y = calibrer->m_robots[0]->GetPos().GetY();
            if (y <= prevY_0)
            {
                yMax = prevY_0;
                watch0 = false;
                calibrer->m_robots[0]->GotoXY(0.3, 0.5, 80, true);
            }
            else
                prevY_0 = y;
        }

        if (watch1)
        {
            double y = calibrer->m_robots[1]->GetPos().GetY();
            if (y >= prevY_1)
            {
                yMin = prevY_1;
                watch1 = false;
                calibrer->m_robots[1]->GotoXY(-0.3, -0.5, 80, true);
            }
            else
                prevY_1 = y;
        }
    }

    calibrer->m_robots[0]->GotoXY(0.3, 0.5, 80, true);
    calibrer->m_robots[1]->GotoXY(-0.3, -0.5, 80, true);
    usleep_cal(7e6);

    calibrer->m_robots[0]->GotoXY(10, (3*yMax + yMin) / 4, 80, false);
    calibrer->m_robots[1]->GotoXY(-10, (3*yMin + yMax) / 4, 80, false);
    usleep_cal(2e6);

    watch0 = true; watch1 = true;
    double prevX_0 = 0.3, prevX_1 = -0.3;
    while (watch1 || watch0)
    {
        usleep_cal(1e6);

        if (watch0)
        {
            double x = calibrer->m_robots[0]->GetPos().GetX();
            if (x <= prevX_0)
            {
                xMax = prevX_0;
                watch0 = false;
                calibrer->m_robots[0]->GotoXY(0, 0.2);
            }
            else
                prevX_0 = x;
        }

        if (watch1)
        {
            double x = calibrer->m_robots[1]->GetPos().GetX();
            if (x >= prevX_1)
            {
                xMin = prevX_1;
                watch1 = false;
                calibrer->m_robots[1]->GotoXY(0, -0.2);
            }
            else
                prevX_1 = x;
        }
    }
	
    if (!calibrer->m_stopCalibrating)
    {
        calibrer->m_ty = -(yMax + yMin) / 2;
        calibrer->m_tx = -(xMax + xMin) / 2;
        calibrer->m_cosTheta = 1; calibrer->m_sinTheta = 0;
        calibrer->m_ky = 2 / (yMax - yMin);
        calibrer->m_kx = 2 / (xMax - xMin);
    }
	
    exit_cal();
}
