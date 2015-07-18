#include "coordinates.h"
#include "geometry.h"


/**
 * @brief Initializing  constructor.
 *
 */
CoordinatesCalibrer::CoordinatesCalibrer()
{
    Init();
}

/**
 * @brief Constructor of the coordinates calibrer based on 4 positions in the field.
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
 * @brief Initializes the parameters for the Coordinates calibrer class
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
 * @brief sets manually the coodinates calibrer based on 4 positions
 *
 * @param a First position
 * @param b Second position
 * @param c Third position
 * @param d Fourth position
 * @return bool
 */
bool CoordinatesCalibrer::SetManualCoordCalibration(Position a, Position b, Position c, Position d)
{
    /*
    ********** A **********
    *                     *
    D          O          B
    *                     *
    ********** C **********
    */

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
 * @brief It normalizes a position
 *
 *
 * @param pos Position to be normalized.
 * @return Position Normalized position.
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
 * @brief It unnormalizes a position.
 *
 * @param pos Position to be unnormalized.
 * @return Position Unnormalized position.
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
 * @brief It normalizes an angle to an angle between -M_PI and M_PI.
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
 * @brief This function starts the coordinates calibration
 *
 * @param robot1 First robot.
 * @param robot2 Second robot.
 * @return bool True if thr coordinates calibration was started. False if not.
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
 * @brief It waits for the end of the coordinates calibration before ending it
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
 * @brief It gets the results of coordinates calibration.
 *
 * @param tx
 * @param ty
 * @param theta
 * @param kx
 * @param ky
 * @return bool
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
 * @brief This function gets the results of coordinates calibration.
 *
 * @param xMax The maximum value os x.
 * @param xMin The minimum value os x.
 * @param yMax The maximum value os y.
 * @param yMin The minimum value os y.
 * @return bool True if the calibration results are got. Flase if not.
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
 * @brief This function realizes the calibration of robots coordinates.
 *
 * @param data Data needed for calibration.
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
