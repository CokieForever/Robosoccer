#include "coordinates.h"

static double tx_c = 0, ty_c = 0, cosTheta_c = 1, sinTheta_c = 0, kx_c = 1, ky_c = 1;
static bool isCalibrating = false;
static bool calibrationSuccessful = false;
static bool stopCalibrating = false;
static pthread_t calibrationThread;

static void* CalibrationFn(void *data);

bool SetManualCoordCalibration(Position a, Position b, Position c, Position d)
{
	/*
	********** A **********
	*                     *
	D          O          B
	*                     *
	********** C **********
	*/
	
	tx_c = -(a.GetX() + c.GetX()) / 2;
        ty_c = -(a.GetY() + c.GetY()) / 2;
        double dist = d.DistanceTo(b);
        cosTheta_c = (b.GetX() - d.GetX()) / dist;
        sinTheta_c = (d.GetY() - b.GetY()) / dist;
        kx_c = 2 / dist;
        ky_c = 2 / a.DistanceTo(c);
	
	calibrationSuccessful = true;
	return true;
}

Position NormalizePosition(Position pos)
{
	double x = pos.GetX(), y = pos.GetY();
	
	//Translation
	x += tx_c;
	y += ty_c;
	
	//Rotation
	x = x * cosTheta_c - y * sinTheta_c;
	y = x * sinTheta_c + y * cosTheta_c;
	
	//Dilatation
	x *= kx_c;
	y *= ky_c;
	
	return Position(x, y);
}

Position UnnormalizePosition(Position pos)
{
	double x = pos.GetX(), y = pos.GetY();
	
	//Dilatation
	x /= kx_c;
	y /= ky_c;
	
	//Rotation
	x = x * cosTheta_c + y * sinTheta_c;
	y = -x * sinTheta_c + y * cosTheta_c;
	
	//Translation
	x -= tx_c;
	y -= ty_c;
	
	return Position(x, y);
}

bool StartCoordCalibration(RoboControl *robot1, RoboControl *robot2)
{
    RoboControl **array = new RoboControl*[2];
    array[0] = robot1;
    array[1] = robot2;
    pthread_create(&calibrationThread, NULL, CalibrationFn, array);

    usleep(0.1e6);
    return true;
}

bool WaitForCoordCalibrationEnd(bool stopNow)
{
    if (!isCalibrating)
        return false;
    if (stopNow)
        stopCalibrating = true;
    pthread_join(calibrationThread, NULL);
    return true;
}

bool GetCoordCalibrationResults(double *tx, double *ty, double *theta, double *kx, double *ky)
{
    if (!calibrationSuccessful)
        return false;

    if (tx)
        *tx = tx_c;
    if (ty)
        *ty = ty_c;
    if (theta)
        *theta = asin(sinTheta_c);
    if (kx)
        *kx = kx_c;
	if (ky)
        *ky = ky_c;

    return true;
}

bool GetCoordCalibrationResults(double *xMax, double *xMin, double *yMax, double *yMin)
{
    if (!calibrationSuccessful)
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

#define exit_cal() do {calibrationSuccessful = !stopCalibrating; isCalibrating = false; delete robots; return NULL;} while (0)
#define usleep_cal(t) do { if (stopCalibrating) exit_cal(); usleep(t); if (stopCalibrating) exit_cal(); } while (0)
static void* CalibrationFn(void *data)
{
    RoboControl **robots = (RoboControl**)data;
    double xMax=0, xMin=0, yMax=0, yMin=0;
    isCalibrating = true;
    stopCalibrating = false;

    robots[0]->GotoXY(0, 0.2, 80, true);
    robots[1]->GotoXY(0, -0.2, 80, true);
    usleep_cal(7e6);

    robots[0]->GotoXY(0, 10, 80, false);
    robots[1]->GotoXY(0, -10, 80, false);
    usleep_cal(2e6);

    bool watch0 = true, watch1 = true;
    double prevY_0 = 0.2, prevY_1 = -0.2;
    while (watch1 || watch0)
    {
        usleep(1e6);

        if (watch0)
        {
            double y = robots[0]->GetPos().GetY();
            if (y <= prevY_0)
            {
                yMax = prevY_0;
                watch0 = false;
                robots[0]->GotoXY(0.3, 0.5, 80, true);
            }
            else
                prevY_0 = y;
        }

        if (watch1)
        {
            double y = robots[1]->GetPos().GetY();
            if (y >= prevY_1)
            {
                yMin = prevY_1;
                watch1 = false;
                robots[1]->GotoXY(-0.3, -0.5, 80, true);
            }
            else
                prevY_1 = y;
        }
    }

    robots[0]->GotoXY(0.3, 0.5, 80, true);
    robots[1]->GotoXY(-0.3, -0.5, 80, true);
    usleep_cal(7e6);

    robots[0]->GotoXY(10, (3*yMax + yMin) / 4, 80, false);
    robots[1]->GotoXY(-10, (3*yMin + yMax) / 4, 80, false);
    usleep_cal(2e6);

    watch0 = true; watch1 = true;
    double prevX_0 = 0.3, prevX_1 = -0.3;
    while (watch1 || watch0)
    {
        usleep_cal(1e6);

        if (watch0)
        {
            double x = robots[0]->GetPos().GetX();
            if (x <= prevX_0)
            {
                xMax = prevX_0;
                watch0 = false;
                robots[0]->GotoXY(0, 0.2);
            }
            else
                prevX_0 = x;
        }

        if (watch1)
        {
            double x = robots[1]->GetPos().GetX();
            if (x >= prevX_1)
            {
                xMin = prevX_1;
                watch1 = false;
                robots[1]->GotoXY(0, -0.2);
            }
            else
                prevX_1 = x;
        }
    }
	
    if (!stopCalibrating)
    {
        ty_c = -(yMax + yMin) / 2;
        tx_c = -(xMax + xMin) / 2;
        cosTheta_c = 1; sinTheta_c = 0;
        ky_c = 2 / (yMax - yMin);
        kx_c = 2 / (xMax - xMin);
    }
	
    exit_cal();
}
