#include "coordinates.h"

static int xMax_c, yMax_c, xMin_c, yMin_c;
static bool isCalibrating = false;
static bool calibrationSuccessful = false;
static bool stopCalibrating = false;
static pthread_t calibrationThread;

static void* CalibrationFn(void *data);

bool StartCoordCalibration(RoboControl *robot1, RoboControl *robot2)
{
    RoboControl **array = new RoboControl*[2];
    array[0] = robot1;
    array[1] = robot2;
    pthread_create(&calibrationThread, NULL, CalibrationFn, array);

    return true;
}

bool StopCoordCalibration()
{
    if (!isCalibrating)
        return false;
    stopCalibrating = true;
    pthread_join(calibrationThread, NULL);
    return true;
}

bool WaitForCoordCalibrationEnd()
{
    if (!isCalibrating)
        return false;
    pthread_join(calibrationThread, NULL);
    return true;
}

bool GetCoordCalibrationResults(double *xMax, double *yMax, double *xMin, double *yMin)
{
    if (!calibrationSuccessful)
        return false;

    if (xMax)
        *xMax = xMax_c;
    if (yMax)
        *yMax = yMax_c;
    if (xMin)
        *xMin = xMin_c;
    if (yMin)
        *yMin = yMin_c;

    return true;
}

#define exit_cal() do {calibrationSuccessful = !stopCalibrating; isCalibrating = false; delete robots; return NULL;} while (0)
#define usleep_cal(t) do { if (stopCalibrating) exit_cal(); usleep(t); if (stopCalibrating) exit_cal(); } while (0)
static void* CalibrationFn(void *data)
{
    RoboControl **robots = (RoboControl**)data;
    isCalibrating = true;
    stopCalibrating = false;

    robots[0]->GotoXY(0, 0.1);
    robots[1]->GotoXY(0, -0.1);
    usleep_cal(5e6);

    robots[0]->GotoXY(0, 10, 80, false);
    robots[1]->GotoXY(0, -10, 80, false);
    usleep_cal(0.5e6);

    bool watch0 = true, watch1 = true;
    double prevY_0 = 0.1, prevY_1 = -0.1;
    while (watch1 || watch0)
    {
        usleep(0.5e6);

        if (watch0)
        {
            double y = robots[0]->GetPos().GetY();
            if (y <= prevY_0)
            {
                yMax_c = prevY_0;
                watch0 = false;
            }
        }

        if (watch1)
        {
            double y = robots[1]->GetPos().GetY();
            if (y >= prevY_1)
            {
                yMin_c = prevY_1;
                watch1 = false;
            }
        }
    }

    robots[0]->GotoXY(0, yMax_c / 2);
    robots[1]->GotoXY(0, yMin_c / 2);
    usleep_cal(5e6);

    robots[0]->GotoXY(10, yMax_c / 2, 80, false);
    robots[1]->GotoXY(10, yMin_c / 2, 80, false);
    usleep_cal(0.5e6);

    watch0 = true; watch1 = true;
    double prevX_0 = 0, prevX_1 = 0;
    while (watch1 || watch0)
    {
        usleep_cal(0.5e6);

        if (watch0)
        {
            double x = robots[0]->GetPos().GetX();
            if (x <= prevX_0)
            {
                xMax_c = prevX_0;
                watch0 = false;
            }
        }

        if (watch1)
        {
            double x = robots[1]->GetPos().GetX();
            if (x >= prevX_1)
            {
                xMin_c = prevX_1;
                watch1 = false;
            }
        }
    }

    exit_cal();
}
