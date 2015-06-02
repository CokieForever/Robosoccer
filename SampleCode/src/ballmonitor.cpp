#include "ballmonitor.h"

#define NB_POSTIME 10

static RawBall *mainBall = NULL;
static bool stopBallMonitoring = false;
static bool ballMonitoring = false;
static pthread_mutex_t ballMonitoringMtx;
static pthread_t ballMonitoringThread = NULL;
static bool ballFollowing = false;
static pthread_t ballFollowingThread;
static bool stopBallFollowing = false;
static PosTime ballPosTime[NB_POSTIME];
static int ballPosTimeInd = 0;
static int nbBallPosTime = 0;


static void ResetPosTimeList();
static bool ComputeLinearRegression(double *a, double *b, int precision = 2);
static void* BallMonitoringFn(void *data);
static void* BallFollowingFn(void *data);


bool StartBallMonitoring(RawBall *ball)
{
    if (ballMonitoring)
        return false;
    else
    {
        pthread_mutex_init(&ballMonitoringMtx, NULL);
        mainBall = ball;
        pthread_create(&ballMonitoringThread, NULL, BallMonitoringFn, NULL);
        usleep(0.1e6);
        return true;
    }
}

bool StopBallMonitoring()
{
    if (!ballMonitoring)
        return false;
    else
    {
        stopBallMonitoring = true;
        pthread_join(ballMonitoringThread, NULL);
        pthread_mutex_destroy(&ballMonitoringMtx);
        return true;
    }
}

bool GetBallPosition(Position *pos)
{
    if (!ballMonitoring)
        return false;
    else
    {
        pthread_mutex_lock(&ballMonitoringMtx);
        if (nbBallPosTime < 1)
        {
            pthread_mutex_unlock(&ballMonitoringMtx);
            return false;
        }
        *pos = ballPosTime[ballPosTimeInd].pos;
        pthread_mutex_unlock(&ballMonitoringMtx);
        return true;
    }
}

bool GetBallDirection(Direction *dir)
{
    if (!ballMonitoring)
        return false;
    else
    {
        pthread_mutex_lock(&ballMonitoringMtx);
        if (nbBallPosTime < 2)
        {
            pthread_mutex_unlock(&ballMonitoringMtx);
            return false;
        }

        int i2 = ballPosTimeInd;
        int i1 = (ballPosTimeInd-1 + NB_POSTIME) % NB_POSTIME;
        double t = (ballPosTime[i2].time - ballPosTime[i1].time) / (double)CLOCKS_PER_SEC;
        dir->x = (ballPosTime[i2].pos.GetX() - ballPosTime[i1].pos.GetX()) / t;
        dir->y = (ballPosTime[i2].pos.GetY() - ballPosTime[i1].pos.GetY()) / t;
        pthread_mutex_unlock(&ballMonitoringMtx);

        return true;
    }
}

bool PredictBallPosition(Position *pos, int precision)
{
    if (!ballMonitoring)
        return false;
    else if (!IsBallMoving())
    {
        ResetPosTimeList();
        return false;
    }
    else if (precision > 1 && nbBallPosTime < std::min(NB_POSTIME, precision))
        return false;
    else
    {
        Position ballPos1, ballPos2;

        pthread_mutex_lock(&ballMonitoringMtx);
        ballPos1 = NormalizePosition(ballPosTime[(ballPosTimeInd-1 + NB_POSTIME) % NB_POSTIME].pos);
        ballPos2 = NormalizePosition(ballPosTime[ballPosTimeInd].pos);
        pthread_mutex_unlock(&ballMonitoringMtx);

        double xMax=0.85, xMin=-0.85, yMax=0.85, yMin=-0.85;
        double a, b;

        if ((precision > 1 && !ComputeLinearRegression(&a, &b, precision))|| (precision <= 1 && fabs(ballPos2.GetX() - ballPos1.GetX()) <= 1e-4))
        {
            pos->SetX(ballPos2.GetX());
            pos->SetY(ballPos2.GetY() >= ballPos1.GetY() ? yMax : yMin);
        }
        else
        {
            if (precision <= 1)
            {
                a = (ballPos2.GetY() - ballPos1.GetY()) / (ballPos2.GetX() - ballPos1.GetX());
                b = ballPos2.GetY() - a * ballPos2.GetX();
            }

            if (fabs(a) <= 1e-4)
            {
                pos->SetX(ballPos2.GetX() >= ballPos1.GetX() ? xMax : xMin);
                pos->SetY(ballPos2.GetY());
            }
            else
            {
                if (ballPos2.GetY() >= ballPos1.GetY())
                {
                    if (ballPos2.GetX() >= ballPos1.GetX())
                    {
                        pos->SetX((yMax - b) / a);
                        pos->SetY(yMax);
                        if (pos->GetX() > xMax)
                        {
                            pos->SetX(xMax);
                            pos->SetY(a * xMax + b);
                        }
                    }
                    else
                    {
                        pos->SetX((yMax - b) / a);
                        pos->SetY(yMax);
                        if (pos->GetX() < xMin)
                        {
                            pos->SetX(xMin);
                            pos->SetY(a * xMin + b);
                        }
                    }
                }
                else
                {
                    if (ballPos2.GetX() >= ballPos1.GetX())
                    {
                        pos->SetX((yMin - b) / a);
                        pos->SetY(yMin);
                        if (pos->GetX() > xMax)
                        {
                            pos->SetX(xMax);
                            pos->SetY(a * xMax + b);
                        }
                    }
                    else
                    {
                        pos->SetX((yMin - b) / a);
                        pos->SetY(yMin);
                        if (pos->GetX() < xMin)
                        {
                            pos->SetX(xMin);
                            pos->SetY(a * xMin + b);
                        }
                    }
                }
            }
        }

        *pos = UnnormalizePosition(*pos);
        return true;
    }
}

bool IsBallMoving()
{
    pthread_mutex_lock(&ballMonitoringMtx);
    if (nbBallPosTime < 2)
    {
        pthread_mutex_unlock(&ballMonitoringMtx);
        return false;
    }
    int i2 = ballPosTimeInd;
    int i1 = (ballPosTimeInd-1 + NB_POSTIME) % NB_POSTIME;
    bool moving = ballPosTime[i1].pos != ballPosTime[i2].pos && clock() - ballPosTime[i1].time <= CLOCKS_PER_SEC * 0.100;
    pthread_mutex_unlock(&ballMonitoringMtx);
    return moving;
}

bool StartBallFollowing(RoboControl *robo)
{
    if (ballFollowing)
        return false;
    else
    {
        pthread_create(&ballFollowingThread, NULL, BallFollowingFn, robo);
        usleep(0.1e6);
        return true;
    }
}

bool StopBallFollowing()
{
    if (!ballFollowing)
        return false;
    else
    {
        stopBallFollowing = true;
        pthread_join(ballFollowingThread, NULL);
        return true;
    }
}

bool IsBallFollowingStarted()
{
    return ballFollowing;
}


static bool ComputeLinearRegression(double *a, double *b, int precision)
{
    PosTime tab[NB_POSTIME];
    int n, s;

    pthread_mutex_lock(&ballMonitoringMtx);
    memcpy(tab, ballPosTime, sizeof(PosTime)*NB_POSTIME);
    n = std::min(std::max(precision, 2), nbBallPosTime);
    s = ballPosTimeInd;
    pthread_mutex_unlock(&ballMonitoringMtx);

    double sumXi=0, sumXi2=0, sumYi=0, sumXiYi=0;
    for (int i=0 ; i < n ; i++)
    {
        int j = (s-i + NB_POSTIME) % NB_POSTIME;
        Position pos = NormalizePosition(tab[j].pos);
        double x = pos.GetX();
        double y = pos.GetY();
        sumXi += x;
        sumXi2 += x * x;
        sumYi += y;
        sumXiYi += y * x;
    }

    double k = n * sumXi2 - sumXi * sumXi;
    if (fabs(k) <= 1e-4)
        return false;

    if (b)
        *b = (sumXi2 * sumYi - sumXi * sumXiYi) / k;
    if (a)
        *a = (n * sumXiYi - sumXi * sumYi) / k;

    return true;
}

static void ResetPosTimeList()
{
    pthread_mutex_lock(&ballMonitoringMtx);
    nbBallPosTime = 1;
    pthread_mutex_unlock(&ballMonitoringMtx);
}

static void* BallMonitoringFn(void *data)
{
    Position pos;

    ballMonitoring = true;
    stopBallMonitoring = false;

    pthread_mutex_lock(&ballMonitoringMtx);
    ballPosTime[0].pos = mainBall->GetPos();
    ballPosTime[0].time = clock();
    nbBallPosTime = 1;
    ballPosTimeInd = 0;
    pthread_mutex_unlock(&ballMonitoringMtx);

    while (!stopBallMonitoring)
    {
        while (!stopBallMonitoring && (pos = mainBall->GetPos()).DistanceTo(ballPosTime[ballPosTimeInd].pos) < 0.025)
            usleep(1000);

        if (stopBallMonitoring)
            break;

        pthread_mutex_lock(&ballMonitoringMtx);
        ballPosTimeInd = (ballPosTimeInd + 1) % NB_POSTIME;
        nbBallPosTime = nbBallPosTime < NB_POSTIME ? nbBallPosTime+1 : nbBallPosTime;
        ballPosTime[ballPosTimeInd].pos = pos;
        ballPosTime[ballPosTimeInd].time = clock();
        pthread_mutex_unlock(&ballMonitoringMtx);
    }

    ballMonitoring = false;
    return NULL;
}

static void* BallFollowingFn(void *data)
{
    RoboControl *robot = (RoboControl*)data;

    int robotNum = GetRobotNum(robot);
    bool kickMode = false;
    bool waitMode = true;

    ballFollowing = true;
    stopBallFollowing = false;
    while (!stopBallFollowing)
    {
        if (!kickMode)
        {
            Position ballPos;
            if (PredictBallPosition(&ballPos, 5))
            {
                waitMode = false;
                ProgressiveGoto(robotNum, ballPos);

                Position robotPos = NormalizePosition(robot->GetPos());
                Position realBallPos;
                GetBallPosition(&realBallPos);

                double d1 = NormalizePosition(ballPos).DistanceTo(robotPos);
                double d2 = NormalizePosition(realBallPos).DistanceTo(robotPos);
                if (d1 <= 0.045 && d2 <= 1.25)
                    kickMode = true;
            }
            else if (!waitMode && !IsBallMoving())
                kickMode = true;
        }

        if (kickMode)
        {
            Position ballPos;
            GetBallPosition(&ballPos);
            double d = NormalizePosition(ballPos).DistanceTo(NormalizePosition(robot->GetPos()));
            if (d <= 0.07)
                break;
            else
                ProgressiveKick(robotNum, ballPos);
        }

        usleep(5000);
    }

    ballFollowing = false;
    return NULL;
}



