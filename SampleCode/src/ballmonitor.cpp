#include "ballmonitor.h"


static bool stopBallMonitoring = false;
static bool ballMonitoring = false;
static PosTime ballPosTime1, ballPosTime2;
static pthread_mutex_t ballMonitoringMtx;
static pthread_t ballMonitoringThread = NULL;

static void* BallMonitoringFn(void *data);


bool StartBallMonitoring(RawBall *ball)
{
    if (ballMonitoring)
        return false;
    else
    {
        pthread_mutex_init(&ballMonitoringMtx, NULL);
        pthread_create(&ballMonitoringThread, NULL, BallMonitoringFn, ball);
        ballMonitoring = true;
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
        ballMonitoring = false;
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
        *pos = ballPosTime2.pos;
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
        double t = (ballPosTime2.time - ballPosTime1.time) / CLOCKS_PER_SEC;
        dir->x = (ballPosTime2.pos.GetX() - ballPosTime1.pos.GetX()) / t;
        dir->y = (ballPosTime2.pos.GetY() - ballPosTime1.pos.GetY()) / t;
        pthread_mutex_unlock(&ballMonitoringMtx);
        return true;
    }
}

bool PredictBallPosition(Position *pos)
{
    if (!ballMonitoring)
        return false;
    else
    {
        pthread_mutex_lock(&ballMonitoringMtx);

        double a = (ballPosTime2.pos.GetY() - ballPosTime1.pos.GetY()) / (ballPosTime2.pos.GetX() - ballPosTime1.pos.GetX());
        double b = ballPosTime2.pos.GetY() - a * ballPosTime2.pos.GetX();

        double xMax = 1, xMin = -1, yMax = 1, yMin = -1;

        if (ballPosTime2.pos.GetY() >= ballPosTime1.pos.GetY())
        {
            if (ballPosTime2.pos.GetX() >= ballPosTime1.pos.GetX())
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
            if (ballPosTime2.pos.GetX() >= ballPosTime1.pos.GetX())
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

        pthread_mutex_unlock(&ballMonitoringMtx);
        return true;
    }
}



static void* BallMonitoringFn(void *data)
{
    RawBall *ball = (RawBall*)data;
    Position pos;

    stopBallMonitoring = false;

    pthread_mutex_lock(&ballMonitoringMtx);
    ballPosTime2.pos = ball->GetPos();
    ballPosTime2.time = clock();
    pthread_mutex_unlock(&ballMonitoringMtx);

    while (!stopBallMonitoring)
    {
        while ((pos = ball->GetPos()) == ballPosTime2.pos)
            usleep(1000);

        pthread_mutex_lock(&ballMonitoringMtx);
        ballPosTime1 = ballPosTime2;
        ballPosTime2.pos = pos;
        ballPosTime2.time = clock();
        pthread_mutex_unlock(&ballMonitoringMtx);
    }

    return NULL;
}


