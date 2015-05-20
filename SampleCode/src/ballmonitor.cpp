#include "ballmonitor.h"


static bool stopBallMonitoring = false;
static bool ballMonitoring = false;
static PosTime ballPosTime1 = {Position(0,0), 0}, ballPosTime2 = {Position(0,0), 0};
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
        double t = (ballPosTime2.time - ballPosTime1.time) / (double)CLOCKS_PER_SEC;
        dir->x = (ballPosTime2.pos.GetX() - ballPosTime1.pos.GetX()) / t;
        dir->y = (ballPosTime2.pos.GetY() - ballPosTime1.pos.GetY()) / t;
        pthread_mutex_unlock(&ballMonitoringMtx);
        return true;
    }
}

bool PredictBallPosition(Position *pos)
{
    if (!ballMonitoring || !IsBallMoving())
        return false;
    else
    {
        pthread_mutex_lock(&ballMonitoringMtx);
        Position ballPos1 = NormalizePosition(ballPosTime1.pos);
        Position ballPos2 = NormalizePosition(ballPosTime2.pos);
        pthread_mutex_unlock(&ballMonitoringMtx);

        double a = (ballPos2.GetY() - ballPos1.GetY()) / (ballPos2.GetX() - ballPos1.GetX());
        double b = ballPos2.GetY() - a * ballPos2.GetX();
        double xMax=0.9, xMin=-0.9, yMax=0.9, yMin=-0.9;

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

        *pos = UnnormalizePosition(*pos);
        return true;
    }
}

bool IsBallMoving()
{
    return ballPosTime1.pos != ballPosTime2.pos && clock() - ballPosTime1.time <= CLOCKS_PER_SEC * 0.100;
}



static void* BallMonitoringFn(void *data)
{
    RawBall *ball = (RawBall*)data;
    Position pos;

    ballMonitoring = true;
    stopBallMonitoring = false;

    pthread_mutex_lock(&ballMonitoringMtx);
    ballPosTime2.pos = ball->GetPos();
    ballPosTime2.time = clock();
    pthread_mutex_unlock(&ballMonitoringMtx);

    while (!stopBallMonitoring)
    {
        while ((pos = ball->GetPos()).DistanceTo(ballPosTime2.pos) < 0.025)
            usleep(1000);

        pthread_mutex_lock(&ballMonitoringMtx);
        ballPosTime1 = ballPosTime2;
        ballPosTime2.pos = pos;
        ballPosTime2.time = clock();
        pthread_mutex_unlock(&ballMonitoringMtx);

        cout << "Ball at " << pos << " (" << NormalizePosition(pos) << ")" << endl;
    }

    ballMonitoring = false;
    return NULL;
}


