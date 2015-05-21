#include "ballmonitor.h"


static RawBall *mainBall = NULL;
static bool stopBallMonitoring = false;
static bool ballMonitoring = false;
static PosTime ballPosTime1 = {Position(0,0), 0}, ballPosTime2 = {Position(0,0), 0};
static pthread_mutex_t ballMonitoringMtx;
static pthread_t ballMonitoringThread = NULL;
static bool ballFollowing = false;
static pthread_t ballFollowingThread;
static bool stopBallFollowing = false;

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

        double xMax=0.85, xMin=-0.85, yMax=0.85, yMin=-0.85;

        if (fabs(ballPos2.GetX() - ballPos1.GetX()) <= 1e-4)
        {
            pos->SetX(ballPos2.GetX());
            pos->SetY(ballPos2.GetY() >= ballPos1.GetY() ? yMax : yMin);
        }
        else
        {
            double a = (ballPos2.GetY() - ballPos1.GetY()) / (ballPos2.GetX() - ballPos1.GetX());
            double b = ballPos2.GetY() - a * ballPos2.GetX();

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

        *pos = UnnormalizePosition(*pos);
        return true;
    }
}

bool IsBallMoving()
{
    return ballPosTime1.pos != ballPosTime2.pos && clock() - ballPosTime1.time <= CLOCKS_PER_SEC * 0.100;
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


static void* BallMonitoringFn(void *data)
{
    Position pos;

    ballMonitoring = true;
    stopBallMonitoring = false;

    pthread_mutex_lock(&ballMonitoringMtx);
    ballPosTime2.pos = mainBall->GetPos();
    ballPosTime2.time = clock();
    pthread_mutex_unlock(&ballMonitoringMtx);

    while (!stopBallMonitoring)
    {
        while ((pos = mainBall->GetPos()).DistanceTo(ballPosTime2.pos) < 0.025)
            usleep(1000);

        pthread_mutex_lock(&ballMonitoringMtx);
        ballPosTime1 = ballPosTime2;
        ballPosTime2.pos = pos;
        ballPosTime2.time = clock();
        pthread_mutex_unlock(&ballMonitoringMtx);
    }

    ballMonitoring = false;
    return NULL;
}

static void* BallFollowingFn(void *data)
{
    RoboControl *robot = (RoboControl*)data;
    int i = 0, n = 0;
    const int nbMeans = 5;
    Position ballPosTab[nbMeans];

    int robotNum = GetRobotNum(robot);

    ballFollowing = true;
    stopBallFollowing = false;
    while (!stopBallFollowing)
    {
        Position ballPos;
        if (PredictBallPosition(&ballPos))
        {
            ballPosTab[i] = ballPos;
            i = (i+1) % nbMeans;
            n = n < nbMeans ? n+1 : n;

            if (n >= nbMeans)
            {
                Position meanPos = ballPosTab[0];
                for (int j=1 ; j < n ; j++)
                    meanPos += ballPosTab[j];
                meanPos.SetX(meanPos.GetX() / n);
                meanPos.SetY(meanPos.GetY() / n);

                ProgressiveGoto(robotNum, meanPos);

                /*double d = NormalizePosition(ballPos).DistanceTo(NormalizePosition(roboBall->robo->GetPos()));
                if (d <= 0.1)
                {
                    cout << "Kick!" << endl;
                    roboBall->robo->GotoXY(roboBall->ball->GetX(), roboBall->ball->GetY(), 120, false);
                    usleep(7e6);
                    cout << "Ready" << endl;
                }*/
            }
        }
        else
        {
            i = 0;
            n = 0;
        }

        usleep(10000);
    }

    ballFollowing = false;
    return NULL;
}



