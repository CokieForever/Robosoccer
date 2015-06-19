#include "ballmonitor.h"

BallMonitor::BallMonitor(CoordinatesCalibrer *coordCalibrer, RobotMonitor *robotMonitor, RawBall *ball)
{
    m_mainBall = ball;
    m_stopBallMonitoring = false;
    m_ballMonitoring = false;
    m_ballMonitoringThread = NULL;
    m_ballFollowing = false;
    m_ballFollowingThread = NULL;
    m_stopBallFollowing = false;
    m_ballPosTimeInd = 0;
    m_nbBallPosTime = 0;
    m_followerRobot = NULL;
    m_coordCalibrer = coordCalibrer;
    m_robotMonitor = robotMonitor;
}

bool BallMonitor::StartMonitoring(RawBall *ball)
{
    if (m_ballMonitoring || !m_coordCalibrer || !m_robotMonitor)
        return false;
    else
    {
        if (ball)
            m_mainBall = ball;
        if (!m_mainBall)
            return false;

        pthread_mutex_init(&m_ballMonitoringMtx, NULL);
        pthread_create(&m_ballMonitoringThread, NULL, BallMonitoringFn, this);
        usleep(0.1e6);
        return true;
    }
}

bool BallMonitor::StopMonitoring()
{
    if (!m_ballMonitoring)
        return false;
    else
    {
        m_stopBallMonitoring = true;
        pthread_join(m_ballMonitoringThread, NULL);
        pthread_mutex_destroy(&m_ballMonitoringMtx);
        return true;
    }
}

bool BallMonitor::GetBallPosition(Position *pos)
{
    if (!m_ballMonitoring)
        return false;
    else
    {
        pthread_mutex_lock(&m_ballMonitoringMtx);
        if (m_nbBallPosTime < 1)
        {
            pthread_mutex_unlock(&m_ballMonitoringMtx);
            return false;
        }
        *pos = m_ballPosTime[m_ballPosTimeInd].pos;
        pthread_mutex_unlock(&m_ballMonitoringMtx);
        return true;
    }
}

bool BallMonitor::GetBallDirection(Direction *dir)
{
    if (!m_ballMonitoring)
        return false;
    else
    {
        pthread_mutex_lock(&m_ballMonitoringMtx);
        if (m_nbBallPosTime < 2)
        {
            pthread_mutex_unlock(&m_ballMonitoringMtx);
            return false;
        }

        int i2 = m_ballPosTimeInd;
        int i1 = (m_ballPosTimeInd-1 + NB_POSTIME) % NB_POSTIME;
        double t = (m_ballPosTime[i2].time - m_ballPosTime[i1].time) / (double)CLOCKS_PER_SEC;
        dir->x = (m_ballPosTime[i2].pos.GetX() - m_ballPosTime[i1].pos.GetX()) / t;
        dir->y = (m_ballPosTime[i2].pos.GetY() - m_ballPosTime[i1].pos.GetY()) / t;
        pthread_mutex_unlock(&m_ballMonitoringMtx);

        return true;
    }
}

bool BallMonitor::PredictBallPosition(Position *pos, int precision)
{
    if (!m_ballMonitoring)
        return false;
    else if (!IsBallMoving())
    {
        ResetPosTimeList();
        return false;
    }
    else if (precision > 1 && m_nbBallPosTime < std::min(NB_POSTIME, precision))
        return false;
    else
    {
        Position ballPos1, ballPos2;

        pthread_mutex_lock(&m_ballMonitoringMtx);
        ballPos1 = m_coordCalibrer->NormalizePosition(m_ballPosTime[(m_ballPosTimeInd-1 + NB_POSTIME) % NB_POSTIME].pos);
        ballPos2 = m_coordCalibrer->NormalizePosition(m_ballPosTime[m_ballPosTimeInd].pos);
        pthread_mutex_unlock(&m_ballMonitoringMtx);

        double xMax=0.85, xMin=-0.85, yMax=0.85, yMin=-0.85;
        double a, b;

        if ((precision > 1 && !ComputeLinearRegression(&a, &b, precision)) || (precision <= 1 && fabs(ballPos2.GetX() - ballPos1.GetX()) <= 1e-4))
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

        *pos = m_coordCalibrer->UnnormalizePosition(*pos);
        return true;
    }
}

bool BallMonitor::IsBallMoving()
{
    pthread_mutex_lock(&m_ballMonitoringMtx);
    if (m_nbBallPosTime < 2)
    {
        pthread_mutex_unlock(&m_ballMonitoringMtx);
        return false;
    }
    int i2 = m_ballPosTimeInd;
    int i1 = (m_ballPosTimeInd-1 + NB_POSTIME) % NB_POSTIME;
    bool moving = m_ballPosTime[i1].pos != m_ballPosTime[i2].pos && clock() - m_ballPosTime[i1].time <= CLOCKS_PER_SEC * 0.100;
    pthread_mutex_unlock(&m_ballMonitoringMtx);
    return moving;
}

bool BallMonitor::StartBallFollowing(RoboControl *robo)
{
    if (m_ballFollowing)
        return false;
    else
    {
        if (!robo)
            return false;
        m_followerRobot = robo;
        pthread_create(&m_ballFollowingThread, NULL, BallFollowingFn, this);
        usleep(0.1e6);
        return true;
    }
}

bool BallMonitor::StopBallFollowing()
{
    if (!m_ballFollowing)
        return false;
    else
    {
        m_stopBallFollowing = true;
        pthread_join(m_ballFollowingThread, NULL);
        return true;
    }
}

bool BallMonitor::IsBallFollowingStarted()
{
    return m_ballFollowing;
}



bool BallMonitor::ComputeLinearRegression(double *a, double *b, int precision)
{
    PosTime tab[NB_POSTIME];
    int n, s;

    pthread_mutex_lock(&m_ballMonitoringMtx);
    memcpy(tab, m_ballPosTime, sizeof(PosTime)*NB_POSTIME);
    n = std::min(std::max(precision, 2), m_nbBallPosTime);
    s = m_ballPosTimeInd;
    pthread_mutex_unlock(&m_ballMonitoringMtx);

    double sumXi=0, sumXi2=0, sumYi=0, sumXiYi=0;
    for (int i=0 ; i < n ; i++)
    {
        int j = (s-i + NB_POSTIME) % NB_POSTIME;
        Position pos = m_coordCalibrer->NormalizePosition(tab[j].pos);
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

void BallMonitor::ResetPosTimeList()
{
    pthread_mutex_lock(&m_ballMonitoringMtx);
    m_nbBallPosTime = 1;
    pthread_mutex_unlock(&m_ballMonitoringMtx);
}

void* BallMonitor::BallMonitoringFn(void *data)
{
    BallMonitor *monitor = (BallMonitor*)data;
    Position pos;

    monitor->m_ballMonitoring = true;
    monitor->m_stopBallMonitoring = false;

    pthread_mutex_lock(&monitor->m_ballMonitoringMtx);
    monitor->m_ballPosTime[0].pos = monitor->m_mainBall->GetPos();
    monitor->m_ballPosTime[0].time = clock();
    monitor->m_nbBallPosTime = 1;
    monitor->m_ballPosTimeInd = 0;
    pthread_mutex_unlock(&monitor->m_ballMonitoringMtx);

    while (!monitor->m_stopBallMonitoring)
    {
        while (!monitor->m_stopBallMonitoring && (pos = monitor->m_mainBall->GetPos()).DistanceTo(monitor->m_ballPosTime[monitor->m_ballPosTimeInd].pos) < 0.025)
            usleep(1000);

        if (monitor->m_stopBallMonitoring)
            break;

        pthread_mutex_lock(&monitor->m_ballMonitoringMtx);
        monitor->m_ballPosTimeInd = (monitor->m_ballPosTimeInd + 1) % NB_POSTIME;
        monitor->m_nbBallPosTime = monitor->m_nbBallPosTime < NB_POSTIME ? monitor->m_nbBallPosTime+1 : monitor->m_nbBallPosTime;
        monitor->m_ballPosTime[monitor->m_ballPosTimeInd].pos = pos;
        monitor->m_ballPosTime[monitor->m_ballPosTimeInd].time = clock();
        pthread_mutex_unlock(&monitor->m_ballMonitoringMtx);
    }

    monitor->m_ballMonitoring = false;
    return NULL;
}

void* BallMonitor::BallFollowingFn(void *data)
{
    BallMonitor *monitor = (BallMonitor*)data;
    RoboControl *robot = monitor->m_followerRobot;

    int robotNum = monitor->m_robotMonitor->GetRobotNum(robot);
    bool kickMode = false;
    bool waitMode = true;

    monitor->m_ballFollowing = true;
    monitor->m_stopBallFollowing = false;
    while (!monitor->m_stopBallFollowing)
    {
        if (!kickMode)
        {
            Position ballPos;
            if (monitor->PredictBallPosition(&ballPos, 5))
            {
                waitMode = false;
                monitor->m_robotMonitor->ProgressiveGoto(robotNum, ballPos);

                Position robotPos = monitor->m_coordCalibrer->NormalizePosition(robot->GetPos());
                Position realBallPos;
                monitor->GetBallPosition(&realBallPos);

                double d1 = monitor->m_coordCalibrer->NormalizePosition(ballPos).DistanceTo(robotPos);
                double d2 = monitor->m_coordCalibrer->NormalizePosition(realBallPos).DistanceTo(robotPos);
                if (d1 <= 0.045 && d2 <= 1.25)
                    kickMode = true;
            }
            else if (!waitMode && !monitor->IsBallMoving())
                kickMode = true;
        }

        if (kickMode)
        {
            Position ballPos;
            monitor->GetBallPosition(&ballPos);
            double d = monitor->m_coordCalibrer->NormalizePosition(ballPos).DistanceTo(monitor->m_coordCalibrer->NormalizePosition(robot->GetPos()));
            if (d <= 0.07)
                break;
            else
                monitor->m_robotMonitor->ProgressiveKick(robotNum, ballPos);
        }

        usleep(5000);
    }

    monitor->m_ballFollowing = false;
    return NULL;
}



