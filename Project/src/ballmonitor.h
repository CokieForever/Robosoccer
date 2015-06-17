#ifndef BALLMONITOR_H
#define BALLMONITOR_H

#include <pthread.h>
#include <time.h>
#include "kogmo_rtdb.hxx"
#include "robo_control.h"
#include "referee.h"
#include "coordinates.h"
#include "robotmonitor.h"

class BallMonitor
{

public:

    static const int NB_POSTIME  = 10;

    typedef struct
    {
        double x, y;
    } Direction;

    typedef struct
    {
        Position pos;
        clock_t time;
    } PosTime;

    BallMonitor(CoordinatesCalibrer *coordCalibrer, RobotMonitor *robotMonitor, RawBall *ball = NULL);

    bool StartMonitoring(RawBall *ball = NULL);
    bool StopMonitoring();
    bool GetBallPosition(Position *pos);
    bool GetBallDirection(Direction *dir);
    bool PredictBallPosition(Position *pos, int precision = 0);
    bool IsBallMoving();

    bool StartBallFollowing(RoboControl *robo);
    bool StopBallFollowing();
    bool IsBallFollowingStarted();

private:

    static void* BallMonitoringFn(void *data);
    static void* BallFollowingFn(void *data);

    RawBall *m_mainBall;
    bool m_stopBallMonitoring;
    bool m_ballMonitoring;
    pthread_mutex_t m_ballMonitoringMtx;
    pthread_t m_ballMonitoringThread;
    bool m_ballFollowing;
    pthread_t m_ballFollowingThread;
    bool m_stopBallFollowing ;
    PosTime m_ballPosTime[NB_POSTIME];
    int m_ballPosTimeInd;
    int m_nbBallPosTime;
    RoboControl *m_followerRobot;
    CoordinatesCalibrer *m_coordCalibrer;
    RobotMonitor *m_robotMonitor;

    void ResetPosTimeList();
    bool ComputeLinearRegression(double *a, double *b, int precision = 2);


};

#endif // BALLMONITOR_H
