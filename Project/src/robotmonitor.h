#ifndef ROBOTMONITOR_H
#define ROBOTMONITOR_H

#include <pthread.h>
#include "robo_control.h"
#include "coordinates.h"

class RobotMonitor
{

public:
    typedef struct
    {
        Position target;
        RoboControl *robot;

        pthread_t thread;
        bool isValid;
    } GotoOrder;

    RobotMonitor(CoordinatesCalibrer *coordCalibrer, RoboControl **robots = NULL);

    void SetRobot(RoboControl *robot, int num);
    void SetAllRobots(RoboControl **robots);
    int GetRobotNum(RoboControl *robot);

    void ProgressiveGoto(int robot, Position pos0);
    void ProgressiveKick(int robot, Position pos0);
    int GetSuitedSpeed(double distToTgt);

private:
    static void* RobotWatchingFn(void *data);

    GotoOrder currentOrders[6];
    CoordinatesCalibrer *m_coordCalibrer;

};

#endif // ROBOTMONITOR_H
