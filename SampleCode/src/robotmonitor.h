#ifndef ROBOTMONITOR_H
#define ROBOTMONITOR_H

#include <pthread.h>
#include "robo_control.h"
#include "coordinates.h"

typedef struct
{
    Position target;
    RoboControl *robot;

    pthread_t thread;
    bool isValid;
} GotoOrder;

void SetRobot(RoboControl *robot, int num);
void SetAllRobots(RoboControl **robots);
int GetRobotNum(RoboControl *robot);

void ProgressiveGoto(int robot, Position pos0);
int GetSuitedSpeed(double distToTgt);


#endif // ROBOTMONITOR_H
