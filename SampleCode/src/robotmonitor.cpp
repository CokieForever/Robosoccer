#include "robotmonitor.h"

static GotoOrder currentOrders[6] = {{Position(), NULL, NULL, false},
                                     {Position(), NULL, NULL, false},
                                     {Position(), NULL, NULL, false},
                                     {Position(), NULL, NULL, false},
                                     {Position(), NULL, NULL, false},
                                     {Position(), NULL, NULL, false}};

static void* RobotWatchingFn(void *data);


void SetRobot(RoboControl *robot, int num)
{
    if (num < 6 && num >= 0)
        currentOrders[num].robot = robot;
}

void SetAllRobots(RoboControl **robots)
{
    for (int i=0 ; i < 6 ; i++)
        currentOrders[i].robot = robots[i];
}

int GetRobotNum(RoboControl *robot)
{
    for (int i=0 ; i < 6 ; i++)
    {
        if (currentOrders[i].robot == robot)
            return i;
    }

    return -1;
}

void ProgressiveGoto(int robot, Position pos0)
{
    GotoOrder *order = &(currentOrders[robot]);
    Position pos = NormalizePosition(pos0);

    if (order->isValid)
    {
        double d1 = pos.DistanceTo(order->target);
        double d2 = pos.DistanceTo(NormalizePosition(order->robot->GetPos()));

        if (d1 >= 0.25 * d2)
            order->robot->GotoXY(pos0.GetX(), pos0.GetY(), GetSuitedSpeed(d2), false);

        order->target = pos;
    }
    else
    {
        order->target = pos;
        order->isValid = true;
        order->robot->GotoXY(pos0.GetX(), pos0.GetY(), GetSuitedSpeed(pos.DistanceTo(NormalizePosition(order->robot->GetPos()))), false);
        pthread_create(&(order->thread), NULL, RobotWatchingFn, order);
    }
}

void ProgressiveKick(int robot, Position pos0)
{
    GotoOrder *order = &(currentOrders[robot]);
    Position pos = NormalizePosition(pos0);
    Position robotPos = NormalizePosition(order->robot->GetPos());

    if (order->isValid)
    {
        double d1 = pos.DistanceTo(order->target);
        double d2 = pos.DistanceTo(robotPos);
        double d3 = robotPos.DistanceTo(order->target);
        double cosAlpha = (d2*d2 + d3*d3 - d1*d1) / (2*d2*d3);

        if (cosAlpha <= 0.985)  // = 10°
            order->robot->GotoXY(pos0.GetX(), pos0.GetY(), std::max(GetSuitedSpeed(d2), 120), false);

        order->target = pos;
    }
    else
    {
        order->target = pos;
        order->isValid = true;
        order->robot->GotoXY(pos0.GetX(), pos0.GetY(), std::max(GetSuitedSpeed(pos.DistanceTo(NormalizePosition(order->robot->GetPos()))), 120), false);
        pthread_create(&(order->thread), NULL, RobotWatchingFn, order);
    }
}

int GetSuitedSpeed(double distToTgt)
{   
    if (distToTgt <= 0.15)
        return 60;
    else if (distToTgt <= 0.5)
        return 90;
    else if (distToTgt <= 1.25)
        return 130;
    else
        return 250;
}

static void* RobotWatchingFn(void *data)
{
    GotoOrder *order = (GotoOrder*)data;
    order->isValid = true;

    while (order->robot->GetSpeedLeft() <= 0 && order->robot->GetSpeedRight() <= 0)
        usleep(10000);

    while (order->robot->GetSpeedLeft() > 0 || order->robot->GetSpeedRight() > 0)
        usleep(10000);

    order->isValid = false;
    return NULL;
}
