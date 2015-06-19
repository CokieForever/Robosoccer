#include "robotmonitor.h"

RobotMonitor::RobotMonitor(CoordinatesCalibrer *coordCalibrer, RoboControl **robots)
{
    m_coordCalibrer = coordCalibrer;

    for (int i=0 ; i < 6 ; i++)
    {
        currentOrders[i].isValid = false;
        currentOrders[i].robot = robots ? robots[i] : NULL;
        currentOrders[i].target = Position();
        currentOrders[i].thread = NULL;
    }
}


void RobotMonitor::SetRobot(RoboControl *robot, int num)
{
    if (num < 6 && num >= 0)
        currentOrders[num].robot = robot;
}

void RobotMonitor::SetAllRobots(RoboControl **robots)
{
    for (int i=0 ; i < 6 ; i++)
        currentOrders[i].robot = robots[i];
}

int RobotMonitor::GetRobotNum(RoboControl *robot)
{
    for (int i=0 ; i < 6 ; i++)
    {
        if (currentOrders[i].robot == robot)
            return i;
    }

    return -1;
}

void RobotMonitor::ProgressiveGoto(int robot, Position pos0)
{
    GotoOrder *order = &(currentOrders[robot]);
    Position pos = m_coordCalibrer->NormalizePosition(pos0);

    if (order->isValid)
    {
        double d1 = pos.DistanceTo(order->target);
        double d2 = pos.DistanceTo(m_coordCalibrer->NormalizePosition(order->robot->GetPos()));

        if (d1 >= 0.25 * d2)
            order->robot->GotoXY(pos0.GetX(), pos0.GetY(), GetSuitedSpeed(d2), false);

        order->target = pos;
    }
    else
    {
        order->target = pos;
        order->isValid = true;
        order->robot->GotoXY(pos0.GetX(), pos0.GetY(), GetSuitedSpeed(pos.DistanceTo(m_coordCalibrer->NormalizePosition(order->robot->GetPos()))), false);
        pthread_create(&(order->thread), NULL, RobotWatchingFn, order);
    }
}

void RobotMonitor::ProgressiveKick(int robot, Position pos0)
{
    GotoOrder *order = &(currentOrders[robot]);
    Position pos = m_coordCalibrer->NormalizePosition(pos0);
    Position robotPos = m_coordCalibrer->NormalizePosition(order->robot->GetPos());

    if (order->isValid)
    {
        double d1 = pos.DistanceTo(order->target);
        double d2 = pos.DistanceTo(robotPos);
        double d3 = robotPos.DistanceTo(order->target);
        double cosAlpha = (d2*d2 + d3*d3 - d1*d1) / (2*d2*d3);

        if (cosAlpha <= 0.995)
            order->robot->GotoXY(pos0.GetX(), pos0.GetY(), std::max(GetSuitedSpeed(d2), 120), false);

        order->target = pos;
    }
    else
    {
        order->target = pos;
        order->isValid = true;
        order->robot->GotoXY(pos0.GetX(), pos0.GetY(), std::max(GetSuitedSpeed(pos.DistanceTo(m_coordCalibrer->NormalizePosition(order->robot->GetPos()))), 120), false);
        pthread_create(&(order->thread), NULL, RobotWatchingFn, order);
    }
}

int RobotMonitor::GetSuitedSpeed(double distToTgt)
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

void* RobotMonitor::RobotWatchingFn(void *data)
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
