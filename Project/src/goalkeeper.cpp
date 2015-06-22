/*
 * interpreter.cpp
 *
 *  Created on: Apr 26, 2015
 *      Author: dean
 */

#include "goalkeeper.h"
#include "interpreter.h"


Goalkeeper::Goalkeeper(RoboControl *x,RawBall *b)
{
    m_robot = x;
    m_ball  = b;
}

RoboControl* Goalkeeper::getRobot()
{
    return m_robot;
}

Position Goalkeeper::getDefaultPosition()
{
    return m_defaultPos;
}

void Goalkeeper::setDefaultPositionX(double x)
{
    m_defaultPos.SetX(x);
}

void Goalkeeper::setDefaultPositionY(double y)
{
    m_defaultPos.SetY(y);
}

void Goalkeeper::setDefaultPosition(Position pos)
{
    m_defaultPos = pos;
}


void Goalkeeper::setNextCmd(void *s)
{

    Interpreter* info = (Interpreter*)s;
    Interpreter::GameData mode = info->getMode();

    switch(mode.mode)
    {
        case PENALTY:
            if (mode.turn != Interpreter::OUR_TURN)
                m_nextCmd = PREVENT_GOAL;
            else
                m_nextCmd = GO_TO_DEF_POS;
            break;

        case PLAY_ON:
            m_nextCmd = PREVENT_GOAL;
            break;

        default:
            m_nextCmd = GO_TO_DEF_POS;
            break;
    }

}

void Goalkeeper::setCmdParam()
{
    switch(m_nextCmd)
    {
        case PREVENT_GOAL:
        {
            Position ballPos = m_ball->GetPos();
            double y = ballPos.GetY();

            if (y > 0.15)
            y = 0.15;
            else if (y < -0.15)
            y = -0.15;

            double deltaY = fabs(m_robot->GetY() - y);

            if (deltaY >= 0.05)
            {
                cout << "Goal keeper moving to y = " << y << std::endl;
                m_preventGoalParam.SetX(m_defaultPos.GetX());
                m_preventGoalParam.SetY(y);
            }
            else
            {
                m_preventGoalParam.SetX(m_defaultPos.GetX());
                m_preventGoalParam.SetY(m_defaultPos.GetY());
            }

            break;
        }

        case GO_TO_DEF_POS:
            std::cout << "Goal keeper moving to Position = " << m_defaultPos <<std::endl;
            break;

        default :
            break;
    }
}



void *Goalkeeper::performCmd(void)
{

    switch(m_nextCmd)
    {
        case Goalkeeper::PREVENT_GOAL:
            std::cout << "Next command Prevent Goal Position: " << m_preventGoalParam.GetPos()<< std::endl;
            m_robot->GotoXY(m_defaultPos.GetX(),m_preventGoalParam.GetY());
            break;

        case Goalkeeper::GO_TO_DEF_POS:
            m_robot->GotoXY(m_defaultPos.GetX(),m_defaultPos.GetY());
            while(m_robot->GetPos().DistanceTo(m_defaultPos)> 0.1)
            {
                usleep(10000000);
            }
            break;

        default:
            break;
    }

    return 0;
}

void *Goalkeeper::performCmd_helper(void *context)
{
    return ((Goalkeeper*)context)->performCmd();
}





