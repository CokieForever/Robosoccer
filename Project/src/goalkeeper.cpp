/*
 * interpreter.cpp
 *
 *  Created on: Apr 26, 2015
 *      Author: dean
 */

#include "goalkeeper.h"
#include "interpreter.h"


Goalkeeper::Goalkeeper(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer *coordCalib, RawBall *b) : TeamRobot(DBC, deviceNr, coordCalib, b)
{
}

void Goalkeeper::setNextCmd(const Interpreter::GameData& info)
{
    switch(info.mode)
    {
        case PENALTY:
            if (info.turn != Interpreter::OUR_TURN)
                m_nextCmd = PREVENT_GOAL;
            else
                m_nextCmd = GO_TO_DEF_POS;
            break;

        case PLAY_ON:
            m_nextCmd = PREVENT_GOAL;
            break;

        case BEFORE_KICK_OFF:
        case BEFORE_PENALTY:
            m_nextCmd = GO_TO_DEF_POS;
            break;

        case REFEREE_INIT:
        case KICK_OFF:
        case PAUSE:
        case TIME_OVER:
            m_nextCmd = STOP;
            break;
    }

}

void Goalkeeper::setCmdParam(const Interpreter& interpreter)
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

            double deltaY = fabs(GetY() - y);

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
            m_defaultPos = interpreter.getGKDefaultPos();
            std::cout << "Goal keeper moving to Position = " << m_defaultPos <<std::endl;
            break;

        case STOP:
            break;
    }
}

void Goalkeeper::performCmd(void)
{

    switch(m_nextCmd)
    {
        case Goalkeeper::PREVENT_GOAL:
            std::cout << "Next command Prevent Goal Position: " << m_preventGoalParam.GetPos()<< std::endl;
            GotoXY(m_defaultPos.GetX(),m_preventGoalParam.GetY());
            break;

        case Goalkeeper::GO_TO_DEF_POS:
            GotoXY(m_defaultPos.GetX(),m_defaultPos.GetY());
            while(GetPos().DistanceTo(m_defaultPos)> 0.1)
            {
                usleep(10000000);
            }
            break;

        case STOP:
            break;
    }
}

void Goalkeeper::AddObstacleForFormation(Interpreter::Strategy formation)
{
    m_areaObstacle = NULL;
}

