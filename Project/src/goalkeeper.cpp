/*
 * interpreter.cpp
 *
 *  Created on: Apr 26, 2015
 *      Author: dean
 */

#include "goalkeeper.h"
#include "interpreter.h"
#include "newrobocontrol.h"


Goalkeeper::Goalkeeper(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer *coordCalib, RawBall *b, BallMonitor *ballPm) : TeamRobot(DBC, deviceNr, coordCalib, b, ballPm)
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
            eSide side = interpreter.getMode().our_side;
            double x = side == LEFT_SIDE ? -1 : +1;

            BallMonitor::Direction dir;
            m_ballPm->GetBallDirection(&dir);

            bool success = false;
            if ((dir.x >= 0) != (side == LEFT_SIDE))
            {
                double a, b;
                if (m_ballPm->PredictBallPosition(&a, &b, 4) && a < PathFinder::INFINI_TY)
                {
                    double y = std::max(-0.25, std::min(0.25, a * x + b));
                    m_preventGoalParam = m_coordCalib->UnnormalizePosition(Position(x,y));
                    success = true;
                }
            }

            if (!success)
            {
                Position ballPos;
                m_ballPm->GetBallPosition(&ballPos);
                ballPos = m_coordCalib->NormalizePosition(ballPos);
                m_preventGoalParam = m_coordCalib->UnnormalizePosition(Position(x, ballPos.GetY()));
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
        case PREVENT_GOAL:
            cruisetoBias(m_preventGoalParam.GetX(), m_preventGoalParam.GetY(), 650, -10, 30);
            break;

        case GO_TO_DEF_POS:
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

