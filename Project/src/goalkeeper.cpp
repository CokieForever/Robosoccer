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
  m_ballgk = ballgk;
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
    static int counter = 0;
    
    switch(m_nextCmd)
    {
        case PREVENT_GOAL:
        {
            m_ballgk->GetBallPosition(&m_predictballgk);
            m_defendgk.SetX(-1.43);
            m_defendgk.SetY(0.036);
            double y;
            double deltaY;
            
            if (counter >= 9)      //counter for cruisetobias(), otherwise too many commands, robot has not enough time to drive
            {
                m_ballgk->PredictBallPosition(&m_predictballgk, 4, -0.99);
                y = m_predictballgk.GetY();
                //Define Goal borders
                if (y > 0.22)
                    y = 0.22;
                else if (y < -0.15)
                    y = -0.15;

                deltaY = fabs(GetY() - y);
                m_preventGoalParam.SetX(m_defendgk.GetX());
                m_preventGoalParam.SetY(y);
                counter = 0;
            }
            
            counter ++;
    default :
      break;
  }
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
            cruisetoBias(m_defendgk.GetX(), m_preventGoalParam.GetY(), 650, -10, 30);
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

