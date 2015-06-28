/*
 * PlayerTwo.cpp
 *
 *
 *  Created on: Mai 07, 2015
 *      Author: chiraz
 */
#include "interpreter.h"
#include <iostream>
#include "share.h"
#include "kogmo_rtdb.hxx"
#include "playertwo.h"


PlayerTwo::PlayerTwo(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer *coordCalib, RawBall *b, BallMonitor *ballPm, RefereeDisplay *display) : TeamRobot(DBC, deviceNr, coordCalib, b, ballPm, display)
{
}

void PlayerTwo::setNextCmd(const Interpreter::GameData& info)
{
    switch(info.mode)
    {
        case PLAY_ON:
            if (mode.formation == Interpreter::DEF)
                m_nextCmd = DEFENSE;
            else
                m_nextCmd = FOLLOWPATH;
            break;

        case BEFORE_KICK_OFF:
        case BEFORE_PENALTY:
            m_nextCmd = GO_TO_DEF_POS;
            break;

        case PENALTY:
        case REFEREE_INIT:
        case KICK_OFF:
        case PAUSE:
        case TIME_OVER:
            m_nextCmd = STOP;
            break;
    }

}

void PlayerTwo::setCmdParam(const Interpreter& interpreter)
{
    switch(m_nextCmd)
    {
        case GO_TO_DEF_POS:
            m_defaultPos = interpreter.getP2DefaultPos();
            break;

        case FOLLOWPATH:
            ComputePath(interpreter);
            break;

        case STOP:
            break;
            
        //PlayerMain in Defense Mode follows y-coordinates of ball
        case DEFENSE:
        {
            static int counter = 0;
            double y;

            if (counter >= 10)
            {
                m_ballpm->GetBallPosition(&m_defendpm);
                y = m_defendpm.GetY();
                //define Goal borders
                if (y > 0.30)
                    y = 0.30;
                else if (y < -0.25)
                    y = -0.25;
                m_defendpm.SetY(y);
                m_defendpm.SetX(-1.1);

                counter = 0;
            }
            
            counter++;
            break;
        }
    }
}

void PlayerTwo::performCmd(void)
{
    switch(m_nextCmd)
    {
        case GO_TO_DEF_POS:
            //std::cout << "Player2 Perform Go To Default Pos:" <<std::endl;
            GotoXY(m_defaultPos.GetX(), m_defaultPos.GetY());
            break;

        case FOLLOWPATH:
            FollowPath();
            break;

        case STOP:
            break;
			
        case DEFENSE:
            cruisetoBias(m_defendpm.GetX(), m_defendpm.GetY(), 650, -10, 30);
            break;
    }
}

void PlayerTwo::AddObstacleForFormation(Interpreter::Strategy formation)
{
    if (formation == Interpreter::ATK)
        m_areaObstacle = m_pathFinder.AddRectangle(PathFinder::CreatePoint(-2, -2), PathFinder::CreatePoint(2, 0));
    else if (formation == Interpreter::DEF)
        m_areaObstacle = NULL;  //Behavior different in this mode
    else if (formation == Interpreter::MIX)
        m_areaObstacle = m_pathFinder.AddRectangle(PathFinder::CreatePoint(-2, -2), PathFinder::CreatePoint(0, 2));
    else
        m_areaObstacle = NULL;  //Should never happen
}

