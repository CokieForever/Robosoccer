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


PlayerTwo::PlayerTwo(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer *coordCalib, RawBall *b, RefereeDisplay *display) : TeamRobot(DBC, deviceNr, coordCalib, b, display)
{
}

void PlayerTwo::setNextCmd(const Interpreter::GameData& info)
{
    switch(info.mode)
    {
        case PLAY_ON:
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

