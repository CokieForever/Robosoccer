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


PlayerTwo::PlayerTwo(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer *coordCalib, RawBall *b) : TeamRobot(DBC, deviceNr, coordCalib, b)
{
}

void PlayerTwo::setNextCmd(Interpreter *info)
{
    Interpreter::GameData mode = info->getMode();

    switch(mode.mode)
    {
        case PENALTY:
            m_nextCmd = GO_TO_DEF_POS;
            break;

        case PLAY_ON:
            m_nextCmd = PLAY;
            break;

        case KICK_OFF:
            if (mode.turn == Interpreter::OUR_TURN)
                m_nextCmd = PlayerTwo::KICK_OFF;
            else
                m_nextCmd = GO_TO_DEF_POS;
            break;

        case PAUSE:
            m_nextCmd = STOP;
            break;

        case TIME_OVER:
            m_nextCmd = STOP;
            break;

        default:
            m_nextCmd = GO_TO_DEF_POS;
    }

}

void PlayerTwo::setCmdParam(void)
{
    switch(m_nextCmd)
    {
        default:
            break;
    }
}

void *PlayerTwo::performCmd(void)
{
    switch(m_nextCmd)
    {
        case PlayerTwo::GO_TO_DEF_POS:
            std::cout << "Player1 Perform Go To Default Pos:" <<std::endl;
            GotoXY(m_defaultPos.GetX(), m_defaultPos.GetY());
            break;

        default:
            std::cout << "Player2 Perform Default Command: " <<std::endl;
            GotoXY(m_defaultPos.GetX(), m_defaultPos.GetY());
            break;
    }

    return 0;
}

