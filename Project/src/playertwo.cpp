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
#include "robo_control.h"
#include "playertwo.h"


PlayerTwo::PlayerTwo(RoboControl *x, RawBall *b)
{
    robot = x;
    ball  = b;
}

RoboControl* PlayerTwo::getRobot() const
{
    return robot;
}

Position PlayerTwo::getDefaultPosition() const
{
    return defaultPos;
}

void PlayerTwo::setDefaultPositionX(double x)
{
    defaultPos.SetX(x);
}

void PlayerTwo::setDefaultPositionY(double y)
{
    defaultPos.SetY(y);
}

void PlayerTwo::setDefaultPosition(Position pos)
{
    defaultPos = pos;
}

void PlayerTwo::setNextCmd(void *s)
{
    Interpreter* info = (Interpreter*)s;
    Interpreter::GameData mode = info->getMode();

    switch(mode.mode)
    {
        case PENALTY:
            nextCmd = GO_TO_DEF_POS;
            break;

        case PLAY_ON:
            nextCmd = PLAY;
            break;

        case KICK_OFF:
            if (mode.turn == Interpreter::OUR_TURN)
                nextCmd = PlayerTwo::KICK_OFF;
            else
                nextCmd = GO_TO_DEF_POS;
            break;

        case PAUSE:
            nextCmd = STOP;
            break;

        case TIME_OVER:
            nextCmd = STOP;
            break;

        default:
            nextCmd = GO_TO_DEF_POS;
    }

}

void PlayerTwo::setCmdParam()
{
    switch(nextCmd)
    {
        default:
            break;
    }
}

void *PlayerTwo::performCmd()
{
    switch(nextCmd)
    {
        case PlayerTwo::GO_TO_DEF_POS:
            std::cout << "Player1 Perform Go To Default Pos:" <<std::endl;
            robot->GotoXY(defaultPos.GetX(),defaultPos.GetY());
            break;

        default:
            std::cout << "Player2 Perform Default Command: " <<std::endl;
            robot->GotoXY(defaultPos.GetX(),defaultPos.GetY());
            break;
    }

    return 0;
}

void *PlayerTwo::performCmd_helper(void *context)
{
    return ((PlayerTwo*)context)->performCmd();
}



