/*
 * playermain.cpp
 *
 *
 *  Created on: Mai 07, 2015
 *      Author: chiraz
 */

#include "interpreter.h"
#include <iostream>
#include "share.h"
#include "kogmo_rtdb.hxx"
#include <queue>
#include "playermain.h"
#include "refereedisplay.h"
#include "goalkeeper.h"
#include "playertwo.h"

using namespace std;

PlayerMain::PlayerMain(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer *c, RawBall *b, RefereeDisplay *display) : TeamRobot(DBC, deviceNr, c, b, display)
{
}

void PlayerMain::setNextCmd(const Interpreter::GameData& info)
{
    switch(info.mode)
    {
        case PENALTY:
            if (info.turn == Interpreter::OUR_TURN)
                m_nextCmd = KICK_PENALTY;
            else
                m_nextCmd = STOP;
            break;

        case BEFORE_PENALTY:
            m_nextCmd = GO_TO_DEF_POS;
            m_kickPenaltyParam.action1Performed = 0;
            m_kickPenaltyParam.action2Performed = 0;
            break;

        case PLAY_ON:
            m_nextCmd = FOLLOWPATH;
            break;

        case KICK_OFF:
            if (info.turn == Interpreter::OUR_TURN)
                m_nextCmd = PlayerMain::KICK_OFF;
            else
                m_nextCmd = STOP;
            break;

        case BEFORE_KICK_OFF:
            m_nextCmd = GO_TO_DEF_POS;
            break;

        case REFEREE_INIT:
        case PAUSE:
        case TIME_OVER:
            m_nextCmd = STOP;
            break;
    }
}

void PlayerMain::setCmdParam(const Interpreter& interpreter)
{
    switch(m_nextCmd)
    {

        case KICK_PENALTY:
            m_kickPenaltyParam.ball = m_ball->GetPos();
            m_kickPenaltyParam.pos.SetX(0.5);
            m_kickPenaltyParam.pos.SetY(0.25);
            break;

        case FOLLOWPATH:
            ComputePath(interpreter);
            break;

        case GO_TO_DEF_POS:
            m_defaultPos = interpreter.getP1DefaultPos();
            break;

        case KICK_OFF:
        case STOP:
            break;
    }
}

void PlayerMain::performCmd(void)
{
    cout << "Player 1 next command is:" << m_nextCmd << endl << "1: GO_TO_DEF_POS,KICK_PENALTY 2: KICK_OFF 3: STOP 4: FOLLOWPATH" << endl;

    switch(m_nextCmd)
    {
        case KICK_PENALTY:

            //at first go to an predefined position then kick the ball
            // approach to define multiple commands in one command : define checkpoints in action structure

            if (m_kickPenaltyParam.action1Performed==0)
            {
                cout << "Player1 Perform Kick Penalty: " <<  endl;
                GotoXY(m_kickPenaltyParam.pos.GetX(),m_kickPenaltyParam.pos.GetY());

                while(GetPos().DistanceTo(m_kickPenaltyParam.pos)>0.05)
                {
                    usleep(100000);
                }
                m_kickPenaltyParam.action1Performed=1;
            }

            if (m_kickPenaltyParam.action1Performed==1 && m_kickPenaltyParam.action2Performed==0)
            {
                GotoXY(m_kickPenaltyParam.ball.GetX(),m_kickPenaltyParam.ball.GetY());
                m_kickPenaltyParam.action2Performed=1;
            }
            break;

        case GO_TO_DEF_POS:
            cout << "Player1 Perform Go To Default Pos:" <<endl;
            GotoXY(m_defaultPos.GetX(),m_defaultPos.GetY());
            break;

        case KICK_OFF:
            cout << "Player1 Perform Kick Off:" << endl;
            GotoXY(m_ball->GetX(),m_ball->GetY());
            break;

        case FOLLOWPATH:
            FollowPath();
            break;

        case STOP:
            break;
    }
}

void PlayerMain::AddObstacleForFormation(Interpreter::Strategy formation)
{
    if (formation == Interpreter::ATK)
        m_areaObstacle = m_pathFinder.AddRectangle(PathFinder::CreatePoint(-2, 0), PathFinder::CreatePoint(2, 2));
    else if (formation == Interpreter::DEF)
        m_areaObstacle = m_pathFinder.AddRectangle(PathFinder::CreatePoint(-2, -2), PathFinder::CreatePoint(-0.75, 2));
    else if (formation == Interpreter::MIX)
        m_areaObstacle = m_pathFinder.AddRectangle(PathFinder::CreatePoint(0, -2), PathFinder::CreatePoint(2, 2));
    else
        m_areaObstacle = NULL;   //Should never happen
}
