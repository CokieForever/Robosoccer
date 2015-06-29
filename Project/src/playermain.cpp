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
#include "log.h"

using namespace std;

PlayerMain::PlayerMain(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer *c, RawBall *b, BallMonitor *ballPm, RefereeDisplay *display) : TeamRobot(DBC, deviceNr, c, b, ballPm, display)
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
                m_nextCmd = KICK_OFF;
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
        case FOLLOWPATH:
            ComputePath(interpreter);

        case GO_TO_DEF_POS:
            m_defaultPos = interpreter.getP1DefaultPos();
            break;

        case KICK_PENALTY:
        case KICK_OFF:
        case STOP:
            break;
    }
}

void PlayerMain::performCmd(const Interpreter::GameData& info)
{
    switch(m_nextCmd)
    {
        case KICK_PENALTY:

            //at first go to an predefined position then kick the ball
            // approach to define multiple commands in one command : define checkpoints in action structure

            if (m_kickPenaltyParam.action1Performed == 0)
            {
                Log("Player1 Perform Kick Penalty", INFO);
                GotoXY(m_kickPenaltyParam.pos.GetX(), m_kickPenaltyParam.pos.GetY());

                while (GetPos().DistanceTo(m_kickPenaltyParam.pos) > 0.05)
                {
                    usleep(100000);
                }
                m_kickPenaltyParam.action1Performed = 1;
            }

            if (m_kickPenaltyParam.action1Performed == 1 && m_kickPenaltyParam.action2Performed == 0)
            {
                GotoXY(m_kickPenaltyParam.ball.GetX(), m_kickPenaltyParam.ball.GetY());
                m_kickPenaltyParam.action2Performed = 1;
            }
            break;

        case GO_TO_DEF_POS:
            Log("Player1 Perform Go To Default Pos", DEBUG);
            cruisetoBias(m_defaultPos.GetX(),m_defaultPos.GetY(), 650, -10, 30);
            break;

        case KICK_OFF:
            Log("Player1 Perform Kick Off", INFO);
            cruisetoBias(m_ball->GetX(),m_ball->GetY(), 650, -10, 30);
            break;

        case FOLLOWPATH:
            FollowPath(info);
            break;

        case STOP:
            break;
    }
}

void PlayerMain::AddObstacleForFormation(const Interpreter::GameData& info)
{
    if (info.formation == Interpreter::ATK)
    {
        Log("ATTAAAAAAACK", INFO);
        m_areaObstacle = m_pathFinder.AddRectangle(PathFinder::CreatePoint(-2, 0), PathFinder::CreatePoint(2, 2));
    }
    else if (info.formation == Interpreter::DEF)
    {
        Log("DEFEEEEEEENSE", INFO);
        if (info.our_side == LEFT_SIDE)
            m_areaObstacle = m_pathFinder.AddRectangle(PathFinder::CreatePoint(-2, -2), PathFinder::CreatePoint(-0.75, 2));
        else
            m_areaObstacle = m_pathFinder.AddRectangle(PathFinder::CreatePoint(0.75, -2), PathFinder::CreatePoint(2, 2));
    }
    else if (info.formation == Interpreter::MIX)
    {
        Log("MIIIIIIIX", INFO);
        if (info.our_side == LEFT_SIDE)
            m_areaObstacle = m_pathFinder.AddRectangle(PathFinder::CreatePoint(-2, -2), PathFinder::CreatePoint(0, 2));
        else
            m_areaObstacle = m_pathFinder.AddRectangle(PathFinder::CreatePoint(0, -2), PathFinder::CreatePoint(2, 2));
    }
    else
        m_areaObstacle = NULL;   //Should never happen
}


//Player main defend the goal corners
//could be used in Defend Mode for P2
/*void defend_p2(void)
{
    static int counter = 0;
    double y;
    if (counter >= 10)        //Counter for Cruisetobias function
    {
      m_ballpt->GetBallPosition(&m_defendp2);
      y = defendp2.GetY();

      //define Goal borders
      if (y > 0.35)
      {
        y = 0.5;
        m_defendp2.SetX(-1.4);
      }

      else if (y < -0.25)
      {
        y = -0.5;
        m_defendp2.SetX(-1.4);
      }

      else if (y < 0.3 && 0 < y)
      {
        y = 0.5;
        m_defendp2.SetX(-0.5);
      }
      else if (-0.2 < y && y < 0)
      {
        y = -0.5;
        m_defendp2.SetX(-0.5);
      }
      m_defendp2.SetY(y);
      counter = 0;
    }
    counter++;
}*/

