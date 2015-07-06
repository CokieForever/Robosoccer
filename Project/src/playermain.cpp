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

/**
 * @brief
 *
 * @param DBC
 * @param deviceNr
 * @param c
 * @param b
 * @param ballPm
 * @param display
 */
PlayerMain::PlayerMain(RTDBConn& DBC, const int deviceNr, const CoordinatesCalibrer *c, RawBall *b, BallMonitor *ballPm, RefereeDisplay *display) : TeamRobot(DBC, deviceNr, c, b, ballPm, display)
{
    memset(m_otherRobots, 0, sizeof(NewRoboControl*) * 5);
}

/**
 * @brief
 *
 * @param info
 */
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

/**
 * @brief
 *
 * @param interpreter
 */
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
            m_otherRobots[0] = interpreter.getGK();
            m_otherRobots[1] = interpreter.getP2();
            m_otherRobots[2] = interpreter.getE1();
            m_otherRobots[3] = interpreter.getE2();
            m_otherRobots[4] = interpreter.getE3();
            break;

        case STOP:
            break;
    }
}

/**
 * @brief
 *
 * @param info
 */
void PlayerMain::performCmd(const Interpreter::GameData& info)
{
    switch(m_nextCmd)
    {
        case KICK_PENALTY:
            KickPenalty((const NewRoboControl**)m_otherRobots);
            break;

        case GO_TO_DEF_POS:
            Log("Player1 Perform Go To Default Pos", DEBUG);
            cruisetoBias(m_defaultPos.GetX(),m_defaultPos.GetY(), 650);
            break;

        case KICK_OFF:
            Log("Player1 Perform Kick Off", INFO);
            KickOff((const NewRoboControl**)m_otherRobots, info.our_side, true);
            break;

        case FOLLOWPATH:
            FollowPath(info);
            break;

        case STOP:
            break;
    }
}

/**
 * @brief
 *
 * @param info
 */
void PlayerMain::AddObstacleForFormation(const Interpreter::GameData& info)
{
    if (info.formation == Interpreter::ATK)
    {
        m_areaObstacle = m_pathFinder.AddRectangle(PathFinder::CreatePoint(-2, 0), PathFinder::CreatePoint(2, 2));
    }
    else if (info.formation == Interpreter::DEF)
    {
        if (info.our_side == LEFT_SIDE)
            m_areaObstacle = m_pathFinder.AddRectangle(PathFinder::CreatePoint(-2, -2), PathFinder::CreatePoint(-0.75, 2));
        else
            m_areaObstacle = m_pathFinder.AddRectangle(PathFinder::CreatePoint(0.75, -2), PathFinder::CreatePoint(2, 2));
    }
    else if (info.formation == Interpreter::MIX)
    {
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

