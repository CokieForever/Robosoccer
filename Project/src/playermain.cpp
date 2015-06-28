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

PlayerMain::PlayerMain(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer *c, RawBall *b, BallMonitor *ballPm, RefereeDisplay *display) : TeamRobot(DBC, deviceNr, c, b, ballPm, display)
{
  m_ballpm = ballpm;
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
      if (mode.formation == 1)
        m_nextCmd = DEFENSE;
      else if (mode.formation == 0)
        m_nextCmd = ATTACK;
      else
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

  //interpreter::Point *pt;
  char c;
  int j, idx_tmp;
  unsigned int interpolate_n = 15;
  Position robo_n, ball_n;

        case FOLLOWPATH:
            ComputePath(interpreter);
      //take n points and interpolate
      if (m_q.size() >= interpolate_n)
      {
        for (unsigned int i = 0; i < interpolate_n; i++)
        {
          idx_tmp = m_q.front();
          m_go_x = m_go_x + Interpreter::DX[idx_tmp];
          m_go_y = m_go_y + Interpreter::DY[idx_tmp];
          m_q.pop();
        }

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

      if (m_kickPenaltyParam.action1Performed == 0)
      {
        cout << "Player1 Perform Kick Penalty: " <<  endl;
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

