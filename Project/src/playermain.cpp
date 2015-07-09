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
#include "newrobocontrol.h"

using namespace std;

PlayerMain::PlayerMain(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer* c, RawBall* b, BallMonitor* ballpm) : TeamRobot(DBC, deviceNr, c, b)
{
  m_ballpm = ballpm;
}

void PlayerMain::setNextCmd(Interpreter* info)
{
  Interpreter::GameData mode = info->getMode();

  switch (mode.mode)
  {
    case PENALTY:
      if (mode.turn == Interpreter::OUR_TURN)
        m_nextCmd = KICK_PENALTY;
      else
        m_nextCmd = GO_TO_DEF_POS;
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
      if (mode.turn == Interpreter::OUR_TURN)
        m_nextCmd = PlayerMain::KICK_OFF;
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
      break;
  }
}

void PlayerMain::setCmdParam(void)
{
  //both needed when it comes to path tracking
  Interpreter::Point A, B;


  //interpreter::Point *pt;
  char c;
  int j, idx_tmp;
  unsigned int interpolate_n = 15;
  Position robo_n, ball_n;

  switch (m_nextCmd)
  {

    case KICK_PENALTY:
      m_kickPenaltyParam.ball = m_ball->GetPos();
      m_kickPenaltyParam.pos.SetX(0.5);
      m_kickPenaltyParam.pos.SetY(0.25);
      break;

    case FOLLOWPATH:
      if (m_q.size() == 0)
      {
        //create queue of move indices

        robo_n = m_coordCalib->NormalizePosition(GetPos());
        ball_n = m_coordCalib->NormalizePosition(m_ball->GetPos());

        A.x = Interpreter::coord2mapX(robo_n.GetX());
        A.y = Interpreter::coord2mapY(robo_n.GetY());
        B.x = Interpreter::coord2mapX(ball_n.GetX());
        B.y = Interpreter::coord2mapY(ball_n.GetY());

        //get string with motion commands
        m_path = Interpreter::pathFind(m_map, A, B);
        Interpreter::showMap(m_map, m_path, A);

        for (unsigned int i = 0 ; i < m_path.length() ; i++)
        {
          c = m_path.at(i);
          j = atoi(&c);
          m_q.push(j);
        }
      }
      m_go_x = 0;
      m_go_y = 0;

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

      }
      else
      {
        while (m_q.size() != 0)
        {
          idx_tmp = m_q.front();
          m_go_x = m_go_x + Interpreter::DX[idx_tmp];
          m_go_y = m_go_y + Interpreter::DY[idx_tmp];
          m_q.pop();
        }
      }
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
    case GO_TO_DEF_POS:
      break;
    case KICK_OFF:
      break;
    case STOP:
      break;
    default:
      break;
  }
}

void* PlayerMain::performCmd(void)
{
  cout << "Player 1 next command is:" << m_nextCmd << endl << "1: GO_TO_DEF_POS,KICK_PENALTY 2: KICK_OFF 3: STOP 4: FOLLOWPATH" << endl;
  Position pos;
  int mapx, mapy;

  switch (m_nextCmd)
  {
    case PlayerMain::KICK_PENALTY:

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

    case PlayerMain::GO_TO_DEF_POS:
      cout << "Player1 Perform Go To Default Pos:" << endl;
      GotoXY(m_defaultPos.GetX(), m_defaultPos.GetY());
      break;
      //           usleep(100000);
      //        }
      //        kickPenaltyParam.action1Performed=1;

    case PlayerMain::KICK_OFF:
      cout << "Player1 Perform Kick Off:" << endl;
      GotoXY(m_ball->GetX(), m_ball->GetY());
      break;

    case PlayerMain::FOLLOWPATH:
      cout << "Player1 Perform Followpath (queue size): " << m_q.size() << endl;
      /*          while(robot->GetPos().DistanceTo(defaultPos) > 0.05)
                {
                    usleep(10000000);
                }*/
      pos = m_coordCalib->NormalizePosition(GetPos());
      mapx = Interpreter::coord2mapX(pos.GetX()) + m_go_x;
      mapy = Interpreter::coord2mapY(pos.GetY()) + m_go_y;

      pos.SetX(Interpreter::map2coordX(mapx));
      pos.SetY(Interpreter::map2coordY(mapy));
      pos = m_coordCalib->UnnormalizePosition(pos.GetPos());

      //CruisetoBias(pos.GetX(), pos.GetY(), 600, -10, 30);
      //robot->GotoPos(pos);
      //wait until movement is done
      //usleep(0.5e6);

      break;
    case DEFENSE:

      NewRoboControl::cruisetoBias(m_defendpm.GetX(), m_defendpm.GetY(), 650, -10, 30);
      break;

      break;

    default:
      cout << "Player1 Perform Default Command: " << endl;
      GotoXY(m_defaultPos.GetX(), m_defaultPos.GetY());
      break;
  }

  return 0;

}
