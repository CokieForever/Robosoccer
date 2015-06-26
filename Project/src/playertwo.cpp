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
#include "newrobocontrol.h"

PlayerTwo::PlayerTwo(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer* coordCalib, RawBall* b,  BallMonitor* ballpt) : TeamRobot(DBC, deviceNr, coordCalib, b)
{
  m_ballpt = ballpt;

}

void PlayerTwo::setNextCmd(Interpreter* info)
{
  Interpreter::GameData mode = info->getMode();

  switch (mode.mode)
  {
    case PENALTY:
      m_nextCmd = GO_TO_DEF_POS;
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

  switch (m_nextCmd)
  {


    case DEFENSE:

      break;

default:
  break;
  }
}



void* PlayerTwo::performCmd(void)
{

  switch (m_nextCmd)
  {
    case PlayerTwo::GO_TO_DEF_POS:
      std::cout << "Player1 Perform Go To Default Pos:" << std::endl;
      GotoXY(m_defaultPos.GetX(), m_defaultPos.GetY());
      break;

    case DEFENSE:
      NewRoboControl::cruisetoBias(m_defendp2.GetX(), m_defendp2.GetY(), 650, -10, 30);
      break;

    default:
      /*
            std::cout << "Player2 Perform Default Command: " << std::endl;
            GotoXY(m_defaultPos.GetX(), m_defaultPos.GetY());
            break;*/
      break;
  }

  return 0;
}


//Playertwo defend the goal corners
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
