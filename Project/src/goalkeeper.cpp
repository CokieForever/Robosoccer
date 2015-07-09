/*
 * interpreter.cpp
 *
 *  Created on: Apr 26, 2015
 *      Author: dean
 */

#include "goalkeeper.h"
#include "interpreter.h"
#include "newrobocontrol.h"


Goalkeeper::Goalkeeper(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer* coordCalib, RawBall* b, BallMonitor* ballgk) : TeamRobot(DBC, deviceNr, coordCalib, b)
{
  m_ballgk = ballgk;
}


void Goalkeeper::setNextCmd(Interpreter* info)
{
  Interpreter::GameData mode = info->getMode();

  switch (mode.mode)
  {
    case PENALTY:
      if (mode.turn != Interpreter::OUR_TURN)
        m_nextCmd = PREVENT_GOAL;
      else
        m_nextCmd = GO_TO_DEF_POS;
      break;

    case PLAY_ON:
      m_nextCmd = PREVENT_GOAL;
      break;

    default:
      m_nextCmd = GO_TO_DEF_POS;
      break;
  }
}

void Goalkeeper::setCmdParam()
{
  m_ballgk->GetBallPosition(&m_predictballgk);
  m_defendgk.SetX(-1.43);
  m_defendgk.SetY(0.036);
  static int counter = 0;
  double y;
  double deltaY;

  switch (m_nextCmd)
  {
      //GK calculate predict value of ball to goal line
    case PREVENT_GOAL:
      if (counter >= 9)      //counter for cruisetobias(), otherwise too many comants, robot has not enough time to drive
      {
        m_ballgk->PredictBallPosition(& m_predictballgk, 4, -0.99);
        y = m_predictballgk.GetY();
        //Define Goal borders
        if (y > 0.22)
          y = 0.22;
        else if (y < -0.15)
          y = -0.15;

        deltaY = fabs(GetY() - y);
        m_preventGoalParam.SetX(m_defendgk.GetX());
        m_preventGoalParam.SetY(y);
        counter = 0;
      }
      counter ++;
      break;

    case GO_TO_DEF_POS:
      std::cout << " = " << m_defendgk << std::endl;
      break;

    default :
      break;
  }
}


void* Goalkeeper::performCmd(void)
{

  switch (m_nextCmd)
  {
    case Goalkeeper::PREVENT_GOAL:
      NewRoboControl::cruisetoBias(m_defendgk.GetX(), m_preventGoalParam.GetY(), 650, -10, 30);
      break;

    case Goalkeeper::GO_TO_DEF_POS:
      GotoXY(m_defendgk.GetX(), m_defendgk.GetY());
      while (GetPos().DistanceTo(m_defendgk) > 0.1)
      {
        usleep(10000000);
      }
      break;

    default:
      break;
  }

  return 0;
}

