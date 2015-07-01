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
#include "log.h"

PlayerTwo::PlayerTwo(RTDBConn& DBC, const int deviceNr, const CoordinatesCalibrer *coordCalib, RawBall *b, BallMonitor *ballPm, RefereeDisplay *display) : TeamRobot(DBC, deviceNr, coordCalib, b, ballPm, display)
{
}

void PlayerTwo::setNextCmd(const Interpreter::GameData& info)
{
    switch(info.mode)
    {
        case PLAY_ON:
            if (info.formation == Interpreter::DEF)
                m_nextCmd = DEFENSE;
            else
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
            
        //PlayerTwo in Defense Mode follows y-coordinates of ball
        case DEFENSE:
        {
            eSide side = interpreter.getMode().our_side;
            Position ballPos;
            m_ballPm->GetBallPosition(&ballPos);

            if (ShouldGoalKick(ballPos, side))
                GoalKick(ballPos);
            else
            {
                double x = side == LEFT_SIDE ? -0.8 : +0.8;

                BallMonitor::Direction dir;
                m_ballPm->GetBallDirection(&dir);

                bool success = false;
                if ((dir.x >= 0) != (side == LEFT_SIDE))
                {
                    double a, b;
                    if (m_ballPm->PredictBallPosition(&a, &b, 4) && a < PathFinder::INFINI_TY)
                    {
                        double y = std::max(-0.5, std::min(0.5, a * x + b));
                        m_defendpm = m_coordCalib->UnnormalizePosition(Position(x,y));
                        success = true;
                    }
                }

                if (!success)
                {
                    Position ballPos;
                    m_ballPm->GetBallPosition(&ballPos);
                    ballPos = m_coordCalib->NormalizePosition(ballPos);
                    double y = std::max(-0.5, std::min(0.5, ballPos.GetY()));
                    m_defendpm = m_coordCalib->UnnormalizePosition(Position(x, y));
                }
            }

            break;
        }
    }
}

void PlayerTwo::performCmd(const Interpreter::GameData& info)
{
    switch(m_nextCmd)
    {
        case GO_TO_DEF_POS:
            Log("Player2 Perform Go To Default Pos", DEBUG);
            cruisetoBias(m_defaultPos.GetX(), m_defaultPos.GetY(), 650, -10, 30);
            break;

        case FOLLOWPATH:
            FollowPath(info);
            break;

        case STOP:
            break;
			
        case DEFENSE:
            cruisetoBias(m_defendpm.GetX(), m_defendpm.GetY(), 650, -10, 30);
            break;
    }
}

void PlayerTwo::AddObstacleForFormation(const Interpreter::GameData& info)
{
    if (info.formation == Interpreter::ATK)
        m_areaObstacle = m_pathFinder.AddRectangle(PathFinder::CreatePoint(-2, -2), PathFinder::CreatePoint(2, 0));
    else if (info.formation == Interpreter::DEF)
        m_areaObstacle = NULL;  //Behavior different in this mode
    else if (info.formation == Interpreter::MIX)
    {
        if (info.our_side == RIGHT_SIDE)
            m_areaObstacle = m_pathFinder.AddRectangle(PathFinder::CreatePoint(-2, -2), PathFinder::CreatePoint(0, 2));
        else
            m_areaObstacle = m_pathFinder.AddRectangle(PathFinder::CreatePoint(0, -2), PathFinder::CreatePoint(2, 2));
    }else
        m_areaObstacle = NULL;  //Should never happen
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
