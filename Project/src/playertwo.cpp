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

/**
 * @brief
 *
 * @param DBC
 * @param deviceNr
 * @param coordCalib
 * @param b
 * @param ballPm
 * @param display
 */
PlayerTwo::PlayerTwo(RTDBConn& DBC, const int deviceNr, const CoordinatesCalibrer *coordCalib, RawBall *b, BallMonitor *ballPm, RefereeDisplay *display) : TeamRobot(DBC, deviceNr, coordCalib, b, ballPm, display)
{
}

/**
 * @brief
 *
 * @param info
 */
void PlayerTwo::setNextCmd(const Interpreter::GameData& info)
{
    switch(info.mode)
    {
        case PLAY_ON:
            if (info.formation == Interpreter::DEF)
            {
                Position ballPos;
                m_ballPm->GetBallPosition(&ballPos);
                ballPos = m_coordCalib->NormalizePosition(ballPos);

                if ((ballPos.GetX() <= -DEFENSE_LINE && info.our_side == LEFT_SIDE) || (ballPos.GetX() >= DEFENSE_LINE && info.our_side == RIGHT_SIDE))
                    m_nextCmd = FOLLOWPATH;
                else
                    m_nextCmd = DEFENSE;
            }
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

/**
 * @brief
 *
 * @param interpreter
 */
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
            
        case DEFENSE:
        {
            eSide side = interpreter.getMode().our_side;
            double x = side == LEFT_SIDE ? -DEFENSE_LINE : +DEFENSE_LINE;

            BallMonitor::Direction dir;
            m_ballPm->GetBallDirection(&dir);

            bool predicted = false;
            if ((dir.x > 0 && side == RIGHT_SIDE) || (dir.x < 0 && side == LEFT_SIDE))
            {
                double a, b;
                if (m_ballPm->PredictBallPosition(&a, &b, 4) && a < PathFinder::INFINI_TY)
                {
                    double y = std::max(-0.9, std::min(0.9, a * x + b));
                    m_defendpm = m_coordCalib->UnnormalizePosition(Position(x,y));
                    predicted = true;
                }
            }

            if (!predicted)
            {
                Position ballPos;
                m_ballPm->GetBallPosition(&ballPos);
                ballPos = m_coordCalib->NormalizePosition(ballPos);
                double y = std::max(-0.9, std::min(0.9, ballPos.GetY()));
                m_defendpm = m_coordCalib->UnnormalizePosition(Position(x, y));
            }

            break;
        }
    }
}

/**
 * @brief
 *
 * @param info
 */
void PlayerTwo::performCmd(const Interpreter::GameData& info)
{
    switch(m_nextCmd)
    {
        case GO_TO_DEF_POS:
            Log("Player2 Perform Go To Default Pos", DEBUG);
            cruisetoBias(m_defaultPos.GetX(), m_defaultPos.GetY(), 650);
            break;

        case FOLLOWPATH:
            FollowPath(info);
            break;

        case STOP:
            break;
			
        case DEFENSE:
            Position ballPos;
            m_ballPm->GetBallPosition(&ballPos);
            if (ShouldGoalKick(ballPos, info.our_side))
                KickBall(ballPos);
            else
                cruisetoBias(m_defendpm.GetX(), m_defendpm.GetY(), 650);
            break;
    }
}

/**
 * @brief
 *
 * @param info
 */
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
