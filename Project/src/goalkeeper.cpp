/*
 * interpreter.cpp
 *
 *  Created on: Apr 26, 2015
 *      Author: dean
 */

#include "goalkeeper.h"
#include "interpreter.h"
#include "newrobocontrol.h"
#include "log.h"


/**
 * @brief This is the constructor of Goalkeeper class
 *
 * @param DBC Database connexion.
 * @param deviceNr The number of the robot.
 * @param coordCalib Calibrer of coordinates.
 * @param b The ball.
 * @param ballPm The ball monitor of the ball.
 */
Goalkeeper::Goalkeeper(RTDBConn& DBC, const int deviceNr, const CoordinatesCalibrer *coordCalib, RawBall *b, BallMonitor *ballPm) : TeamRobot(DBC, deviceNr, coordCalib, b, ballPm)
{
}

/**
 * @brief This function sets the next command for the goalkeeper.
 *
 * @param info Information about the playmode given by the interpreter.
 */
void Goalkeeper::setNextCmd(const Interpreter::GameData& info)
{
    switch(info.mode)
    {
        case PENALTY:
            if (info.turn != Interpreter::OUR_TURN)
                m_nextCmd = PREVENT_GOAL;
            else
                m_nextCmd = GO_TO_DEF_POS;
            break;

        case PLAY_ON:
        {
            Position ballPos = m_coordCalib->NormalizePosition(m_ball->GetPos());

            if (fabs(ballPos.GetY()) <= 0.35 && (info.our_side == LEFT_SIDE ? ballPos.GetX() <= -0.8 : ballPos.GetX() >= +0.8) && m_ball->GetVelocity() <= 0.00005)
                m_nextCmd = CLEAR_GOAL;
            else
                m_nextCmd = PREVENT_GOAL;
            break;
        }

        case BEFORE_KICK_OFF:
        case BEFORE_PENALTY:
            m_nextCmd = GO_TO_DEF_POS;
            break;

        case REFEREE_INIT:
        case KICK_OFF:
        case PAUSE:
        case TIME_OVER:
            m_nextCmd = STOP;
            break;
    }

}

/**
 * @brief This function sets the next command settings for the goalkeeper.
 *
 * @param interpreter The interpreter
 */
void Goalkeeper::setCmdParam(const Interpreter& interpreter)
{
    switch(m_nextCmd)
    {
        case PREVENT_GOAL:
        {
            eSide side = interpreter.getMode().our_side;
            if (interpreter.getMode().mode == PENALTY)
                side = LEFT_SIDE;

            double x = side == LEFT_SIDE ? -0.97 : +0.97;

            BallMonitor::Direction dir;
            m_ballPm->GetBallDirection(&dir);

            bool success = false;
            if ((dir.x >= 0) != (side == LEFT_SIDE))
            {
                double a, b;
                if (m_ballPm->PredictBallPosition(&a, &b, 4) && a < PathFinder::INFINI_TY)
                {
                    double y = std::max(-0.22, std::min(0.22, a * x + b));
                    m_preventGoalParam = m_coordCalib->UnnormalizePosition(Position(x,y));
                    success = true;
                }
            }

            if (!success)
            {
                Position ballPos;
                m_ballPm->GetBallPosition(&ballPos);
                ballPos = m_coordCalib->NormalizePosition(ballPos);
                double y = std::max(-0.22, std::min(0.22, ballPos.GetY()));
                m_preventGoalParam = m_coordCalib->UnnormalizePosition(Position(x, y));
            }

            break;
        }

        case GO_TO_DEF_POS:
            m_defaultPos = interpreter.getGKDefaultPos();
            Log("Goal keeper moving to Position = " + ToString(m_defaultPos), DEBUG);
            break;

        case CLEAR_GOAL:
        case STOP:
            break;
    }
}

/**
 * @brief perform the commands set by @ref Goalkeeper::setNextCmd()
 *
 * @param info Information about the playmode given by the interpreter.
 */
void Goalkeeper::performCmd(const Interpreter::GameData& info)
{

    switch(m_nextCmd)
    {
        case PREVENT_GOAL:
            cruisetoBias(m_preventGoalParam.GetX(), m_preventGoalParam.GetY(), 600);
            break;

        case GO_TO_DEF_POS:
            cruisetoBias(m_defaultPos.GetX(),m_defaultPos.GetY(), 650);
            break;

        case CLEAR_GOAL:
            cruisetoBias(m_ball->GetX(), m_ball->GetY(), 1000);
            break;

        case STOP:
            break;
    }
}

/**
 * @brief This methods adds obstacles where the goalkeeper shouln't go.
 *
 * @param info Information about the game given by the interpreter.
 */
void Goalkeeper::AddObstacleForFormation(const Interpreter::GameData& info)
{
    m_areaObstacle = NULL;
}

