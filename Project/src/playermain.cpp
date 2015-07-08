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
    Interpreter::GameData mode = info->getMode();

    switch(mode.mode)
    {
        case PENALTY:
            if (mode.turn == Interpreter::OUR_TURN)
                m_nextCmd = KICK_PENALTY;
            else
                m_nextCmd = GO_TO_DEF_POS;
            break;

        case BEFORE_PENALTY:
            m_nextCmd = GO_TO_DEF_POS;
            break;

        case PLAY_ON:
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

/**
 * @brief
 *
 * @param interpreter
 */
void PlayerMain::setCmdParam(const Interpreter& interpreter)
{
    //both needed when it comes to path tracking
    Interpreter::Point A,B;
    //interpreter::Point *pt;
    char c;
    int j,idx_tmp;
    unsigned int interpolate_n = 15;
    Position robo_n,ball_n;

    switch(m_nextCmd)
    {

        case KICK_PENALTY:
            m_kickPenaltyParam.ball = m_ball->GetPos();
            m_kickPenaltyParam.pos.SetX(0.5);
            m_kickPenaltyParam.pos.SetY(0.25);
            break;

        case FOLLOWPATH:
            if (m_q.size()==0)
            {
                //create queue of move indices

                robo_n = m_coordCalib->NormalizePosition(GetPos());
                ball_n = m_coordCalib->NormalizePosition(m_ball->GetPos());

                A.x = Interpreter::coord2mapX(robo_n.GetX());
                A.y = Interpreter::coord2mapY(robo_n.GetY());
                B.x = Interpreter::coord2mapX(ball_n.GetX());
                B.y = Interpreter::coord2mapY(ball_n.GetY());

                //get string with motion commands
                m_path = Interpreter::pathFind(m_map,A,B);
                Interpreter::showMap(m_map,m_path,A);

                for (unsigned int i= 0 ; i<m_path.length() ; i++)
                {
                    c=m_path.at(i);
                    j=atoi(&c);
                    m_q.push(j);
                }
            }
            m_go_x = 0;
            m_go_y = 0;

            //take n points and interpolate
            if (m_q.size()>= interpolate_n)
            {
                for (unsigned int i=0;i<interpolate_n;i++)
                {
                    idx_tmp = m_q.front();
                    m_go_x = m_go_x + Interpreter::DX[idx_tmp];
                    m_go_y = m_go_y + Interpreter::DY[idx_tmp];
                    m_q.pop();
                }

            }
            else
            {
                while(m_q.size()!=0)
                {
                    idx_tmp = m_q.front();
                    m_go_x = m_go_x + Interpreter::DX[idx_tmp];
                    m_go_y = m_go_y + Interpreter::DY[idx_tmp];
                    m_q.pop();
                }
            }

            break;

        case GO_TO_DEF_POS:
            break;
        case KICK_OFF:
            m_otherRobots[0] = interpreter.getGK();
            m_otherRobots[1] = interpreter.getP2();
            m_otherRobots[2] = interpreter.getE1();
            m_otherRobots[3] = interpreter.getE2();
            m_otherRobots[4] = interpreter.getE3();
            break;

        case STOP:
            break;
        default:
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
        case PlayerMain::KICK_PENALTY:
            KickPenalty((const NewRoboControl**)m_otherRobots);
            break;

        case PlayerMain::GO_TO_DEF_POS:
            Log("Player1 Perform Go To Default Pos", DEBUG);
            cruisetoBias(m_defaultPos.GetX(),m_defaultPos.GetY(), 650);
            break;

        case PlayerMain::KICK_OFF:
            Log("Player1 Perform Kick Off", INFO);
            KickOff((const NewRoboControl**)m_otherRobots, info.our_side, true);
            break;

        case PlayerMain::FOLLOWPATH:
            cout << "Player1 Perform Followpath (queue size): " << m_q.size() << endl;

            pos = m_coordCalib->NormalizePosition(GetPos());
            mapx = Interpreter::coord2mapX(pos.GetX())+ m_go_x;
            mapy = Interpreter::coord2mapY(pos.GetY())+ m_go_y;

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

            cruisetoBias(pos.GetX(),pos.GetY(), 600, -10, 30);
            //robot->GotoPos(pos);
            //wait until movement is done
            //usleep(0.5e6);

//Player main defend the goal corners
//could be used in Defend Mode for P2
/**
 * @brief
 *
 */
void PlayerMain::defend_p2(void)
{
    static int counter = 0;
    double y;
    if (counter >= 10)        //Counter for Cruisetobias function
    {
        m_ballpm->GetBallPosition(&m_defendp2);
        y = m_defendp2.GetY();

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
}

    return 0;
}
