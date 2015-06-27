/*
 * interpreter.cpp
 *
 *  Created on: Apr 26, 2015
 *      Author: dean
 */

#include "interpreter.h"
#include "referee.h"
#include <iostream>
#include "share.h"
#include "goalkeeper.h"
#include "playermain.h"
#include <queue>
#include "node.h"
#include "coordinates.h"
#include "playertwo.h"
#include "matrix.h"
#include "sdlutilities.h"

//include the libs from sample code

using namespace std;

/*** if dir==8 ***/
const int Interpreter::DX[Interpreter::DIR] = {1, 1, 0, -1, -1, -1, 0, 1};
const int Interpreter::DY[Interpreter::DIR] = {0, 1, 1, 1, 0, -1, -1, -1};
/*** end if ***/


int Interpreter::coord2mapX(double x)
{
    //coordinate from -1 to +1
    int mapX;
    int width = MAP_WIDTH - 2 * MAP_BORDERSIZE;

    mapX = floor(MAP_WIDTH/2 + x * (width-1)/2.0);

    return mapX;
}

int Interpreter::coord2mapY(double y)
{
    //coordinate from -1 to +1
    int mapY;
    int height = MAP_HEIGHT - 2 * MAP_BORDERSIZE;

    mapY = floor(MAP_HEIGHT/2 + y * (height-1)/2.0);

    return mapY;
}

double Interpreter::map2coordX(int mapX)
{
    double coordX;
    int width = MAP_WIDTH - 2 * MAP_BORDERSIZE;
    coordX = (mapX-(MAP_WIDTH/2.0)) * (2.0/(width-1.5));

    return coordX;
}

double Interpreter::map2coordY(int mapY)
{
    double coordY;
    int height = MAP_HEIGHT - 2 * MAP_BORDERSIZE;
    coordY = (mapY-(MAP_HEIGHT/2.0)) * (2.0/(height-1.5));

    return coordY;
}

Interpreter::Point* Interpreter::getCheckPoints(Interpreter::Point start, string path)
{

    Point *ptr = NULL;
    Point tmp = start;

    int n = path.length();
    char c;
    int j;
    ptr = new Point[n];

    //calculate checkpoints from start position
    for (int i=0 ; i<(int)path.length() ; i++)
    {
        c =path.at(i);
        j = atoi(&c);

        tmp.x = tmp.x + DX[j];
        tmp.y = tmp.y + DY[j];

        ptr[i] = tmp;
    }

    //don't forget to delete array and set pointer to NULL after walking the checkpoints
    return ptr;
}

string Interpreter::pathFind(Interpreter::Map map, Interpreter::Point start, Interpreter::Point finish)
{
    static priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
    static int pqi=0; // pq index
    static node* n0;
    static node* m0;
    static int i, j, x, y, xdx, ydy;
    static char c;
    static Map closed_nodes_map;
    static Map open_nodes_map;
    static Map dir_map;

    // reset the node maps
    for(y=0 ; y<MAP_HEIGHT ; y++)
    {
        for(x=0 ; x<MAP_WIDTH ; x++)
        {
            closed_nodes_map[x][y]=0;
            open_nodes_map[x][y]=0;
            dir_map[x][y] = 0;
        }
    }

    // create the start node and push into list of open nodes
    n0 = new node(start.x, start.y, 0, 0);
    n0->updatePriority(finish.x, finish.y);
    pq[pqi].push(*n0);
    open_nodes_map[x][y] = n0->getPriority(); // mark it on the open nodes map

    // A* search
    while(!pq[pqi].empty())
    {
        // get the current node w/ the highest priority
        // from the list of open nodes
        n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x=n0->getxPos(); y=n0->getyPos();

        pq[pqi].pop(); // remove the node from the open list
        open_nodes_map[x][y]=0;
        // mark it on the closed nodes map
        closed_nodes_map[x][y]=1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x==finish.x && y==finish.y)
        {
            // generate the path from finish to start
            // by following the directions
            string path="";
            while(!(x==start.x && y==start.y))
            {
                j = dir_map[x][y];
                c = '0' + (j+DIR/2) % DIR;
                path = c + path;
                x += DX[j];
                y += DY[j];
            }

            // garbage collection
            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();
            return path;
        }

        // generate moves (child nodes) in all possible directions
        for(i=0 ; i < DIR ; i++)
        {
            xdx = x + DX[i];
            ydy = y + DY[i];

            if(!(xdx<0 || xdx>MAP_WIDTH-1 || ydy<0 || ydy>MAP_HEIGHT-1 || map[xdx][ydy]==1
                || closed_nodes_map[xdx][ydy]==1))
            {
                // generate a child node
                m0=new node( xdx, ydy, n0->getLevel(),
                             n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(finish.x, finish.y);

                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    // mark its parent node direction
                    dir_map[xdx][ydy] = (i + DIR/2) % DIR;
                }
                else if(open_nodes_map[xdx][ydy]>(Uint32)m0->getPriority())
                {
                    // update the priority info
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy]= (i + DIR/2) % DIR;

                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqi].top().getxPos()==xdx &&
                           pq[pqi].top().getyPos()==ydy))
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pq[pqi].pop(); // remove the wanted node

                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0); // add the better node instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    return ""; // no route found
}

void Interpreter::showMap(const Interpreter::Map& map0, string path, Interpreter::Point start)
{
    Map map(map0);

    //show planned path
    cout << "path planned:" << path << endl;
    if (path.length()>0)
    {
        int j; char c;
        int x = start.x;
        int y = start.y;
        map[x][y]=2;

        int l = (int)path.length();
        for(int i=0 ; i < l ; i++)
        {
            c = path.at(i);
            j = atoi(&c);
            x = x + DX[j];
            y = y + DY[j];
            map[x][y]=3;
        }
        map[x][y]=4;

        // display the map with the route
        for(int y=0 ; y<MAP_HEIGHT ; y++)
        {
            for(int x=0 ; x<MAP_WIDTH ; x++)
            {
                if(map[x][y]==0)
                    cout<<".";
                else if(map[x][y]==1)
                    cout<<"O"; //obstacle
                else if(map[x][y]==2)
                    cout<<"S"; //start
                else if(map[x][y]==3)
                    cout<<"R"; //path
                else if(map[x][y]==4)
                    cout<<"F"; //finish
            }
            cout << endl;
        }
    }
}

void Interpreter::matrixupdate(Interpreter::Map& map, const NewRoboControl* ref, const NewRoboControl* obstacles[5], RawBall* ball, CoordinatesCalibrer* coordCalibrer, eSide our_side)
{
    CoordinatesCalibrer *m_coordCalibrer = coordCalibrer;
    //Normalize coordinates of our robot
    Position pos1 = m_coordCalibrer->NormalizePosition((ref->GetPos()));//ref is our robot
    Position posptr[5];
    //indices of the robot's position in the matrix
    int i1 = coord2mapX(pos1.GetX());
    int j1 = coord2mapY(pos1.GetY());
    int width = MAP_WIDTH;
    int height = MAP_HEIGHT;
    int border = MAP_BORDERSIZE;
    int obstacle_r = 1.5 * MAP_BORDERSIZE;

    //clear map
    map.Fill(0);

    //get all positions of the robots
    for (int k=0 ; k < 5 ; k++)
    {
        //Normalize coordinates of the obstacles
        posptr[k] = m_coordCalibrer->NormalizePosition((obstacles[k])->GetPos());   //obstacles[i] are the robots
    }

    pos1=m_coordCalibrer->NormalizePosition(ref->GetPos());
    //restrict borders and penalty area
    for (int i = 0 ; i<width;i++)
    {
        for (int j =0; j<height;j++)
        {
            //TODO Make this readable...
            if((i>=0 && i< border) || (i >=(width - border) && i<= (width-1)) || (j>=0 && j< border) || (j >=(height - border) && j<= (height-1))
                    || ((j > MAP_HEIGHT/2 - MAP_WIDTH/10) && (j < MAP_HEIGHT/2 + MAP_WIDTH/10)
                        && ((i < MAP_WIDTH/10)|| (i > MAP_WIDTH-MAP_WIDTH/10))))
            map[i][j] = 1;

            //set penalty areas as obstacles

            if (ceil(pow(i-coord2mapX(posptr[0].GetX()),2)+pow(j-coord2mapY(posptr[0].GetY()),2)) <= ceil(pow(obstacle_r,2)))
                map[i][j] = 1;
            if (ceil(pow(i-coord2mapX(posptr[1].GetX()),2)+pow(j-coord2mapY(posptr[1].GetY()),2)) <= ceil(pow(obstacle_r,2)))
                map[i][j] = 1;
            if (ceil(pow(i-coord2mapX(posptr[2].GetX()),2)+pow(j-coord2mapY(posptr[2].GetY()),2)) <= ceil(pow(obstacle_r,2)))
                map[i][j] = 1;
            if (ceil(pow(i-coord2mapX(posptr[3].GetX()),2)+pow(j-coord2mapY(posptr[3].GetY()),2)) <= ceil(pow(obstacle_r,2)))
                map[i][j] = 1;
            if (ceil(pow(i-coord2mapX(posptr[4].GetX()),2)+pow(j-coord2mapY(posptr[4].GetY()),2)) <= ceil(pow(obstacle_r,2)))
                map[i][j] = 1;

            //clear the area around reference

            if (ceil(pow(i-coord2mapX(pos1.GetX()),2)+pow(j-coord2mapY(pos1.GetY()),2)) <= ceil(pow(obstacle_r,2)))
                map[i][j] = 0;
        }
    }

    //Normalize coordinates of the ball
    pos1 = m_coordCalibrer->NormalizePosition(ball->GetPos());

    //indices of the robot's position in the matrix
    int i=coord2mapX(pos1.GetX());
    int j=coord2mapY(pos1.GetY());
    map[i][j]=3; // affect 3 to the position of the ball in the matrix

    //generate the obstacles around the ball depending on the side in which our team plays
    double bx = coord2mapX(pos1.GetX()), by = coord2mapY(pos1.GetY());
    double goalX = coord2mapX(our_side == LEFT_SIDE ? 1 : -1);
    double cosAngle, sinAngle;
    ComputeLineAngle(bx, by, goalX, coord2mapY(0), &cosAngle, &sinAngle);

    double x, y;
    ComputeVectorEnd(bx, by, cosAngle, sinAngle, 5, &x, &y);

    double x1, y1;
    ComputeVectorEnd(x, y, sinAngle, -cosAngle, 5, &x1, &y1);

    double x2, y2;
    ComputeVectorEnd(x, y, -sinAngle, cosAngle, 5, &x2, &y2);
    map.DrawThickLine(Map::CreatePoint(x1,y1), Map::CreatePoint(x2,y2), 2, 1);

    ComputeVectorEnd(x1, y1, -cosAngle, -sinAngle, 10, &x, &y);
    map.DrawThickLine(Map::CreatePoint(x1,y1), Map::CreatePoint(x,y), 3, 1);

    ComputeVectorEnd(x2, y2, -cosAngle, -sinAngle, 10, &x, &y);
    map.DrawThickLine(Map::CreatePoint(x2,y2), Map::CreatePoint(x,y), 3, 1);

    map[i1][j1] = 2; // affect 2 to the position of our robot in the matrix

    /*
    //indices of the robot's position in the matrix
    i=coord2mapX(pos1.GetX());
    j=coord2mapY(pos1.GetY());

    cout<< "i: "<<i<<" j: "<<j<<endl;

    map[i][j]=1; // affect 1 to the position of the obstacle and its surroundings in the matrix
    map[i-1][j]=1;
    map[i+1][j]=1;
    map[i][j-1]=1;
    map[i][j+1]=1;
    */
}


Interpreter::Interpreter(int x,Referee *y,Goalkeeper *z,PlayerMain *p,PlayerTwo *t,OpponentRobot *a,OpponentRobot *b,OpponentRobot *c,RawBall *d,CoordinatesCalibrer *e)
{
    m_ref  = y;
    m_gk = z;
    m_p1 = p;
    m_p2 = t;
    m_ball = d;
    m_e1 = a;
    m_e2 = b;
    m_e3 = c;
    m_cal = e;

    m_mode.mode = m_ref->GetPlayMode();
    m_mode.formation = DEF;
    m_mode.team = x;

    if (x== 0)
        cout << "We are team blue!" << endl;
    else
        cout << "We are team red!" << endl;

    //set gk,p1,p2 map to zero and place penalty area
    m_p1Map.Fill(0);
    m_p2Map.Fill(0);

    pthread_mutex_init(&m_p1MapMutex, NULL);
    pthread_mutex_init(&m_p2MapMutex, NULL);

    pthread_mutex_init(&m_mutex, NULL);
    pthread_cond_init(&m_cond, NULL);

    cout << "Interpreter initialized" << endl;
}

Interpreter::~Interpreter()
{
    pthread_cond_destroy(&m_cond);
    pthread_mutex_destroy(&m_mutex);

    pthread_mutex_destroy(&m_p1MapMutex);
    pthread_mutex_destroy(&m_p2MapMutex);
}

Interpreter::GameData Interpreter::getMode() const
{
    return m_mode;
}

void Interpreter::SetP1MapToRobot(PlayerMain *p1) const
{
    pthread_mutex_lock((pthread_mutex_t*)&m_p1MapMutex);
    p1->setMap(m_p1Map);
    pthread_mutex_unlock((pthread_mutex_t*)&m_p1MapMutex);
}

void Interpreter::SetP2MapToRobot(PlayerTwo *p2) const
{
    pthread_mutex_lock((pthread_mutex_t*)&m_p2MapMutex);
    p2->setMap(m_p2Map);
    pthread_mutex_unlock((pthread_mutex_t*)&m_p2MapMutex);
}

Position Interpreter::getGKDefaultPos() const
{
    return m_gkDefaultPosition;
}

Position Interpreter::getP1DefaultPos() const
{
    return m_p1DefaultPosition;
}

Position Interpreter::getP2DefaultPos() const
{
    return m_p2DefaultPosition;
}

const Goalkeeper* Interpreter::getGK() const
{
    return m_gk;
}

const PlayerMain* Interpreter::getP1() const
{
    return m_p1;
}

const PlayerTwo* Interpreter::getP2() const
{
    return m_p2;
}

const OpponentRobot* Interpreter::getE1() const
{
    return m_e1;
}

const OpponentRobot* Interpreter::getE2() const
{
    return m_e2;
}

const OpponentRobot* Interpreter::getE3() const
{
    return m_e3;
}

bool Interpreter::verifyPos()
{
    //check if all robots are on their default position and orientation
    return (m_gk->GetPos().DistanceTo(m_gk->getDefaultPosition())< 0.01)
            && (m_p1->GetPos().DistanceTo(m_p1->getDefaultPosition())< 0.01)
            && (m_p2->GetPos().DistanceTo(m_p2->getDefaultPosition())< 0.01);
}

void Interpreter::setDefaultPos()
{
    //sets default position struct depending on game mode of all robots to predefined values
    switch(m_ref->GetPlayMode())
    {

        case BEFORE_PENALTY:
            if((m_mode.turn == Interpreter::OUR_TURN))
            {
                m_gkDefaultPosition = Position(-0.3, 0.4);
                m_p1DefaultPosition = Position(0.0, 0.0);
                m_p2DefaultPosition = Position(-1.0, -0.5);
            }
            else if ((m_mode.turn == Interpreter::THEIR_TURN))
            {
                m_gkDefaultPosition = Position(1.2, 0.0);
                m_p1DefaultPosition = Position(-0.5, -0.5);
                m_p2DefaultPosition = Position(-1.0, -0.5);
            }
            else
            {
                m_gkDefaultPosition = Position(-0.3, 0.4);
                m_p1DefaultPosition = Position(0.0, 0.4);
                m_p2DefaultPosition = Position(-1.0, -0.5);
            }
            break;

        case BEFORE_KICK_OFF:
            setScores();

            if(m_mode.our_side == LEFT_SIDE)
            {
                m_gkDefaultPosition = Position(-1.1, 0.0);
                m_p1DefaultPosition = Position(-0.3, -0.2);
                m_p2DefaultPosition = Position(-0.3, 0.2);
            }
            else if(m_mode.our_side == RIGHT_SIDE)
            {
                m_gkDefaultPosition = Position(1.1, 0.0);
                m_p1DefaultPosition = Position(0.3, 0.2);
                m_p2DefaultPosition = Position(0.3, -0.2);
            }
            else
            {
                m_gkDefaultPosition = Position(-1.0, 0.4);
                m_p1DefaultPosition = Position(0.2, 0.4);
                m_p2DefaultPosition = Position(-0.2, -0.4);
            }

            break;

        case PLAY_ON:
            //if our team leads with at 2 goals go to mixed formation
            if(m_mode.our_score > m_mode.oponent_score+1)
                m_mode.formation = MIX;
            else
            {
                switch(m_mode.our_side)
                {
                    case LEFT_SIDE:
                        //if our team is on the left side
                        if (m_ball->GetX() > MID_THRESHOLD && (m_mode.formation==DEF || m_mode.formation==MIX))
                            m_mode.formation = ATK;
                        else if(m_ball->GetX() < -1 * MID_THRESHOLD && (m_mode.formation==ATK || m_mode.formation==MIX))
                            m_mode.formation = DEF;
                        break;

                    case RIGHT_SIDE:
                        //if our team is on the right side
                        if (m_ball->GetX() > MID_THRESHOLD && (m_mode.formation==ATK || m_mode.formation==MIX))
                            m_mode.formation = DEF;
                        else if(m_ball->GetX() < -1 * MID_THRESHOLD && (m_mode.formation==DEF || m_mode.formation==MIX))
                            m_mode.formation = ATK;
                        break;

                    default:
                        break;
                }

            }
            break;

        case PENALTY:
            if (m_mode.turn == THEIR_TURN)
                m_gkDefaultPosition.SetX(1.1);
            break;

        case KICK_OFF:
            break;

        default:
            m_gkDefaultPosition = Position(1.1, 0.5);
            m_p1DefaultPosition = Position(0.5, 0.2);
            m_p2DefaultPosition = Position(-0.5, -0.2);

            //states such "as referee init, kick_off/penalty -> defpos doesnt change
            break;
    }
}

void Interpreter::setPlayMode()
{
    m_mode.mode = m_ref->GetPlayMode();

    if (m_mode.mode==BEFORE_KICK_OFF || m_mode.mode==BEFORE_PENALTY)
    {
        setSide();
        setTurn();
        if (verifyPos())
            m_ref->SetReady(m_mode.team);
    }

    /*else if (playmode==BEFORE_PENALTY)
    {
        setSide();
        setTurn();
    }


    *
    * 	other playmodes and their actions need to be defined
    *
    */

}

void Interpreter::setScores()
{
    if (m_mode.our_side==LEFT_SIDE)
    {
        m_mode.our_score = m_ref->GetLeftSideGoals();
        m_mode.oponent_score = m_ref->GetRightSideGoals();
    }
    else
    {
        m_mode.our_score = m_ref->GetRightSideGoals();
        m_mode.oponent_score = m_ref->GetLeftSideGoals();
    }
}

void Interpreter::setSide()
{

        eSide tmp_side = m_ref->GetBlueSide();
	// 0<-> blue 1<->red
        if(((m_mode.team == 0) && (tmp_side==LEFT_SIDE)) || ((m_mode.team == 1) && (tmp_side  == RIGHT_SIDE)))
                m_mode.our_side = LEFT_SIDE;
	else
                m_mode.our_side = RIGHT_SIDE;

}

void Interpreter::setTurn()
{
    if (m_mode.our_side==m_ref->GetSide())
    {
        m_mode.turn = OUR_TURN;
    }
    else
    {
        m_mode.turn = THEIR_TURN;
    }
}

void Interpreter::updateSituation()
{
    m_situationId = rand();

    setPlayMode();
    setDefaultPos();

    #ifdef PATHPLANNING_ASTAR
    const NewRoboControl *robots1[5] = {m_gk, m_p2, m_e1, m_e2, m_e3};
    pthread_mutex_lock((pthread_mutex_t*)&m_p1MapMutex);
    matrixupdate(m_p1Map, m_p1, robots1, m_ball, m_cal, m_mode.our_side);
    formationUpdateP1();
    pthread_mutex_unlock((pthread_mutex_t*)&m_p1MapMutex);

    const NewRoboControl *robots2[5] = {m_gk, m_p1, m_e1, m_e2, m_e3};
    pthread_mutex_lock((pthread_mutex_t*)&m_p2MapMutex);
    matrixupdate(m_p2Map, m_p2, robots2, m_ball, m_cal, m_mode.our_side);
    formationUpdateP2();
    pthread_mutex_unlock((pthread_mutex_t*)&m_p2MapMutex);
    #endif

    pthread_mutex_lock((pthread_mutex_t*)&m_mutex);
    pthread_cond_broadcast(&m_cond);
    pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);
}

int Interpreter::waitForUpdate(int id)
{
    if (m_situationId != id)
        return m_situationId;

    pthread_mutex_lock((pthread_mutex_t*)&m_mutex);
    pthread_cond_wait(&m_cond, &m_mutex);
    id = m_situationId;
    pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);

    return id;
}

void Interpreter::maskUpperLeft(Map &map)
{
    Map::Point ul = {MAP_BORDERSIZE-1, MAP_BORDERSIZE-1};
    Map::Point lr = {MAP_WIDTH/2, MAP_HEIGHT/2};
    map.DrawRectangle(ul, lr, 1);
}

void Interpreter::maskOmitUpperLeft(Map &map)
{
    maskRight(map);
    maskLowerLeft(map);
}

void Interpreter::maskUpperRight(Map &map)
{
    Map::Point ul = {MAP_WIDTH/2, MAP_BORDERSIZE-1};
    Map::Point lr = {MAP_WIDTH-MAP_BORDERSIZE-1, MAP_HEIGHT/2};
    map.DrawRectangle(ul, lr, 1);
}

void Interpreter::maskOmitUpperRight(Map &map)
{
    maskLeft(map);
    maskLowerRight(map);
}

void Interpreter::maskLowerLeft(Map &map)
{
    Map::Point ul = {MAP_BORDERSIZE-1, MAP_HEIGHT/2};
    Map::Point lr = {MAP_WIDTH/2, MAP_HEIGHT-MAP_BORDERSIZE-1};
    map.DrawRectangle(ul, lr, 1);
}

void Interpreter::maskOmitLowerLeft(Map &map)
{
    maskRight(map);
    maskUpperLeft(map);
}

void Interpreter::maskLowerRight(Map &map)
{
    Map::Point ul = {MAP_WIDTH/2, MAP_HEIGHT/2};
    Map::Point lr = {MAP_WIDTH-MAP_BORDERSIZE-1, MAP_HEIGHT-MAP_BORDERSIZE-1};
    map.DrawRectangle(ul, lr, 1);
}

void Interpreter::maskOmitLowerRight(Map &map)
{
    maskUpperRight(map);
    maskLeft(map);
}

void Interpreter::maskLeft(Map &map)
{
    maskUpperLeft(map);
    maskLowerLeft(map);
}

void Interpreter::maskRight(Map &map)
{
    maskUpperRight(map);
    maskLowerRight(map);
}



void Interpreter::formationUpdateP1()
{
    //Player1 plays on the left side for ATK/DEF,for MIXED Player1 is in ATK, form goalie point of view, if formation is unknown -> ATK

    GameData info = this->getMode();

    switch(info.formation)
    {
        case Interpreter::DEF:
            (info.our_side== LEFT_SIDE) ? maskOmitUpperLeft(m_p1Map) : maskOmitLowerRight(m_p1Map);
            break;

        case Interpreter::MIX:
            (info.our_side== LEFT_SIDE) ? maskLeft(m_p1Map) : maskRight(m_p1Map);
            break;

        default:
            //ATK case
            (info.our_side== LEFT_SIDE) ? maskOmitUpperRight(m_p1Map) : maskOmitLowerLeft(m_p1Map);
            break;

    }

}
void Interpreter::formationUpdateP2()
{
    //Player2 plays on the right side for ATK/DEF,for MIXED Player2 is in DEF, form goalie point of view, if formation is unknown -> DEF
    GameData info = this->getMode();

    switch(info.formation)
    {
        case Interpreter::ATK:
            (info.our_side== LEFT_SIDE) ? maskOmitLowerRight(m_p2Map) : maskOmitUpperLeft(m_p2Map);
            break;


        case Interpreter::MIX:
            (info.our_side== LEFT_SIDE) ? maskRight(m_p2Map) : maskLeft(m_p2Map);
            break;

        default:
            //DEF case
            (info.our_side== LEFT_SIDE) ? maskOmitLowerLeft(m_p2Map) : maskOmitUpperRight(m_p2Map);
            break;
    }


}









