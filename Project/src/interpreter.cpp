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
#include "playertwo.h"

//include the libs from sample code



interpreter::interpreter(int x,Referee *y,Goalkeeper *z,PlayerMain *p,PlayerTwo *t,RoboControl *a,RoboControl *b,RoboControl *c,RawBall *d) {
	// TODO Auto-generated constructor stub
	ref  = y;
	gk = z;
        p1 = p;
        p2 = t;
        ball = d;
        e1 = a;
        e2 = b;
        e3 = c;

        mode.mode = ref->GetPlayMode();
        mode.formation = DEF;
        mode.team = x;


	if (x== 0)
		std::cout<<"We are team blue!"<<std::endl;

	else
		std::cout<<"We are team red! "<<std::endl;


        //set gk,p1,p2 map to zero and place penalty area
        for(int i=0 ; i<WIDTH;i++){

            for(int j=0;j<HEIGHT;j++){
                p1->map[i][j] = 0;
            }
        }
        setObstacles(p1->map);

	std::cout<<"Interpreter initialized"<<std::endl;
}

interpreter::~interpreter() {
	// TODO Auto-generated destructor stub
}


bool interpreter::verifyPos(){
	//check if all robots are on their default position and orientation
        if ((gk->robot->GetPos().DistanceTo(gk->defaultPos)< 0.01) && (p1->robot->GetPos().DistanceTo(p1->defaultPos)< 0.01)&& (p2->robot->GetPos().DistanceTo(p2->defaultPos)< 0.01))
                    return 1;

        else
                    return 0;

}


void interpreter::setDefaultPos(){
	//sets default position struct depending on game mode of all robots to predefined values
        switch(ref->GetPlayMode())
	{

                case BEFORE_PENALTY:
                                if((mode.turn == interpreter::OUR_TURN))
				{	
                                        gk->defaultPos.SetX(-0.3);
                                        gk->defaultPos.SetY(0.4);

                                        p1->defaultPos.SetX(0.0);
                                        p1->defaultPos.SetY(0.0);

                                        p2->defaultPos.SetX(-1.0);
                                        p2->defaultPos.SetY(-0.5);
				}
	

                                else if ((mode.turn == interpreter::THEIR_TURN))
				{	
                                    gk->defaultPos.SetX(1.2);
                                    gk->defaultPos.SetY(0.0);

                                    p1->defaultPos.SetX(-0.5);
                                    p1->defaultPos.SetY(-0.5);

                                    p2->defaultPos.SetX(-1.0);
                                    p2->defaultPos.SetY(-0.5);

				}
                                else
                                {
                                    gk->defaultPos.SetX(-0.3);
                                    gk->defaultPos.SetY(0.4);

                                    p1->defaultPos.SetX(0.0);
                                    p1->defaultPos.SetY(0.4);


                                    p2->defaultPos.SetX(-1.0);
                                    p2->defaultPos.SetY(-0.5);

                                }
                                break;

		case BEFORE_KICK_OFF:

                                setScores();

                                if(mode.our_side == LEFT_SIDE)
                                {
                                    gk->defaultPos.SetX(-1.1);
                                    gk->defaultPos.SetY(0.0);

                                    p1->defaultPos.SetX(-0.3);
                                    p1->defaultPos.SetY(-0.2);

                                    p2->defaultPos.SetX(-0.3);
                                    p2->defaultPos.SetY(0.2);



				}
	
                                else if(mode.our_side == RIGHT_SIDE)
				{	
                                    gk->defaultPos.SetX(1.1);
                                    gk->defaultPos.SetY(0.0);

                                    p1->defaultPos.SetX(0.3);
                                    p1->defaultPos.SetY(0.2);

                                    p2->defaultPos.SetX(0.3);
                                    p2->defaultPos.SetY(-0.2);

				}
                                else
                                {
                                    gk->defaultPos.SetX(-1.0);
                                    gk->defaultPos.SetY(0.4);

                                    p1->defaultPos.SetX(0.2);
                                    p1->defaultPos.SetY(0.4);


                                    p2->defaultPos.SetX(0.2);
                                    p2->defaultPos.SetY(-0.4);
                                }

                                break;
		
		case PLAY_ON:
                        //if our team leads with at 2 goals go to mixed formation
                        if(mode.our_score > mode.oponent_score+1)
                            mode.formation = MIX;

                        else
                        {
                            switch(mode.our_side)
                            {

                            case LEFT_SIDE:
                                //if our team is on the left side
                                if (ball->GetX() > MID_THRESHOLD && (mode.formation==DEF ||mode.formation==MIX))
                                    mode.formation = ATK;
                                else if(ball->GetX() < -1 * MID_THRESHOLD && (mode.formation==ATK || mode.formation==MIX))
                                    mode.formation = DEF;
                                break;

                            case RIGHT_SIDE:
                                //if our team is on the right side
                                if (ball->GetX() > MID_THRESHOLD && (mode.formation==ATK||mode.formation==MIX))
                                    mode.formation = DEF;
                                else if(ball->GetX() < -1 * MID_THRESHOLD && (mode.formation==DEF|| mode.formation==MIX))
                                    mode.formation = ATK;
                                break;

                            default:
                                break;
                            }

                         }



			break;
                case PENALTY:
                        if (mode.turn == THEIR_TURN)
                                gk->defaultPos.SetX(1.1);
                        break;
                case KICK_OFF:
                        break;

		default:
                        gk->defaultPos.SetX(1.1);
                        gk->defaultPos.SetY(0.0);

                        p1->defaultPos.SetX(0.3);
                        p1->defaultPos.SetY(0.2);

                        p2->defaultPos.SetX(0.3);
                         p2->defaultPos.SetY(-0.2);
                        //states such "as referee init, kick_off/penalty -> defpos doesnt change
			break;
	}
}



void interpreter::setPlayMode() {

        mode.mode = ref->GetPlayMode();

        if (mode.mode==BEFORE_KICK_OFF || mode.mode==BEFORE_PENALTY )
	{
                setSide();
		setTurn();
		if (verifyPos())
                        ref->SetReady(mode.team);

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





void interpreter::setScores(){
    if (mode.our_side==LEFT_SIDE)
    {
        mode.our_score = ref->GetLeftSideGoals();
        mode.oponent_score = ref->GetRightSideGoals();
    }
    else
    {
        mode.our_score = ref->GetRightSideGoals();
        mode.oponent_score = ref->GetLeftSideGoals();
    }
}




void interpreter::setSide(){

	eSide tmp_side = ref->GetBlueSide();
	// 0<-> blue 1<->red
        if(((mode.team == 0) && (tmp_side==LEFT_SIDE)) || ((mode.team == 1) && (tmp_side  == RIGHT_SIDE)))
                mode.our_side = LEFT_SIDE;
	else
                mode.our_side = RIGHT_SIDE;

}


void interpreter::setTurn(){
        if (mode.our_side==ref->GetSide())
	{
                mode.turn = OUR_TURN;
	}
	else
	{
                mode.turn = THEIR_TURN;
	}

}

void interpreter::updateSituation(){
	setPlayMode();
	setDefaultPos();

}

void interpreter::setObstacles(int map[][HEIGHT])
{
    int width = WIDTH;
    int height = HEIGHT;
    int  gk_x = coord2mapX(gk->robot->GetPos().GetX());
    int  gk_y = coord2mapY(gk->robot->GetPos().GetY());
    int  p1_x = coord2mapX(p1->robot->GetPos().GetX());
    int  p1_y = coord2mapY(p1->robot->GetPos().GetY());
    int  p2_x = coord2mapX(p2->robot->GetPos().GetX());
    int  p2_y = coord2mapY(p2->robot->GetPos().GetY());
    int  borderwidth = 3;

    //restrict borders and penalty area
    for (int i = 0 ; i<width;i++)
    {
        for(int j =0; j<height;j++){

            if((j==0) || (j==height-1) || (i==0)|| (i==width-1) || ((j > HEIGHT/2 - WIDTH/10) && (j < HEIGHT/2 + WIDTH/10) && ((i < WIDTH/10) || (i > WIDTH-WIDTH/10)))

                    ||   //set obstacle at gk position
                                        ((i <(gk_x+borderwidth)) && (i>(gk_x-borderwidth)) && (gk_x-borderwidth>=0) && ((gk_x+borderwidth)<=99) && ((j <(gk_y+borderwidth)) && (j>(gk_y-borderwidth)) && (gk_y-borderwidth>=0) && ((gk_y+borderwidth)<=79)))
                    ||   //set obstacle at p2 position
                                        ((i <(p1_x+borderwidth)) && (i>(p1_x-borderwidth)) && (p1_x-borderwidth>=0) && ((p1_x+borderwidth)<=99) && ((j <(p1_y+borderwidth)) && (j>(p1_y-borderwidth)) && (p1_y-borderwidth>=0) && ((p1_y+borderwidth)<=79)))
                    ||   //set obstacle at p1 position
                                        ((i <(p2_x+borderwidth)) && (i>(p2_x-borderwidth)) && (p2_x-borderwidth>=0) && ((p2_x+borderwidth)<=99) && ((j <(p2_y+borderwidth)) && (j>(p2_y-borderwidth)) && (p2_y-borderwidth>=0) && ((p2_y+borderwidth)<=79))))
                //set borders to 1
                map[i][j] = 1;                                          //set penalty areas as obstacle

        }
    }
}


int coord2mapX(double x){
    //coordinate from -1 to +1
    //map width from 0 to 99
    int mapX;

    mapX = WIDTH/2 + floor(x * (WIDTH-1)/2.0);

    return mapX;
}

int coord2mapY(double y){
    //coordinate from -1 to +1
    //map width from 0 to 99
    int mapY;

    mapY = HEIGHT/2 + floor(y * (HEIGHT-1)/2.0);

    return mapY;

}

double map2coordX(int mapX){

    double coordX;

    coordX = (mapX-WIDTH/2.0)/(WIDTH/2);

    return coordX;
}


double map2coordY(int mapY){

    double coordY;

    coordY = (mapY-HEIGHT/2.0)/(HEIGHT/2);

    return coordY;

}

Point *getCheckPoints(Point start,string path){

    Point *ptr = NULL;
    Point tmp = start;

    int n = path.length();
    char c;
    int j;
    ptr = new Point[n];

    //calculate checkpoints from start position
    for(int i=0;i<(int)path.length();i++)
    {
        c =path.at(i);
        j=atoi(&c);

        tmp.x = tmp.x+ dx[j];
        tmp.y = tmp.y + dy[j];

        ptr[i] = tmp;
    }

    //dont forget to delete array and set poiter to NULL after walking the checkpoints
    return ptr;
}




string pathFind( int map[][HEIGHT],const Point Start, const Point Finish)
{
    static priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
    static int pqi=0; // pq index
    static node* n0;
    static node* m0;
    static int i, j, x, y, xdx, ydy;
    static char c;
    static int closed_nodes_map[WIDTH][HEIGHT];
    static int open_nodes_map[WIDTH][HEIGHT];
    static int dir_map[WIDTH][HEIGHT];

    // reset the node maps
    for(y=0;y<HEIGHT;y++)
    {
        for(x=0;x<WIDTH;x++)
        {
            closed_nodes_map[x][y]=0;
            open_nodes_map[x][y]=0;
            dir_map[x][y] = 0;
        }
    }

    // create the start node and push into list of open nodes
    n0=new node(Start.x, Start.y, 0, 0);
    n0->updatePriority(Finish.x, Finish.y);
    pq[pqi].push(*n0);
    open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map

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
        if(x==Finish.x && y==Finish.y)
        {
            // generate the path from finish to start
            // by following the directions
            string path="";
            while(!(x==Start.x && y==Start.y))
            {
                j=dir_map[x][y];
                c='0'+(j+dir/2)%dir;
                path=c+path;
                x+=dx[j];
                y+=dy[j];
            }

            // garbage collection
            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();
            return path;
        }

        // generate moves (child nodes) in all possible directions
        for(i=0;i<dir;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];

            if(!(xdx<0 || xdx>WIDTH-1 || ydy<0 || ydy>HEIGHT-1 || map[xdx][ydy]==1
                || closed_nodes_map[xdx][ydy]==1))
            {
                // generate a child node
                m0=new node( xdx, ydy, n0->getLevel(),
                             n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(Finish.x, Finish.y);

                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    // mark its parent node direction
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    // update the priority info
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy]=(i+dir/2)%dir;

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

void showMap(int map[][HEIGHT], string path,const Point start){

    //show planned path
    cout<< "path planned:"<< path << endl;
    if(path.length()>0)
    {
        int j; char c;
        int x=start.x;
        int y=start.y;
        map[x][y]=2;
        for(int i=0;i<(int)path.length();i++)
        {
            c =path.at(i);
            j=atoi(&c);
            x=x+dx[j];
            y=y+dy[j];
            map[x][y]=3;
        }
        map[x][y]=4;

        // display the map with the route
        for(int y=0;y<HEIGHT;y++)
        {
            for(int x=0;x<WIDTH;x++)
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
            cout<<endl;
        }
    }


}
