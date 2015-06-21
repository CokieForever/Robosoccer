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
#include "robo_control.h"
#include <queue>
#include "playermain.h"

using namespace std;

PlayerMain::PlayerMain(RoboControl *x,RawBall *b) {
    robot = x;
    ball  = b;

    for(int i=0 ; i<WIDTH;i++){

        for(int j=0;j<HEIGHT;j++){
            map[i][j]=0;
        }
    }


}
PlayerMain::~PlayerMain(){
}

void PlayerMain::setNextCmd(void *s){

        interpreter* info = (interpreter*)s;

        switch(info->mode.mode)

        {

                case PENALTY:
                        if (info->mode.turn == interpreter::OUR_TURN)
                            nextCmd = KICK_PENALTY;
                        else
                            nextCmd = GO_TO_DEF_POS;

                        break;

                case BEFORE_PENALTY:
                        nextCmd = GO_TO_DEF_POS;
                        kickPenaltyParam.action1Performed = 0;
                        kickPenaltyParam.action2Performed = 0;
                        break;


                case PLAY_ON:
                        nextCmd = FOLLOWPATH;
                        break;

                case KICK_OFF:

                        if (info->mode.turn == interpreter::OUR_TURN)
                            nextCmd = PlayerMain::KICK_OFF;
                        else
                            nextCmd = GO_TO_DEF_POS;

                        break;

                case PAUSE:
                       nextCmd = STOP;
                       break;
                case TIME_OVER:
                       nextCmd = STOP;
                       break;
                default:
                        nextCmd = GO_TO_DEF_POS;
                        break;
        }

}


void PlayerMain::setCmdParam(){
        //both needed when it comes to path tracking
        Point A,B;
        //Point *pt;
        char c;
        int j;
        Position tmp;
        queue<Position> empty_q;


        switch(nextCmd)
        {

                case KICK_PENALTY:
                        kickPenaltyParam.ball = ball->GetPos();
                        kickPenaltyParam.pos.SetX(0.5);
                        kickPenaltyParam.pos.SetY(0.25);
                        break;

                case FOLLOWPATH:
                        cout << "q size :" <<q.size()<<endl;
                        if(q.size()==0){

                        A.x = coord2mapX(robot->GetX());
                        A.y = coord2mapY(robot->GetY());
                        B.x = coord2mapX(ball->GetX());
                        B.y = coord2mapY(ball->GetY());

                        //get string with motion commands
                        path = pathFind(map,A,B);

                        cout << "current path string"<< path<< endl;


                        /*
                        //gets list of checkpoints
                        pt = getCheckPoints(A,path);
                        //load checkpoints to queue
                        for (unsigned int i= 0 ; i<path.length(); i++){
                            tmp.SetX(map2coordX(pt[i].x));
                            tmp.SetX(map2coordY(pt[i].y));

                            q.push(tmp);
                            cout << "checkpoints x/y: "<< map2coordX(pt[i].x)<< "/"<<map2coordX(pt[i].y)<<endl;
                         }
                        */

                        for (unsigned int i= 0 ; i<path.length(); i++){

                                    c=path.at(i);
                                    j=atoi(&c);
                                    A.x = A.x + dx[j];
                                    A.y = A.y + dy[j];

                                    tmp.SetX(map2coordX(A.x));
                                    tmp.SetX(map2coordY(A.y));

                                    q.push(tmp);

                        }





                        //set memory free of the allocated array in getcheckpoints- necessary?!
                        /*
                        delete[] pt;
                        pt = NULL;

                        std::cout<<path<<std::endl;
                        std::cout<< "Checkpoint "<< "i" << "X : "<<"Y: "<< std::endl;
                        */
                        }

                        break;

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

void *PlayerMain::performCmd(){
            std::cout<< "Player 1 next command is:" << nextCmd<< endl <<"1: GO_TO_DEF_POS,KICK_PENALTY 2: KICK_OFF 3: STOP 4: FOLLOWPATH"<<std::endl;
            Position pos;
            switch(nextCmd)
            {
                    case PlayerMain::KICK_PENALTY:

                                //at first go to an predefined position then kick the ball
                                // approach to define multiple commands in one command : define checkpoints in action structure

                                if (kickPenaltyParam.action1Performed==0)
                                {
                                    std::cout << "Player1 Perform Kick Penalty: " <<  std::endl;
                                    robot->GotoXY(kickPenaltyParam.pos.GetX(),kickPenaltyParam.pos.GetY());

                                    while(robot->GetPos().DistanceTo(kickPenaltyParam.pos)>0.05)
                                    {

                                        usleep(100000);

                                    }
                                    kickPenaltyParam.action1Performed=1;

                                }
                                if (kickPenaltyParam.action1Performed==1 && kickPenaltyParam.action2Performed==0)
                                {
                                    robot->GotoXY(kickPenaltyParam.ball.GetX(),kickPenaltyParam.ball.GetY());
                                    kickPenaltyParam.action2Performed=1;
                                }
                                break;

                    case PlayerMain::GO_TO_DEF_POS:

                            std::cout << "Player1 Perform Go To Default Pos:" <<std::endl;
                            robot->GotoXY(defaultPos.GetX(),defaultPos.GetY());

                            break;

                    case PlayerMain::KICK_OFF:
                            std::cout << "Player1 Perform Kick Off:" <<std::endl;
                            robot->GotoXY(ball->GetX(),ball->GetY());

                            break;

                    case PlayerMain::FOLLOWPATH:

                          std::cout << "Player1 Perform Followpath:" <<std::endl;



                          while(q.size()>0)
                          {
                              pos = q.back();
                              robot->GotoXY(pos.GetX(),pos.GetY());
                              //wait until movement is done
                              usleep(0.5e6);
                              //remove last element from queue
                              q.pop();
                           }
                          break;


                    default:

                            std::cout << "Player1 Perform Default Command: " <<std::endl;

                            robot->GotoXY(defaultPos.GetX(),defaultPos.GetY());

                            break;
            }

        return 0;

}

void *PlayerMain::performCmd_helper(void *context){

    return ((PlayerMain*)context)->performCmd();

}
