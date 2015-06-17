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
#include "playermain.h"



PlayerMain::PlayerMain(RoboControl *x,RawBall *b) {
    robot = x;
    ball  = b;
}
PlayerMain::~PlayerMain(){
}

void PlayerMain::setNextCmd(void *s){

        interpreter* info = (interpreter*)s;

        switch(info->playmode)

        {

                case PENALTY:
                        if (info->turn == interpreter::OUR_TURN)
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
                        nextCmd = PLAY;
                        break;

                case KICK_OFF:

                        if (info->turn == interpreter::OUR_TURN)
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

        switch(nextCmd)
        {

                case KICK_PENALTY:
                        kickPenaltyParam.ball = ball->GetPos();
                        kickPenaltyParam.pos.SetX(0.5);
                        kickPenaltyParam.pos.SetY(0.25);
                        break;
                default:
                       break;



        }
}

void *PlayerMain::performCmd(){
            std::cout<< "next command is:" << nextCmd<<std::endl;

            switch(nextCmd)
            {
                    case PlayerMain::KICK_PENALTY:

                                //at first go to an predefined position then kick the ball
                                // approach to define multiple commands in one command : define checkpoints in action structure

                                if (kickPenaltyParam.action1Performed==0)
                                {
                                    std::cout << "Player1 Perform Kick Penalty: " <<  std::endl;
                                    robot->GotoXY(kickPenaltyParam.pos.GetX(),kickPenaltyParam.pos.GetY());

                                  /*  while(robot->GetPos().DistanceTo(kickPenaltyParam.pos)>0.05)
                                    {
                                       usleep(100000);
                                    }*/
                                    kickPenaltyParam.action1Performed=1;

                                }
                                if (kickPenaltyParam.action1Performed==1 && kickPenaltyParam.action2Performed==0)
                                {
                                    robot->GotoXY(kickPenaltyParam.ball.GetX(),kickPenaltyParam.ball.GetY());
                                    kickPenaltyParam.action2Performed=1;
                                }
                                break;
                    case PlayerMain::GO_TO_DEF_POS:

                std::cout << "Player1 Perform Go To Default Pos:" << robot->GetPos() << endl;
                            robot->GotoXY(defaultPos.GetX(),defaultPos.GetY());
                  /*          while(robot->GetPos().DistanceTo(defaultPos) > 0.05)
                            {
                                usleep(10000000);
                            }*/
                            break;

                    case PlayerMain::KICK_OFF:
                            std::cout << "Player1 Perform Kick Off:" <<std::endl;
                            robot->GotoXY(ball->GetX(),ball->GetY());

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
