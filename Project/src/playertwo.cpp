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
#include "robo_control.h"
#include "playertwo.h"



PlayerTwo::PlayerTwo(RoboControl *x,RawBall *b) {
    robot = x;
    ball  = b;
}
PlayerTwo::~PlayerTwo(){
}

void PlayerTwo::setNextCmd(void *s){

        interpreter* info = (interpreter*)s;

        switch(info->playmode)

        {

                case PENALTY:
                            nextCmd = GO_TO_DEF_POS;

                             break;

                case PLAY_ON:
                        nextCmd = PLAY;
                        break;

                case KICK_OFF:

                        if (info->turn == interpreter::OUR_TURN)
                            nextCmd = PlayerTwo::KICK_OFF;
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
        }

}


void PlayerTwo::setCmdParam(){

        switch(nextCmd)
        {

                default:
                       break;



        }
}

void *PlayerTwo::performCmd(){
        switch(nextCmd)
        {

                case PlayerTwo::GO_TO_DEF_POS:
                        std::cout << "Player1 Perform Go To Default Pos:" <<std::endl;
                        robot->GotoXY(defaultPos.GetX(),defaultPos.GetY());
                        while(robot->GetPos().DistanceTo(defaultPos)> 0.05)
                        {
                            usleep(10000000);
                        }
                        break;
                default:
                        std::cout << "Player1 Perform Default Command: " <<std::endl;
                        robot->GotoXY(defaultPos.GetX(),defaultPos.GetY());
                        break;
        }
           return 0;

}
void *PlayerTwo::performCmd_helper(void *context){

    return ((PlayerTwo*)context)->performCmd();

}



