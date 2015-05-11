/*
 * playermain.cpp
 *
 *
 *  Created on: Mai 07, 2015
 *      Author: chiraz
 */
#include "playermain.h"
#include "interpreter.h"
#include <iostream>
#include "share.h"
//include the libs from sample code


PlayerMain::PlayerMain(RoboControl *x,Rawball *b) {
    player = x;
    ball  = b;

}

PlayerMain::~PlayerMain(){
}
PlayerMain::setNextCmd(Interpreter *info){
        switch(info->playMode)
        {       case BEFORE_PENALTY:
                        nextCmd = GO_TO_PENALTY_POS;
                case PENALTY:
                        nextCmd = KICK_PENALTY;
                        //break; ?
                case PLAY_ON:
                        nextCmd = PLAY;
                case KICK_OFF:
                        nextCmd = KICK_OFF; //same name for mainplayer and play mode?
                case PAUSE:
                       nextCmd = STOP;
                case TIME_OVER:
                       nextCmd = STOP;
                default:
                        nextCmd = GO_TO_DEF_POS;
        }

}
PlayerMain::setCmdParam(){
        switch(nextCmd)
        {
                case GO_TO_PENALTY_POS:
                        if (interpreter.turn == OUR_TURN)
                            Position playerPos(0,0);
                        else {
                            Position playerPos(0.3,0.3);
                        }
                case KICK_PENALTY:
                        if (interpreter.turn == OUR_TURN)
                           { Position ballPos = ball.GetPos();
                            double y = ballPos.GetY();
                            double deltaY = fabs(robot->GetY() - y);
                            // speed for the GoToXY function:
                            int speed =160;
                            // parameters for the kick function:
                            int force = 100;
                            double distanceToBall = ballPos.DistanceTo(PlayerPos);}





        }
}

