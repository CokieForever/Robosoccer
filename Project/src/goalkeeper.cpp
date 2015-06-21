/*
 * interpreter.cpp
 *
 *  Created on: Apr 26, 2015
 *      Author: dean
 */

#include "interpreter.h"
#include <iostream>
#include "share.h"
#include "kogmo_rtdb.hxx"
#include "robo_control.h"




Goalkeeper::Goalkeeper(RoboControl *x,RawBall *b) {
	robot = x;
	ball  = b;

}

Goalkeeper::~Goalkeeper(){
}

void Goalkeeper::setNextCmd(void *s){

        interpreter* info = (interpreter*)s;
        switch(info->mode.mode)
	{
                case PENALTY:
                        if(info->mode.turn != interpreter::OUR_TURN)
                            nextCmd = PREVENT_GOAL;
                        else
                            nextCmd = GO_TO_DEF_POS;
			break;
		case PLAY_ON:

			nextCmd = PREVENT_GOAL;
                        break;
		default:
			nextCmd = GO_TO_DEF_POS;
                        break;
	}

}


void Goalkeeper::setCmdParam(){

    switch(nextCmd)
	{
		case PREVENT_GOAL:

                {
                        Position ballPos = ball->GetPos();
                        double y = ballPos.GetY();

			if (y > 0.15)
				y = 0.15;
			else if (y < -0.15)
				y = -0.15;

			double deltaY = fabs(robot->GetY() - y);

			if (deltaY >= 0.05)
			{
				cout << "Goal keeper moving to y = " << y << std::endl;
                                preventGoalParam.SetX(defaultPos.GetX());
                                preventGoalParam.SetY(y);
			}
			else
                        {
                                preventGoalParam.SetX(defaultPos.GetX());
                                preventGoalParam.SetY(defaultPos.GetY());

                        }
                }
                        break;

                case GO_TO_DEF_POS:
                        std::cout << "Goal keeper moving to Position = " << defaultPos <<std::endl;
                        break;
                default :
			break;

	}
}



void *Goalkeeper::performCmd(void){

	switch(nextCmd)
	{
                case Goalkeeper::PREVENT_GOAL:
                        std::cout << "Next command Prevent Goal Position: " << preventGoalParam.GetPos()<< std::endl;
                        robot->GotoXY(defaultPos.GetX(),preventGoalParam.GetY());
			break;
                case Goalkeeper::GO_TO_DEF_POS:

                        robot->GotoXY(defaultPos.GetX(),defaultPos.GetY());
                        while(robot->GetPos().DistanceTo(defaultPos)> 0.1)
                        {
                            usleep(10000000);
                        }

			break;
		default:
			break;
	}
        return 0;

}

void *Goalkeeper::performCmd_helper(void *context){

    return ((Goalkeeper*)context)->performCmd();

}





