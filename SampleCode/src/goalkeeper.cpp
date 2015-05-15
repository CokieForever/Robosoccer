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
//include the libs from sample code



Goalkeeper::Goalkeeper(RoboControl *x,RawBall *b) {
	robot = x;
	ball  = b;

}

Goalkeeper::~Goalkeeper(){
}

void Goalkeeper::setNextCmd(void *s){
        interpreter* info = (interpreter*)s;
        switch(info->playmode)
	{
		case PENALTY:
			nextCmd = PREVENT_GOAL;
			break;
		case PLAY_ON:
			nextCmd = PREVENT_GOAL;
		default:
			nextCmd = GO_TO_DEF_POS;
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
                                preventGoalParam.SetX(defaultPos.GetX());
                                preventGoalParam.SetY(defaultPos.GetY());

                        break;
                        }

                case GO_TO_DEF_POS:
                        std::cout << "Goal keeper moving to y = "<<std::endl;
                        break;
                default :
			break;

	}
}



void Goalkeeper::performCmd(){
	switch(nextCmd)
	{
		case PREVENT_GOAL:
                        robot->GotoXY(preventGoalParam.GetX(),preventGoalParam.GetY());
			break;
		case GO_TO_DEF_POS:
                        robot->GotoXY(defaultPos.GetX(),defaultPos.GetY());
			break;
		default:
			break;
	}

}
