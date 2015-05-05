/*
 * interpreter.cpp
 *
 *  Created on: Apr 26, 2015
 *      Author: dean
 */

#include "interpreter.h"
#include <iostream>
#include "share.h"
//include the libs from sample code



Goalkeeper::Goalkeeper(RoboControl *x,Rawball *b) {
	robot = x;
	ball  = b;

}

Goalkeeper::~Goalkeeper(){
}

Goalkeeper::setNextCmd(Interpreter *info){
	switch(info->playMode)
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


Goalkeeper::setCmdParam(){
	switch(nextCmd)
	{
		case PREVENT_GOAL:
			
			Position ballPos = roboBall->ball->GetPos();
			double y = ballPos.GetY();

			if (y > 0.15)
				y = 0.15;
			else if (y < -0.15)
				y = -0.15;

			double deltaY = fabs(robot->GetY() - y);

			if (deltaY >= 0.05)
			{
				cout << "Goal keeper moving to y = " << y << std::endl;
				preventGoalParam = {defaultPos.x ,y};
			}
			else
				preventGoalParam = defaultPos;
			break;
		default:
			break;

	}
}



Goalkeeper::performCmd(){
	switch(nextCmd)
	{
		case PREVENT_GOAL:
			robot->GoToXY(preventGoalParam);
			break;
		case GO_TO_DEF_POS:
			robot->GoToXY(defaultPos);
			break;
		default:
			break;
	}

}
