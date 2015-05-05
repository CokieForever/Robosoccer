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
//include the libs from sample code



interpreter::interpreter(int x,Referee *y,Goalkeeper *z) {
	// TODO Auto-generated constructor stub
	team = x;
	ref  = y;
	gk = z;

	playmode = ref->GetPlayMode();
	//p_main = main;
	//p2		= second;

	if (x== 0)
		std::cout<<"We are team blue!"<<std::endl;

	else
		std::cout<<"We are team red! "<<std::endl;

	std::cout<<"Interpreter initialized"<<std::endl;
}

interpreter::~interpreter() {
	// TODO Auto-generated destructor stub
}


bool interpreter::verifyPos(){
	//check if all robots are on their default position and orientation
	(gk->getPos().DistanceTo(gk->defaultPos)< 0.1)? return 1:return 0;

}


void interpreter::setDefaultPos(){
	//sets default position struct depending on game mode of all robots to predefined values
	switch(ref.playMode)
	{
		case BEFORE_PENALTY:
				if((ref.our_side == LEFT_SIDE) && (ref.turn == interpreter::OUR_TURN)) 
				{	
					gk->defaultPos = {-0.3, 0.0};
				}
	
				else if((ref.our_side == RIGHT_SIDE) && (ref.turn == interpreter::OUR_TURN)) 
				{	
					gk->defaultPos = {0.3,0.0};
				}
				else if((ref.our_side == LEFT_SIDE) && (ref.turn == interpreter::NOT_OUR_TURN)) 
				{	
					gk->defaultPos = {-0.3,0.0};
				}
				else
				{	
					gk->defaultPos = {0.3,0.0};
				}
			break;

		case BEFORE_KICK_OFF:
				if((ref->our_side == LEFT_SIDE) && (ref->turn == interpreter::OUR_TURN)) 
				{	
					gk->defaultPos = {-0.3,0.0};
				}
	
				else if((ref.our_side == RIGHT_SIDE) && (ref.turn == interpreter::OUR_TURN)) 
				{	
					gk->defaultPos = {0.3,0.0};
				}
				else if((ref.our_side == LEFT_SIDE) && (ref.turn == interpreter::NOT_OUR_TURN)) 
				{	
					gk->defaultPos = {-0.3,0.0};
				}
				else
				{	
					gk->defaultPos = {0.3,0.0};
				}
			break;
		
		case PLAY_ON:
			//distinguish between ATK and DEF mode 
			//tbd
			break;
		default:
			//states such as referee init, kick_off/penalty -> defpos doesnt change
			break;
	}
}



void interpreter::setPlayMode() {

	playmode = ref->GetPlayMode();

	if (playmode==BEFORE_KICK_OFF)	//if its not working try ref::BEFORE_KICK_OFF
	{
		setSide();
		setTurn();
		if (verifyPos())
			ref->SetReady(team);

	}

	else if (playmode==BEFORE_PENALTY)
	{
		setTurn();

	}
	/*
	 *
	 * 	other playmodes and their actions need to be defined
	 *
	 */

}



void interpreter::setSide(){

	eSide tmp_side = ref->GetBlueSide();
	// 0<-> blue 1<->red
	if(((team == 0) && (tmp_side==LEFT_SIDE)) || ((team == 1) && (tmp_side  == RIGHT_SIDE)))
		our_side = LEFT_SIDE;
	else
		our_side = RIGHT_SIDE;

}


void interpreter::setTurn(){
	if (our_side==ref.GetSide())
	{
		turn = OUR_TURN;
	}
	else
	{
		turn = NOT_OUR_TURN;
	}

}

void interpreter::updateSituation(){
	setPlayMode();
	setDefaultPos();

}
