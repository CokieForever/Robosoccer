/*
 * interpreter.h
 *
 *  Created on: Apr 26, 2015
 *      Author: dean
 */

#ifndef INTERPRETER_H_
#define INTERPRETER_H_


#include "referee.h"
#include "share.h"



struct DefPos{
	double posX  =	0.0;
	double posY  =	0.0;
	double angle = 	0.0;

};


class interpreter {

enum kick_turn{OUR_TURN,NOT_OUR_TURN};

private:

	Referee& ref;
	//Player_main& p1;
	//Player_second &p2;
	//Goalkeeper& gk_main;


	void setPlayMode();
	void setSide();
	void setTurn();
	//bool verifyPos();
	void setDefaultPos();

public:
	int team;
	ePlayMode playmode;
	eSide our_side =LEFT_SIDE;
	kick_turn turn = NOT_OUR_TURN;


	void updateSituation();
	interpreter(int,Referee&);
	virtual ~interpreter();
};

#endif /* INTERPRETER_H_ */
