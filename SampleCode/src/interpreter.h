
#ifndef INTERPRETER_H_
#define INTERPRETER_H_


#include "referee.h"
#include "goalkeeper.h"
#include "share.h"


class interpreter {

enum kick_turn{OUR_TURN,NOT_OUR_TURN};

private:

	Referee* ref;
	//PlayerMain *p1;
	//PlayerSecondary *p2;
	Goalkeeper* gk;


	void setPlayMode();
	void setSide();
	void setTurn();
	bool verifyPos();
	void setDefaultPos();

public:
	int team;
	ePlayMode playmode;
	eSide our_side;
	kick_turn turn;


	void updateSituation();
	interpreter(int,Referee*,Goalkeeper*);
	~interpreter();
};

#endif /* INTERPRETER_H_ */
