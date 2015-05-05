
#ifndef GOALKEEPER_H_
#define GOALKEEPER_H_


#include "interpreter.h"
#include "share.h"


class Goalkeeper {
	enum ActionGk{GO_TO_DEF_POS,PREVENT_GOAL};


private:

	Rawball* ball;
	Robocontrol* robot;
	ActionGk nextCmd;
	Postition preventGoalParam;

public:
	Position defaultPos;

	void setNextCmd(Interpreter*);
	void setCmdParam();
	void performCmd();
	Goalkeeper(Robocontrol*);
	~Goalkeeper();
};

#endif /* GOALKEEPER_H_ */
