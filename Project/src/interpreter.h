
#ifndef INTERPRETER_H_
#define INTERPRETER_H_
#define WIDTH 100
#define HEIGHT 80
#define BORDERSIZE 5


#include <time.h>
#include <iostream>
#include <pthread.h>
#include "kogmo_rtdb.hxx"
#include "robo_control.h"
#include "referee.h"
#include "goalkeeper.h"
#include "playermain.h"
#include "playertwo.h"


static const int dir=8; // number of possible directions to go at any position
// if dir==8
static const int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
static const int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};

typedef struct{
    int x;
    int y;
}Point;



class interpreter {



private:
        CoordinatesCalibrer *cal;
        Referee *ref;
        Goalkeeper *gk;
        PlayerMain *p1;
        PlayerTwo *p2;
        RawBall *ball;
        RoboControl *e1,*e2,*e3;

        void setPlayMode();
	void setSide();
	void setTurn();
        void setScores();
	bool verifyPos();
	void setDefaultPos();

public:
        enum kick_turn{OUR_TURN,THEIR_TURN};
        enum strategy{ATK,DEF,MIX};

        typedef struct{
            int team,oponent_score,our_score;
            ePlayMode mode;
            strategy formation;
            eSide our_side;
            kick_turn turn;
        }gameData;

        static const double MID_THRESHOLD  = 0.30;

        gameData mode;

        void setObstacles(int map[][HEIGHT]);

        interpreter(int x,Referee *y,Goalkeeper *z,PlayerMain *p,PlayerTwo *t,RoboControl *a,RoboControl *b,RoboControl *c,RawBall *d,CoordinatesCalibrer *e);

        void updateSituation();
        ~interpreter();
};

string pathFind( int map[][HEIGHT], const Point A,const Point B);

Point* getCheckPoints(Point start,string path);

int coord2mapX(double);
int coord2mapY(double);
double map2coordX(int);
double map2coordY(int);
void showMap(int map[][HEIGHT],string path,const Point start);
void matrixupdate(int newMatrix[][HEIGHT],RoboControl *ref, RoboControl *obstacles[5], RawBall *ball, CoordinatesCalibrer *coordCalibrer, eSide our_side);

#endif /* INTERPRETER_H_ */
