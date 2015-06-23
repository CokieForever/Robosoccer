
#ifndef INTERPRETER_H_
#define INTERPRETER_H_

class PlayerMain;
class PlayerTwo;
class Goalkeeper;

#include <time.h>
#include <iostream>
#include <pthread.h>
#include "kogmo_rtdb.hxx"
#include "robo_control.h"
#include "referee.h"
#include "coordinates.h"


class Interpreter
{

public:
    enum KickTurn
    {
        OUR_TURN, THEIR_TURN
    };

    enum Strategy
    {
        ATK, DEF, MIX
    };

    struct GameData
    {
        int team, oponent_score, our_score;
        ePlayMode mode;
        Strategy formation;
        eSide our_side;
        KickTurn turn;
    };

    struct Point
    {
        int x, y;
    };

    static const double MID_THRESHOLD  = 0.30;
    static const int WIDTH = 100;
    static const int HEIGHT = 80;
    static const int BORDERSIZE = 5;

    static const int DIR = 8; // number of possible directions to go at any position
    static const int DX[DIR];
    static const int DY[DIR];

    typedef int (Map)[WIDTH][HEIGHT];

    static string pathFind(Map map, Point start, Point finish);
    static Point* getCheckPoints(Point start, string path);
    static int coord2mapX(double);
    static int coord2mapY(double);
    static double map2coordX(int);
    static double map2coordY(int);
    static void showMap(const Map& map, string path, Point start);
    static void matrixupdate(Map& map, RoboControl* ref, RoboControl* obstacles[5], RawBall* ball, CoordinatesCalibrer* coordCalibrer, eSide our_side);

    Interpreter(int x, Referee* y, Goalkeeper* z, PlayerMain* p, PlayerTwo* t, RoboControl* a, RoboControl* b, RoboControl* c, RawBall* d, CoordinatesCalibrer* e);

    GameData getMode() const;
    void updateSituation();

private:
    CoordinatesCalibrer* m_cal;
    Referee* m_ref;
    Goalkeeper* m_gk;
    PlayerMain* m_p1;
    PlayerTwo* m_p2;
    RawBall* m_ball;
    RoboControl* m_e1, *m_e2, *m_e3;
    GameData m_mode;

    void setPlayMode();
    void setSide();
    void setTurn();
    void setScores();
    bool verifyPos();
    void setDefaultPos();
};




#endif /* INTERPRETER_H_ */
