
#ifndef INTERPRETER_H_
#define INTERPRETER_H_

class PlayerMain;
class PlayerTwo;
class Goalkeeper;

#include <time.h>
#include <iostream>
#include <pthread.h>
#include "kogmo_rtdb.hxx"
#include "referee.h"
#include "coordinates.h"
#include "opponentrobot.h"
#include "SDL.h"
#include "matrix.h"

class TeamRobot;

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
        eTeam team;
        int oponent_score, our_score;
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
    static const int MAP_WIDTH = 100;
    static const int MAP_HEIGHT = 80;
    static const int MAP_BORDERSIZE = 5;

    static const int DIR = 8; // number of possible directions to go at any position
    static const int DX[DIR];
    static const int DY[DIR];

    typedef Matrix Map;

    static string pathFind(Map map, Point start, Point finish);
    static Point* getCheckPoints(Point start, string path);
    static int coord2mapX(double);
    static int coord2mapY(double);
    static double map2coordX(int);
    static double map2coordY(int);
    static void showMap(const Map& map, string path, Point start);
    static void matrixupdate(Map& map, const NewRoboControl* ref, const NewRoboControl* obstacles[5], RawBall* ball, CoordinatesCalibrer* coordCalibrer, eSide our_side);

    static void maskLowerRight(Map &map);
    static void maskLowerLeft(Map &map);
    static void maskUpperRight(Map &map);
    static void maskUpperLeft(Map &map);
    static void maskOmitLowerRight(Map &map);
    static void maskOmitLowerLeft(Map &map);
    static void maskOmitUpperRight(Map &map);
    static void maskOmitUpperLeft(Map &map);
    static void maskLeft(Map &map);
    static void maskRight(Map &map);

    Interpreter(eTeam x, Referee* y, Goalkeeper* z, PlayerMain* p, PlayerTwo* t, OpponentRobot* a, OpponentRobot* b, OpponentRobot* c, RawBall* d, CoordinatesCalibrer* e);
    ~Interpreter();

    void End();

    GameData getMode() const;

    void SetP1MapToRobot(TeamRobot *p1) const;
    void SetP2MapToRobot(TeamRobot *p2) const;

    Position getGKDefaultPos() const;
    Position getP1DefaultPos() const;
    Position getP2DefaultPos() const;

    const Goalkeeper* getGK() const;
    const PlayerMain* getP1() const;
    const PlayerTwo* getP2() const;
    const OpponentRobot* getE1() const;
    const OpponentRobot* getE2() const;
    const OpponentRobot* getE3() const;

    void updateSituation();
    int waitForUpdate(int id);

private:
    CoordinatesCalibrer* m_cal;
    Referee* m_ref;
    const Goalkeeper* m_gk;
    const PlayerMain* m_p1;
    const PlayerTwo* m_p2;
    RawBall* m_ball;
    const OpponentRobot* m_e1, *m_e2, *m_e3;
    GameData m_mode;
    int m_situationId;
    Position m_gkDefaultPosition;
    Position m_p1DefaultPosition;
    Position m_p2DefaultPosition;
    pthread_mutex_t m_mutex;
    pthread_cond_t m_cond;

    Map m_p1Map;
    Map m_p2Map;
    pthread_mutex_t m_p1MapMutex;
    pthread_mutex_t m_p2MapMutex;

    void setPlayMode();
    void setSide();
    void setTurn();
    void setScores();
    bool verifyPos();
    void setDefaultPos();

    void formationUpdateP1();
    void formationUpdateP2();

};




#endif /* INTERPRETER_H_ */
