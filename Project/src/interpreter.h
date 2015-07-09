
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

/**
 * @brief
 *
 */
class Interpreter
{

public:
    /**
     * @brief
     *
     */
    enum KickTurn
    {
        OUR_TURN,   /**< TODO */
        THEIR_TURN  /**< TODO */
    };

    /**
     * @brief
     *
     */
    enum Strategy
    {
        ATK,    /**< TODO */
        DEF,    /**< TODO */
        MIX     /**< TODO */
    };

    /**
     * @brief
     *
     */
    struct GameData
    {
        eTeam team;             /**< TODO */
        int oponent_score;      /**< TODO */
        int our_score;          /**< TODO */
        ePlayMode mode;         /**< TODO */
        Strategy formation;     /**< TODO */
        eSide our_side;         /**< TODO */
        KickTurn turn;          /**< TODO */
    };

    /**
     * @brief
     *
     */
    struct Point
    {
        int x;  /**< TODO */
        int y;  /**< TODO */
    };

    static const double MID_THRESHOLD  = 0.30;  /**< TODO */
    static const int MAP_WIDTH = 100;           /**< TODO */
    static const int MAP_HEIGHT = 80;           /**< TODO */
    static const int MAP_BORDERSIZE = 5;        /**< TODO */

    static const int DIR = 8;   /**< Number of possible directions to go at any position */
    static const int DX[DIR];   /**< TODO */
    static const int DY[DIR];   /**< TODO */

    /**
     * @brief
     *
     */
    typedef Matrix Map;

    static string pathFind(Map map, Point start, Point finish);
    static Point* getCheckPoints(Point start, string path);
    static int coord2mapX(double);
    static int coord2mapY(double);
    static double map2coordX(int);
    static double map2coordY(int);
    static void showMap(const Map& map, string path, Point start);
    static void matrixupdate(Map& map, const NewRoboControl* ref, const NewRoboControl* obstacles[5], RawBall* ball, CoordinatesCalibrer* coordCalibrer, eSide our_side);
    static void maskOmitLowerRight(Map &map);
    static void maskOmitLowerLeft(Map &map);
    static void maskOmitUpperRight(Map &map);
    static void maskOmitUpperLeft(Map &map);
    static void maskOmitLeft(Map &map);
    static void maskOmitRight(Map &map);
    void formationUpdateP1(Map& map);
    void formationUpdateP2(Map& map);
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
    CoordinatesCalibrer* m_cal;     /**< TODO */
    Referee* m_ref;                 /**< TODO */
    const Goalkeeper* m_gk;         /**< TODO */
    const PlayerMain* m_p1;         /**< TODO */
    const PlayerTwo* m_p2;          /**< TODO */
    RawBall* m_ball;                /**< TODO */
    const OpponentRobot *m_e1;      /**< TODO */
    const OpponentRobot *m_e2;      /**< TODO */
    const OpponentRobot *m_e3;      /**< TODO */
    GameData m_mode;                /**< TODO */
    int m_situationId;              /**< TODO */
    Position m_gkDefaultPosition;   /**< TODO */
    Position m_p1DefaultPosition;   /**< TODO */
    Position m_p2DefaultPosition;   /**< TODO */
    pthread_mutex_t m_mutex;        /**< TODO */
    pthread_cond_t m_cond;          /**< TODO */

    Map m_p1Map;                    /**< TODO */
    Map m_p2Map;                    /**< TODO */
    pthread_mutex_t m_p1MapMutex;   /**< TODO */
    pthread_mutex_t m_p2MapMutex;   /**< TODO */

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
