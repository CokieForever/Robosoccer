
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
 * @brief The organizing central part of the program. Update and interpretation of game information to set formations, default positions...
 *
 */
class Interpreter
{

public:
    /**
     * @brief KickTurn enum to distinguish the team which performs the kick in penalty shooting and kick off mode
     *
     */
    enum KickTurn
    {
        OUR_TURN,   /**< Own team will perform kick */
        THEIR_TURN  /**< Enemy team will perform kick */
    };

    /**
     * @brief Startegy enum to select predefined formations based on enum value
     *
     */
    enum Strategy
    {
        INIT,   /**< Undefined Strategy for Playmodes: STOP, REFEREE_INIT ... */
        ATK,    /**< Attack formation  */
        DEF,    /**< Defense formation */
        MIX     /**< Mixed formation */
    };

    /**
     * @brief contains important game information
     *
     */
    struct GameData
    {
        eTeam team;             /**< Own team info */
        int oponent_score;      /**< Enemy team goals */
        int our_score;          /**< Own team goals */
        ePlayMode mode;         /**< Playmode */
        Strategy formation;     /**< Own team strategy enum */
        eSide our_side;         /**< Own team side */
        KickTurn turn;          /**< Kick turn information, e.g. for penalty shootout */
    };

    /**
     * @brief data structure for map coordinates
     *
     */
    struct Point
    {
        int x;  /**< x-coordinate in a 2D array */
        int y;  /**< y-coordinate in a 2D array */
    };

    static const double MID_THRESHOLD  = 0.30;  /**< Threshold for switching strategy between ATK and DEF */
    static const int MAP_WIDTH = 100;           /**< Width of the field map */
    static const int MAP_HEIGHT = 80;           /**< Height of the field map */
    static const int MAP_BORDERSIZE = 5;        /**< Bordersize for handling 2d array range*/

    static const int DIR = 8;   /**< Number of possible directions to go at any position */
    static const int DX[DIR];   /**< Constant 1d array for A* algorithm instructions in x direction */
    static const int DY[DIR];   /**< Constant 1d array for A* algorithm instructions in y direction */

    /**
     * @brief alias for 2d a matrix to map
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
    CoordinatesCalibrer* m_cal;     /**< Pointer to a CoordinatesCalibrer class */
    Referee* m_ref;                 /**< Pointer to a Referee class */
    const Goalkeeper* m_gk;         /**< Pointer to a constant Goalkeeper class */
    const PlayerMain* m_p1;         /**< Pointer to a constant PlayerMain class */
    const PlayerTwo* m_p2;          /**< Pointer to a constant PlayerTwo class */
    RawBall* m_ball;                /**< Pointer to a RawBall class */
    const OpponentRobot *m_e1;      /**< Pointer to a constant OpponentRobot class */
    const OpponentRobot *m_e2;      /**< Pointer to a constant OpponentRobot class */
    const OpponentRobot *m_e3;      /**< Pointer to a constant OpponentRobot class */
    GameData m_mode;                /**< Member containing Game information */
    int m_situationId;              /**< Flag for update progress */
    Position m_gkDefaultPosition;   /**< contains default position of the goalkeeper */
    Position m_p1DefaultPosition;   /**< contains default position of the player1 */
    Position m_p2DefaultPosition;   /**< contains default position of the player2 */
    pthread_mutex_t m_mutex;        /**< mutex for updates */
    pthread_cond_t m_cond;          /**< conditional variable for mutex lock */

    Map m_p1Map;                    /**< Map for player1 */
    Map m_p2Map;                    /**< Map for player2 */
    pthread_mutex_t m_p1MapMutex;   /**< mutex for map of player1 */
    pthread_mutex_t m_p2MapMutex;   /**< mutex for map of player2 */

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
