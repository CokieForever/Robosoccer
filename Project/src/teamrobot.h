#ifndef TEAMROBOT_H
#define TEAMROBOT_H

#include "newrobocontrol.h"
#include "coordinates.h"
#include "interpreter.h"
#include "pathfinder.h"
#include "matrix.h"
#include <queue>
#include "ballmonitor.h"

class RefereeDisplay;

/**
 * @brief
 *
 */
class TeamRobot : public NewRoboControl
{

public:
    TeamRobot(RTDBConn& DBC, const int deviceNr, const CoordinatesCalibrer *coordCalib, RawBall *ball, BallMonitor *ballPm, RefereeDisplay *display=NULL);
    virtual ~TeamRobot();

    Position getDefaultPosition() const;
    void setDefaultPositionX(double x);
    void setDefaultPositionY(double y);
    void setDefaultPosition(Position pos);

    const CoordinatesCalibrer* getCoordinatesCalibrer() const;
    RawBall* getBall() const;

    int getMapValue(int i, int j) const;
    const Interpreter::Map& getMap() const;
    bool setMapValue(int i, int j, int val);
    void setMap(const Interpreter::Map &map);

    void GiveDisplay(RefereeDisplay *display);

    void KickOff(const NewRoboControl* otherRobots[5], eSide ourSide, bool likePenalty = false);
    void KickPenalty(const NewRoboControl* otherRobots[5]);
    bool Rotation(Position ballPos);
    void KickBall(Position ballPos, bool rotate=true, bool forward=true);
    void KickMovingBall(RawBall *ball);
    bool ShouldKick(Position ballPos, Position goalPos);
    bool ShouldGoalKick(Position ballPos, eSide ourSide);

    virtual void setNextCmd(const Interpreter::GameData& info) = 0;
    virtual void setCmdParam(const Interpreter& interpreter) = 0;
    virtual void performCmd(const Interpreter::GameData& info) = 0;

protected:
    static bool IsPathOK(PathFinder::Path path, PathFinder::Point& tgt);
    static double AngleDiff(double angle1, double angle2);

    Position m_defaultPos;                                          /**< TODO */
    const CoordinatesCalibrer *m_coordCalib;                        /**< TODO */
    Interpreter::Map m_map;                                         /**< TODO */
    RawBall *m_ball;                                                /**< TODO */
    PathFinder m_pathFinder;                                        /**< TODO */
    RefereeDisplay *m_display;                                      /**< TODO */
    const PathFinder::ConvexPolygon* m_roboObstacles[5];            /**< TODO */
    Position m_roboObstaclePos[5];                                  /**< TODO */
    const PathFinder::ConvexPolygon* m_ballObstacles[3];            /**< TODO */
    Position m_ballObstaclePos;                                     /**< TODO */
    const PathFinder::ConvexPolygon* m_penaltyAreaObstacles[2];     /**< TODO */
    const PathFinder::ConvexPolygon* m_borderObstacles[4];          /**< TODO */
    PathFinder::Path m_pathFinderPath;                              /**< TODO */
    Interpreter::Strategy m_prevFormation;                          /**< TODO */
    const PathFinder::ConvexPolygon* m_areaObstacle;                /**< TODO */
    string m_path;                                                  /**< TODO */
    queue<int> m_q;                                                 /**< TODO */
    bool m_actionPerformed;                                         /**< TODO */
    int m_go_x;                                                     /**< TODO */
    int m_go_y;                                                     /**< TODO */
    BallMonitor *m_ballPm;                                          /**< TODO */

    void AddBorderObstaclesToPathFinder(bool small = false);
    void UpdatePathFinder(const NewRoboControl* obstacles[5], const Interpreter::GameData& info);
    void ComputePath(const Interpreter& interpreter);
    void FollowPath(const Interpreter::GameData& info);

    virtual void AddObstacleForFormation(const Interpreter::GameData& info) = 0;

private:
    /* Well, nothing. */

};

#endif // TEAMROBOT_H
