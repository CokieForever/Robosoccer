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
    void KickBall(Position ballPos);
    void KickMovingBall(RawBall *ball);
    bool ShouldKick(Position ballPos, Position goalPos);
    bool ShouldGoalKick(Position ballPos, eSide ourSide);

    virtual void setNextCmd(const Interpreter::GameData& info) = 0;
    virtual void setCmdParam(const Interpreter& interpreter) = 0;
    virtual void performCmd(const Interpreter::GameData& info) = 0;

protected:
    static bool IsPathOK(PathFinder::Path path, PathFinder::Point& tgt);
    static double AngleDiff(double angle1, double angle2);

    Position m_defaultPos;
    const CoordinatesCalibrer *m_coordCalib;
    Interpreter::Map m_map;
    RawBall *m_ball;
    PathFinder m_pathFinder;
    RefereeDisplay *m_display;
    const PathFinder::ConvexPolygon* m_roboObstacles[5];
    Position m_roboObstaclePos[5];
    const PathFinder::ConvexPolygon* m_ballObstacles[3];
    Position m_ballObstaclePos;
    const PathFinder::ConvexPolygon* m_penaltyAreaObstacles[2];
    const PathFinder::ConvexPolygon* m_borderObstacles[4];
    PathFinder::Path m_pathFinderPath;
    Interpreter::Strategy m_prevFormation;
    const PathFinder::ConvexPolygon* m_areaObstacle;
    string m_path;
    queue<int> m_q;
    bool m_actionPerformed;
    int m_go_x,m_go_y;
    BallMonitor *m_ballPm;

    void AddBorderObstaclesToPathFinder(bool small = false);
    void UpdatePathFinder(const NewRoboControl* obstacles[5], const Interpreter::GameData& info);
    void ComputePath(const Interpreter& interpreter);
    void FollowPath(const Interpreter::GameData& info);

    virtual void AddObstacleForFormation(const Interpreter::GameData& info) = 0;

private:
    /* Well, nothing. */

};

#endif // TEAMROBOT_H
