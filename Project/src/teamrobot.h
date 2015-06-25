#ifndef TEAMROBOT_H
#define TEAMROBOT_H

#include "newrobocontrol.h"
#include "coordinates.h"
#include "interpreter.h"
#include "pathfinder.h"
#include "refereedisplay.h"

class TeamRobot : public NewRoboControl
{

public:
    static void* performCmd_helper(void *context);

    TeamRobot(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer *coordCalib, RawBall *ball, RefereeDisplay *display=NULL);
    virtual ~TeamRobot();

    Position getDefaultPosition() const;
    void setDefaultPositionX(double x);
    void setDefaultPositionY(double y);
    void setDefaultPosition(Position pos);

    CoordinatesCalibrer* getCoordinatesCalibrer() const;
    RawBall* getBall() const;

    int getMapValue(int i, int j) const;
    const Interpreter::Map& getMap() const;
    bool setMapValue(int i, int j, int val);
    void setMap(const Interpreter::Map &map);

    void UpdatePathFinder(NewRoboControl *const obstacles[5], eSide our_side);

    virtual void setNextCmd(Interpreter *info) = 0;
    virtual void setCmdParam() = 0;
    virtual void* performCmd() = 0;

protected:
    Position m_defaultPos;
    CoordinatesCalibrer *m_coordCalib;
    Interpreter::Map m_map;
    RawBall *m_ball;
    PathFinder m_pathFinder;
    RefereeDisplay *m_display;
    const PathFinder::ConvexPolygon* m_roboObstacles[5];
    Position m_roboObstaclePos[5];
    PathFinder::Path m_pathFinderPath;

private:
    /* Well, nothing. */

};

#endif // TEAMROBOT_H
