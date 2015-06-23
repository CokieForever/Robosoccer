#ifndef TEAMROBOT_H
#define TEAMROBOT_H

#include "newrobocontrol.h"
#include "coordinates.h"
#include "interpreter.h"

class TeamRobot : public NewRoboControl
{

public:
    static void* performCmd_helper(void *context);

    TeamRobot(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer *coordCalib, RawBall *ball);

    Position getDefaultPosition() const;
    void setDefaultPositionX(double x);
    void setDefaultPositionY(double y);
    void setDefaultPosition(Position pos);

    int getMapValue(int i, int j) const;
    const Interpreter::Map& getMap() const;
    bool setMapValue(int i, int j, int val);
    void setMap(const Interpreter::Map &map);

    virtual void setNextCmd(Interpreter *info) = 0;
    virtual void setCmdParam() = 0;
    virtual void* performCmd() = 0;

protected:
    Position m_defaultPos;
    CoordinatesCalibrer *m_coordCalib;
    Interpreter::Map m_map;
    RawBall *m_ball;

private:
    /* Well, nothing. */

};

#endif // TEAMROBOT_H
