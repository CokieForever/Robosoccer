
//----------------------------------------Function bodys----------------------------------------------------
//----------------------------------------------------------------------------------------------------------
#include "newrobocontrol.h"
#include "pathfinder.h"
#include "log.h"
#include "geometry.h"

//Berechnung in welcher Richtung Roboter fahren soll
/**
 * @brief with this function, the robot can determine whether it should drive forward or backward, in
 * order to reach the target conveniently
 * @param nominal is the direction that the robot should turn in order to reach the target
 * @param actual is the actual direction of the robot at the moment.
 * @return Controller::eDirection
 */
NewRoboControl::eDirection NewRoboControl::getDirection(double nominal, double actual)
{
    double diffNormal = nominal - actual;
    if ((diffNormal) < (-M_PI))
        diffNormal = diffNormal + 2 * M_PI;
    if ((diffNormal) > (+M_PI))
        diffNormal = diffNormal - 2 * M_PI;

    double diffBackward = nominal + 3.14 - actual;
    if ((diffBackward) < (-M_PI))
        diffBackward = diffBackward + 2 * M_PI;
    if ((diffBackward) > (+M_PI))
        diffBackward = diffBackward - 2 * M_PI;

    if (fabs(diffNormal) <= fabs(diffBackward))
        return FORWARD;
    else
        return BACKWARD;
}

/**
 * @brief this function calculate the angle difference between actual angle of the robot and the angle to
 * which the robot should turn
 * @param nominal is the direction that the robot should turn in order to reach the target
 * @param actual is the actual direction of the robot at the moment.
 * @return double is the result.
 */
double NewRoboControl::getDiffAngle(double nominal, double actual)
{
    double diffA = (nominal - actual);
    if (getDirection(nominal, actual) == BACKWARD)
        diffA += M_PI;
    if ((diffA) < (-M_PI))
        diffA = diffA + 2 * M_PI;
    if ((diffA) > (+M_PI))
        diffA = diffA - 2 * M_PI;
    return diffA;
}


// Regelung

/**
 * @brief
 *
 * @param nominal
 * @param actual
 * @return double
 */
double NewRoboControl::getSpeedP(double nominal, double actual) // Drehgeschwindigkeit ruhig
{

    double diff = getDiffAngle(nominal, actual);
    // if (std::fabs(diff) < 1.57)
    {
        if (std::fabs(diff) < degToRad(19) || std::fabs(diff) > degToRad(161))
        {
            //return 0.45 * diff;
            return 0.4 * diff;
        }
        else if (std::fabs(diff) < degToRad(30) || std::fabs(diff) > degToRad(150))
        {
            //return 0.28 * diff;
            return 0.28 * diff;
        }
        else
        {
            return 0.19 * diff;
        }
    }
}


/**
 * @brief
 *
 * @param nominal
 * @param actual
 * @return double
 */
double NewRoboControl::getSpeedPt(double nominal, double actual, int geschw) // Drehgeschwindigkeit bei der Fahrt
{
    double diff = getDiffAngle(nominal, actual);
    double korr = (geschw / 400) * (geschw / 400) * 1.2;
    Log("Differenz: " + ToString(diff), DEBUG);

    // TODO Create Table

    /*if (std::fabs(diff) > 1.57)
    {
    return diff * 1;
    }
    else if (std::fabs(diff) > 0.5)
    {
    return diff * 0.6;
    }
    else if (std::fabs(diff) > 0.1745)
    {
    return diff * 0.5;
    }
    else
    {
    return diff * 0.6;
    }*/
    if (std::fabs(diff) > 1.57)
    {
        return diff * 0.18 * korr;
    }
    else if (std::fabs(diff) > 0.5)
    {
        return diff * 0.14 * korr;
    }
    else if (std::fabs(diff) > 0.1745)
    {
        return diff * 0.12 * korr;
    }
    else if (std::fabs(diff) > 0.1)
    {
        return diff * 0.11 * korr;
    }
    else if (std::fabs(diff) > 0.05)
    {
        return diff * 0.08 * korr;
    }
    else if (std::fabs(diff) < 0.1745)
    {
        return diff * 0.12 * korr;
    }
    else if (std::fabs(diff) < 0.5)
    {
        return diff * 0.15 * korr;
    }
    else
    {
        return diff * 0.18 * korr;
    }
}

/**
 * @brief
 *
 * @param diff
 * @return double
 */
double NewRoboControl::getSpeedT(double diff)                       // regelt Vorwärtsgeschwindigkeit
{
    if (diff > 0.2)
        return 0.34;
    else if (diff > 0.12)
        return 0.17;
    else
        return 0.09;
}

/**
 * @brief
 *
 * @param deg
 * @return double
 */
double NewRoboControl::degToRad(double deg)
{
    return deg * M_PI / 180.0;
}

/**
 * @brief this function check if the the robot reach the target
 *
 * @param current is the current position of the robot
 * @param target is the target position of the robot
 * @param precise defines how precise the robot should be around the target
 * @return bool return ture if the robot reaches the target
 */
bool NewRoboControl::IsOnTarget(Position current, Position target, bool precise)
{
    double margin = precise ? 0.05 : 0.2;
    return fabs(current.GetX()-target.GetX()) < margin && fabs(current.GetY()-target.GetY()) < margin;
}


/**
 * @brief
 *
 * @param DBC
 * @param deviceNr
 */
NewRoboControl::NewRoboControl(RTDBConn &DBC, const int deviceNr) : RoboControl(DBC, deviceNr)
{
    m_stopCruisingNow = false;
    m_isCruising = false;
    //pthread_create(&m_thread, NULL, Checkspeed, this);
}

//The destructor is empty but is there only to prevent class instantation (see newrobocontrol.h)
/**
 * @brief
 *
 */
NewRoboControl::~NewRoboControl()
{
    //m_checkSpeedFinishNow = true;
    //pthread_join(m_thread, NULL);
}


/**
 * @brief check if the robot has reached its target
 *
 * @param target is The target of the robot at the moment.
 * @param precise defines the precision.
 * @return bool is Ture if the robot is on the target.
 */
bool NewRoboControl::IsOnTarget(Position target, bool precise) const
{
    return IsOnTarget(GetPos(), target, precise);
}

/**
 * @brief move the robot to the target position
 *
 * @param tarX is the x value of target position
 * @param tarY is the y value of target position
 * @param speed the robot should move at this speed
 * @param tarP
 * @param varDir defines whether the robot should move forwards or backwards
 * @return bool
 */
bool NewRoboControl::cruisetoBias(double tarX, double tarY, int speed, double tarP, double varDir)
{
#ifdef SIMULATION   //The cruisetoBias() does not work in simulation

    if (IsOnTarget(Position(tarX, tarY)))
        return true;
    GotoXY(tarX, tarY);
    return false;

#else

    /*   returniert true, wenn Roboter am Ziel angekommen ist. returniert false, wenn Roboter noch unterwegs ist
    **   Ablauf:
    **     1.  Anfahren zur Zielposition
    **     1.1 Ziel anvisieren/andrehen
    **     1.2 Beschleunigen
    **     1.3 Während der Fahrt ggf leichte Richtungskorrekturen on-the-fly
    **     1.4 Kurz vor Zielposition Abbremsvorgang einleiten
    **     2.  Zielwinkel andrehen
    **
    */

    //Position roboterpos = m_robot->GetPos(); //Hier müsst ihr die Roboterposition abrufen
    double posX = GetX();
    double posY = GetY();
    double posP = GetPhi().Rad();  //Hier Roboterwinkel abrufen

    const double variationTrans = 0.05;                              // maximale Abweichung zum Ziel (in Meter)
    const double variationAngle = degToRad(5);                       // maximale Drehabweichung am Ende (in Rad)  degToRad wandelt Grad in Rad um
    const double variationDirec = degToRad(varDir);                       // maximale Drehabweichung am Start, bevor Roboter anfängt zu fahren


    //Abweichungen berechnen
    double diffX = tarX - posX;
    double diffY = tarY - posY;
    double diffP = atan2(diffY, diffX);
    double diffAngle;
    double speedAngle;
    double speedAngleDrive;

    //Berechnen ob Roboter vorwärts oder rückwärts fahren soll
    //eDirection ist ein enum hier
    eDirection eDir =  getDirection(diffP, posP);

    if (!((fabs(diffX) < variationTrans) && (fabs(diffY) < variationTrans)))
    // Schritt 1: Roboter noch nicht an Zielposition?
    {
        diffAngle = getDiffAngle(diffP, posP);
        speedAngle = getSpeedP(diffP, posP);
        speedAngleDrive = getSpeedPt(diffP, posP, speed);
        Log("Diff2: " + ToString(speedAngleDrive), DEBUG);
        if ((fabs(diffAngle) > variationDirec) && (fabs(diffAngle) < degToRad(90)))  // pi/2 = 1,57079633     Schritt 1.1
        {
            //Roboter rotieren
            setSpeed(0, speedAngle, eDir);
        }
        else if ((fabs(diffAngle)) <= variationDirec)  // Schritt 1.1 erfolgreich, es folgt Schritt 1.2
        {

            setSpeed(speed * getSpeedT(sqrt((diffX * diffX) + (diffY * diffY))), speedAngleDrive, eDir);
            // Schritt 1.2 - 1.4 gleichzeitig!
        }
    }
    else   // Schritt 1: fertig. Roboter ist an Zielposition.
    {
        if (tarP == -10)
        {
            setSpeed(0, 0, FORWARD);
            //StopMovement();
            return true;
        }
        else if ((fabs(getDiffAngle(tarP*M_PI/180, posP))) > variationAngle)
        {
            setSpeed(0, getSpeedP(tarP, posP), getDirection(tarP, posP));
        }
        else
        {
            setSpeed(0, 0, FORWARD);
            //StopMovement();
            return true;
        }
    }

    return false;

#endif
}

/**
 * @brief move the robot to a random position
 *
 */
void NewRoboControl::RandomMove()
{
    MoveMs((rand() % 11 - 5) * 50, (rand() % 11 -5) * 50, 250, 0);
}

/**
 * @brief
 *
 * @param path
 * @return bool
 */
bool NewRoboControl::drivePath(std::vector<Position>* path)
{
    Position *target = NULL;
    for (std::vector<Position>::iterator it = path->begin() ; it != path->end() ; it++)
    {
        target = &(*it);
        if (!IsOnTarget(*target, false))
        {
            double d = 0;
            Position prevPos = GetPos();
            for (; it != path->end() ; it++)
            {
                Position pos = *it;
                d += pos.DistanceTo(prevPos);
                prevPos = pos;
            }

            return cruisetoBias(target->GetX(), target->GetY(), 500 + 1000*d);
        }
    }

    if (!target || IsOnTarget(*target))
    {
        RandomMove();
        return true;
    }

    return cruisetoBias(target->GetX(), target->GetY(), 600);
}



//Motorgeschwindigkeit der einzelnen Räder festlegen
/**
 * @brief
 *
 * @param translation
 * @param rotation
 * @param dir
 */
void NewRoboControl::setSpeed(double translation, double rotation, eDirection dir)
{
    double wheelL = 0, wheelR = 0;

    if (dir == FORWARD)
    {
        wheelL = wheelR = translation;
    }
    else if (dir == BACKWARD)
    {
        wheelL = wheelR = -translation;
    }
    Log(ToString(wheelL) + " und " + ToString(wheelR), DEBUG);
    wheelL = wheelL - rotation * 800.0 / 3.14159265358965;
    wheelR = wheelR + rotation * 800.0 / 3.14159265358965;l

    MoveMs(wheelL, wheelR, 50, 0);
}


