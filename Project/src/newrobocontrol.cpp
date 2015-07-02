
//----------------------------------------Function bodys----------------------------------------------------
//----------------------------------------------------------------------------------------------------------
#include "newrobocontrol.h"
#include "pathfinder.h"
#include "log.h"
#include "sdlutilities.h"

//Berechnung in welcher Richtung Roboter fahren soll
/**
 * @brief
 *
 * @param nominal
 * @param actual
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
 * @brief
 *
 * @param nominal
 * @param actual
 * @return double
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

bool NewRoboControl::IsOnTarget(Position current, Position target)
{
    return fabs(current.GetX()-target.GetX()) < 0.05 && fabs(current.GetY()-target.GetY()) < 0.05;
}


NewRoboControl::NewRoboControl(RTDBConn &DBC, const int deviceNr) : RoboControl(DBC, deviceNr)
{
    m_stopCruisingNow = false;
    m_isCruising = false;
    pthread_create(&m_thread, NULL, Checkspeed, this);
}

//The destructor is empty but is there only to prevent class instantation (see newrobocontrol.h)
NewRoboControl::~NewRoboControl()
{
    m_checkSpeedFinishNow = true;
    pthread_join(m_thread, NULL);
}

void* NewRoboControl::Checkspeed(void *data)
{
    NewRoboControl *robo = (NewRoboControl*)data;
    robo->m_checkSpeedFinishNow = false;
    /*
    double speed_current;
    bool speed_check;
    while (!robo->m_checkSpeedFinishNow)
    {
        speed_current = sqrt(robo->GetSpeedLeft()*robo->GetSpeedLeft()+robo->GetSpeedRight()*robo->GetSpeedRight());
        speed_check=(abs(speed_current-robo->m_targetSpeed)>0.1);
        cout<<"current speed is "<<speed_current<<endl<<endl;
        cout<<"check speed is "<<speed_check<<endl<<endl;
        cout<<"collision detection"<<endl<<endl;

        if (speed_check)
        {
            robo->RandomMove();
        }
    }
    */
    /*
    Position previous;
    Position current;
    while(!robo->m_checkSpeedFinishNow)
    {
        previous = robo->GetPos();
        usleep(100000);
        current = robo->GetPos();
        cout<<"Collision detection."<<endl<<endl;
        if(((abs(previous.GetX()-current.GetX())+abs(previous.GetY()-current.GetY()))<0.01) && (robo->m_targetSpeed!=0))
        {
            robo->RandomMove();
            cout<<"Robot is random moving."<<endl<<endl;
        }
    }
    */
    Position current;
    while(!robo->m_checkSpeedFinishNow)
    {
        current = robo->GetPos();
        usleep(5000000);
        cout<<"Collision detection."<<endl<<endl;
        if(!robo->IsOnTarget(robo->m_targetPos))
        //if(((abs(robo->m_targetPos.GetX()-current.GetX())+abs(robo->m_targetPos.GetY()-current.GetY()))>0.2) && (robo->m_targetSpeed!=0))
        {
            robo->RandomMove();
            cout<<"Robot is random moving."<<endl<<endl;
        }
    }
    return 0;
}


void NewRoboControl::driveBack()
{
    double posX=rand()%20;
    double posY=rand()%20;
    posX=posX/10-1;
    posY=posY/10-1;
    //double posP = GetPhi().Rad();
    GotoXY(0,0,160,false);
    Log("collision detection", INFO);
    usleep(10000);
}

bool NewRoboControl::IsOnTarget(Position target) const
{
    return IsOnTarget(GetPos(), target);
}

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

void NewRoboControl::RandomMove()
{
    MoveMs((rand() % 11 - 5) * 100, (rand() % 11 -5) * 100, 250);
}

Position* NewRoboControl::drivePath(std::vector<Position>* path)
{
    for (std::vector<Position>::iterator it = path->begin() ; it != path->end() ; it++)
    {
        Position *pos = &(*it);
        if (!IsOnTarget(*pos))
            return pos;
    }
    return NULL;
}

void NewRoboControl::Kick(eDirection dir)
{
    if (dir == FORWARD)
        MoveMsBlocking(200, 200, 500);
    else
        MoveMsBlocking(-200, -200, 500);
}

void NewRoboControl::GoalKick(Position ballPos)
{
    double phi = GetPhi().Get();

    double robotBallAngle;
    Position pos = GetPos();
    ComputeLineAngle(pos.GetX(), pos.GetY(), ballPos.GetX(), ballPos.GetY(), &robotBallAngle);

    double diff1 = AngleDiff(robotBallAngle, phi);

    double robotBallAngle2 = robotBallAngle<=0 ? robotBallAngle+M_PI : robotBallAngle-M_PI;
    double diff2 = AngleDiff(robotBallAngle2, phi);

    bool forward = fabs(diff1) < fabs(diff2);
    double a = forward ? robotBallAngle : robotBallAngle2;
    double diff3 = AngleDiff(a, phi);

    int i;
    for (i=0 ; i < 1000 && fabs(diff3) >= 10 * M_PI / 180 ; i++)
    {
        if (diff3 > 0)
            MoveMs(50, -50, 200);
        else if (diff3 < 0)
            MoveMs(-50, 50, 200);

        usleep(1000);

        phi = GetPhi().Get();
        diff3 = AngleDiff(a, phi);
    }

    if (i < 1000)
        Kick(forward ? FORWARD : BACKWARD);
}

int NewRoboControl::ShouldKick(Position ballPos, Position goalPos)
{
    Position pos = GetPos();
    if (pos.DistanceTo(ballPos) <= 0.2)
    {
        double robotGoalAngle, robotBallAngle;

        ComputeLineAngle(pos.GetX(), pos.GetY(), goalPos.GetX(), goalPos.GetY(), &robotGoalAngle);
        ComputeLineAngle(pos.GetX(), pos.GetY(), ballPos.GetX(), ballPos.GetY(), &robotBallAngle);
        double diff1 = AngleDiff(robotBallAngle, robotGoalAngle );

        if (fabs(diff1) <= 10 * M_PI / 180 )
        {
            double robotAngle = GetPhi().Get();
            double diff2 = AngleDiff(robotAngle, robotBallAngle);

            if (fabs(diff1 - diff2) <= 20 * M_PI / 180)
            {
                Log("Kick forwards!", INFO);
                Log("Diff1 = " + ToString(diff1), DEBUG);
                Log("Diff2 = " + ToString(diff2), DEBUG);
                return 1;  //Kick forwards
            }
            else
            {
                robotAngle = robotAngle<=0 ? robotAngle+M_PI : robotAngle-M_PI;
                diff2 = AngleDiff(robotAngle, robotBallAngle);

                if (fabs(diff1 - diff2) <= 20 * M_PI / 180)
                    return -1;   //Kick backwards
            }
        }
    }

    return 0;   //Don't kick
}

bool NewRoboControl::ShouldGoalKick(Position ballPos, eSide ourSide)
{
    double robotBallAngle;
    Position pos = GetPos();

    if (pos.DistanceTo(ballPos) <= 0.2)
    {
        ComputeLineAngle(pos.GetX(), pos.GetY(), ballPos.GetX(), ballPos.GetY(), &robotBallAngle);
        if (ourSide == RIGHT_SIDE)
            robotBallAngle = robotBallAngle<=0 ? robotBallAngle+M_PI : robotBallAngle-M_PI;

        return fabs(robotBallAngle) <= 45 * M_PI / 180;
    }

    return false;
}

double NewRoboControl::AngleDiff(double angle1, double angle2)
{
    double diff = fmod(angle1 - angle2 + 2*M_PI, 2*M_PI);
    if (diff >= M_PI)
        diff -= 2*M_PI;
    return diff;
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
    wheelR = wheelR + rotation * 800.0 / 3.14159265358965;

    MoveMs(wheelL, wheelR, 50, 0);
}


