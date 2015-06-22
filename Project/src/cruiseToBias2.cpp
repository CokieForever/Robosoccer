
//----------------------------------------Function bodys----------------------------------------------------
//----------------------------------------------------------------------------------------------------------
#include "cruiseToBias2.h"

bool CruisetoBias(double tarX, double tarY, int speed, double tarP, double varDir, RoboControl * robo)
{

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
    double posX = robo->GetX();
    double posY = robo->GetY();
    double posP = robo->GetPhi().Rad();  //Hier Roboterwinkel abrufen

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
        cout << "Diff2: " << speedAngleDrive << endl;
        if ((fabs(diffAngle) > variationDirec) && (fabs(diffAngle) < degToRad(90)))  // pi/2 = 1,57079633     Schritt 1.1
        {
            //Roboter rotieren
            setSpeed(0, speedAngle, eDir, robo);
        }
        else if ((fabs(diffAngle)) <= variationDirec)  // Schritt 1.1 erfolgreich, es folgt Schritt 1.2
        {

            setSpeed(speed * getSpeedT(sqrt((diffX * diffX) + (diffY * diffY))), speedAngleDrive, eDir, robo);
            // Schritt 1.2 - 1.4 gleichzeitig!
        }
    }
    else   // Schritt 1: fertig. Roboter ist an Zielposition.
    {
        if (tarP == -10)
        {
            setSpeed(0, 0, FORWARD, robo);
            //StopMovement();
            return true;
        }
        else if ((fabs(getDiffAngle(tarP*M_PI/180, posP))) > variationAngle)
        {
            setSpeed(0, getSpeedP(tarP, posP), getDirection(tarP, posP), robo);
        }
        else
        {
            setSpeed(0, 0, FORWARD, robo);
            //StopMovement();
            return true;
        }
    }

    return false;
}

//Berechnung in welcher Richtung Roboter fahren soll
/**
 * @brief
 *
 * @param nominal
 * @param actual
 * @return Controller::eDirection
 */
eDirection getDirection(double nominal, double actual)
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

//Motorgeschwindigkeit der einzelnen Räder festlegen
/**
 * @brief
 *
 * @param translation
 * @param rotation
 * @param dir
 */
void setSpeed(double translation, double rotation, eDirection dir, RoboControl * robo)
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
    cout << wheelL << "und" << wheelR << endl;
    wheelL = wheelL - rotation * 800.0 / 3.14159265358965;
    wheelR = wheelR + rotation * 800.0 / 3.14159265358965;

    robo->MoveMs(wheelL, wheelR, 50, 0);
}


/**
 * @brief
 *
 * @param nominal
 * @param actual
 * @return double
 */
double getDiffAngle(double nominal, double actual)
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
double getSpeedP(double nominal, double actual) // Drehgeschwindigkeit ruhig
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
double getSpeedPt(double nominal, double actual, int geschw) // Drehgeschwindigkeit bei der Fahrt
{
    double diff = getDiffAngle(nominal, actual);
    double korr = (geschw / 400) * (geschw / 400) * 1.2;
    cout << "Differenz: " << diff << endl;

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
double getSpeedT(double diff)                       // regelt Vorwärtsgeschwindigkeit
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
double degToRad(double deg)
{
    return deg * M_PI / 180.0;
}
