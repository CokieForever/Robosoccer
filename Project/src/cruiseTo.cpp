//------------------------------------------------------------------------------------------------------------
// P-Regler vom Christian Spies 2013

/** geregelte gotoPos Alternative**/

/**
 ** tarX und tarY = target position
 ** tarP = target angle
 **/


/**
 * @brief
 *
 * @param tarX
 * @param tarY
 * @param tarP
 * @return bool
 */

bool Controller::cruiseTo(double tarX, double tarY, double tarP)
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
  double posX = m_robot->GetX();
  double posY = m_robot->GetY();
  double posP = m_robot->GetPhi().Rad();  //Hier Roboterwinkel abrufen

  const double variationTrans = 0.025;                              // maximale Abweichung zum Ziel (in Meter)
  const double variationAngle = degToRad(10);                       // maximale Drehabweichung am Ende (in Rad)  degToRad wandelt Grad in Rad um
  const double variationDirec = degToRad(44);                       // maximale Drehabweichung am Start, bevor Roboter anfängt zu fahren


  //Abweichungen berechnen
  double diffX = tarX - posX;
  double diffY = tarY - posY;
  double diffP = atan2(diffY, diffX);

  //Berechnen ob Roboter vorwärts oder rückwärts fahren soll
  //eDirection ist ein enum hier
  eDirection dir;
  dir =  getDirection(diffP, posP);


  if (!((fabs(diffX) < variationTrans) && (fabs(diffY) < variationTrans)))
    // Schritt 1: Roboter noch nicht an Zielposition?
  {


    if (fabs(getDiffAngle(diffP, posP)) > variationDirec && fabs(getDiffAngle(diffP, posP)) < degToRad(90))  // Schritt 1.1
    {
   
      //Roboter rotieren
      setSpeed(0, getSpeedP(diffP, posP));
    }


    else if ((fabs(getDiffAngle(diffP, posP))) <= variationDirec)  // Schritt 1.1 erfolgreich, es folgt Schritt 1.2
    {

      setSpeed(600 * getSpeedT(sqrt((diffX * diffX) + (diffY * diffY))), getSpeedPt(diffP, posP), getDirection(diffP, posP));
      // Schritt 1.2 - 1.4 gleichzeitig!
    }


  }
  else   // Schritt 1: fertig. Roboter ist an Zielposition.
  {
    if ((fabs(getDiffAngle(tarP, posP))) > variationAngle)
    {
      setSpeed(0, getSpeedP(tarP, posP));
    }
    else
    {
      setSpeed(0, 0);
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
Controller::eDirection Controller::getDirection(double nominal, double actual)
{
  double diffNormal = nominal - actual;
  if ((diffNormal) < (-M_PI)) diffNormal = diffNormal + 2 * M_PI;
  if ((diffNormal) > (+M_PI)) diffNormal = diffNormal - 2 * M_PI;

  double diffBackward = nominal + 3.14 - actual;
  if ((diffBackward) < (-M_PI)) diffBackward = diffBackward + 2 * M_PI;
  if ((diffBackward) > (+M_PI)) diffBackward = diffBackward - 2 * M_PI;

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
void Controller::setSpeed(double translation, double rotation, eDirection dir)
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
  wheelL -= rotation * 800.0 / 3.14159265358965;
  wheelR += rotation * 800.0 / 3.14159265358965;

  m_robot->MoveMs(wheelL, wheelR, 300, 0);
}


/**
 * @brief
 *
 * @param nominal
 * @param actual
 * @return double
 */
double Controller::getDiffAngle(double nominal, double actual)
{
  double diffA = (nominal - actual);
  if (getDirection(nominal, actual) == BACKWARD) diffA += M_PI;
  if ((diffA) < (-M_PI)) diffA = diffA + 2 * M_PI;
  if ((diffA) > (+M_PI)) diffA = diffA - 2 * M_PI;
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
double Controller::getSpeedP(double nominal, double actual) // Drehgeschwindigkeit ruhig
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
double Controller::getSpeedPt(double nominal, double actual) // Drehgeschwindigkeit bei der Fahrt
{
  double diff = getDiffAngle(nominal, actual);

    if (std::fabs(diff) <= 1.57)
    {
      if (std::fabs(diff) < 0.1745)
      {
        //return diff * 0.1;
        return diff * 0.12;
      }
      else
      {
        //return diff * 0.18;
        return diff * 0.18;
      }
    }
    else
    {
      //return diff * 0.18;
      return diff * 0.18;
    }
  
}
/**
 * @brief
 *
 * @param diff
 * @return double
 */
double Controller::getSpeedT(double diff)                       // regelt Vorwärtsgeschwindigkeit
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


double Controller::degToRad(double deg)
{
  return deg * M_PI / 180.0;
}
