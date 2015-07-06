#include "geometry.h"
#include "pathfinder.h"

/**
 * @brief
 *
 * @param startX
 * @param startY
 * @param angle
 * @param length
 * @param endX
 * @param endY
 */
void ComputeVectorEnd(double startX, double startY, double angle, double length, double *endX, double *endY)
{
    ComputeVectorEnd(startX, startY, cos(angle), sin(angle), length, endX, endY);
}

/**
 * @brief
 *
 * @param startX
 * @param startY
 * @param cosAngle
 * @param sinAngle
 * @param length
 * @param endX
 * @param endY
 */
void ComputeVectorEnd(double startX, double startY, double cosAngle, double sinAngle, double length, double *endX, double *endY)
{
    *endX = startX + cosAngle * length;
    *endY = startY + sinAngle * length;
}

/**
 * @brief
 *
 * @param startX
 * @param startY
 * @param endX
 * @param endY
 * @param angle
 */
void ComputeLineAngle(double startX, double startY, double endX, double endY, double *angle)
{
    double cosAngle, sinAngle;
    ComputeLineAngle(startX, startY, endX, endY, &cosAngle, &sinAngle);
    *angle = acos(cosAngle);
    if (sinAngle < 0)
        *angle = -(*angle);
}

/**
 * @brief
 *
 * @param startX
 * @param startY
 * @param endX
 * @param endY
 * @param cosAngle
 * @param sinAngle
 */
void ComputeLineAngle(double startX, double startY, double endX, double endY, double *cosAngle, double *sinAngle)
{
    double d = sqrt((endX-startX)*(endX-startX) + (endY-startY)*(endY-startY));
    *cosAngle = (endX - startX) / d;
    *sinAngle = (endY - startY) / d;
}

/**
 * @brief
 *
 * @param startX
 * @param startY
 * @param endX
 * @param endY
 * @param thickness
 * @param (rectX)[]
 * @param (rectY)[]
 */
void ComputeThickLineRect(double startX, double startY, double endX, double endY, double thickness, double (&rectX)[4], double (&rectY)[4])
{
    double cosAngle, sinAngle;
    ComputeLineAngle(startX, startY, endX, endY, &cosAngle, &sinAngle);
    ComputeVectorEnd(startX, startY, sinAngle, -cosAngle, thickness/2, &(rectX[0]), &(rectY[0]));
    ComputeVectorEnd(startX, startY, -sinAngle, cosAngle, thickness/2, &(rectX[3]), &(rectY[3]));
    ComputeVectorEnd(endX, endY, sinAngle, -cosAngle, thickness/2, &(rectX[1]), &(rectY[1]));
    ComputeVectorEnd(endX, endY, -sinAngle, cosAngle, thickness/2, &(rectX[2]), &(rectY[2]));
}

/**
 * @brief
 *
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @param x3
 * @param y3
 * @param x4
 * @param y4
 * @return bool
 */
bool DoSegmentsIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
{
    PathFinder::Segment seg1 = {PathFinder::CreatePoint(x1, y1), PathFinder::CreatePoint(x2, y2)};
    PathFinder::Segment seg2 = {PathFinder::CreatePoint(x3, y3), PathFinder::CreatePoint(x4, y4)};
    return PathFinder::DoSegmentsIntersect(seg1, seg2);
}

/**
 * @brief
 *
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @param x3
 * @param y3
 * @param x4
 * @param y4
 * @param isectX
 * @param isectY
 * @return bool
 */
bool ComputeLinesIntersection(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double *isectX, double *isectY)
{
    double d = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
    if (fabs(d) < 1e-6)
        return false;

    *isectX = ((x1*y2 - y1*x2) * (x3-x4) - (x1-x2) * (x3*y4 - y3*x4)) / d;
    *isectY = ((x1*y2 - y1*x2) * (y3-y4) - (y1-y2) * (x3*y4 - y3*x4)) / d;
    return true;
}

/**
 * @brief
 *
 * @param xMin
 * @param yMin
 * @param xMax
 * @param yMax
 * @param a
 * @param b
 * @param (isectX)[]
 * @param (isectY)[]
 * @return bool
 */
bool GetLineRectIntersections(double xMin, double yMin, double xMax, double yMax, double a, double b, double (&isectX)[2], double (&isectY)[2])
{
    if (a >= PathFinder::INFINI_TY || b >= PathFinder::INFINI_TY)
        return false;

    if (fabs(a) < 1e-6)
    {
        isectX[0] = xMin;
        isectX[1] = xMax;
        isectY[0] = b;
        isectY[1] = b;
        return true;
    }

    double x1 = xMin, x2 = xMax;
    double y1 = a*x1 + b, y2 = a*x2 + b;

    double y3 = yMin, y4 = yMax;
    double x3 = (y3 - b) / a, x4 = (y4 - b) / a;

    int k = 0;
    if (y1 >= yMin && y1 <= yMax)
    {
        isectX[k] = x1;
        isectY[k] = y1;
        k++;
    }

    if (y2 >= yMin && y2 <= yMax)
    {
        isectX[k] = x2;
        isectY[k] = y2;
        k++;
    }

    if (k == 2)
        return true;

    if (x3 >= xMin && x3 <= xMax)
    {
        isectX[k] = x3;
        isectY[k] = y3;
        k++;
    }

    if (k == 2)
        return true;

    if (x4 >= xMin && x4 <= xMax)
    {
        isectX[k] = x4;
        isectY[k] = y4;
        k++;
    }

    return k == 2;
}

/**
 * @brief
 *
 * @param ulX
 * @param ulY
 * @param lrX
 * @param lrY
 * @param startX
 * @param startY
 * @param endX
 * @param endY
 * @param (isectX)[]
 * @param (isectY)[]
 * @return int
 */
int GetSegmentRectIntersections(double ulX, double ulY, double lrX, double lrY, double startX, double startY, double endX, double endY, double (&isectX)[2], double (&isectY)[2])
{
    int k = 0;

    if (DoSegmentsIntersect(ulX, ulY, lrX, ulY, startX, startY, endX, endY))
    {
        if (ComputeLinesIntersection(ulX, ulY, lrX, ulY, startX, startY, endX, endY, &(isectX[k]), &(isectY[k])))
            k++;
    }

    if (DoSegmentsIntersect(lrX, ulY, lrX, lrY, startX, startY, endX, endY))
    {
        if (ComputeLinesIntersection(lrX, ulY, lrX, lrY, startX, startY, endX, endY, &(isectX[k]), &(isectY[k])))
            k++;
    }

    if (k == 2)
        return k;

    if (DoSegmentsIntersect(lrX, lrY, ulX, lrY, startX, startY, endX, endY))
    {
        if (ComputeLinesIntersection(lrX, lrY, ulX, lrY, startX, startY, endX, endY, &(isectX[k]), &(isectY[k])))
            k++;
    }

    if (k == 2)
        return k;

    if (DoSegmentsIntersect(ulX, lrY, ulX, ulY, startX, startY, endX, endY))
    {
        if (ComputeLinesIntersection(ulX, lrY, ulX, ulY, startX, startY, endX, endY, &(isectX[k]), &(isectY[k])))
            k++;
    }

    return k;
}
