#include "geometry.h"
#include "pathfinder.h"

/**
 * @brief This function computes the end of a vector given a start point an angle and length of the vector.
 *
 * @param startX Coordinate x of the start point (origin of the vector).
 * @param startY Coordinate y of the start point (origin of the vector).
 * @param angle Angle between the vector and the x axis (clockwise).
 * @param length Length of the vector.
 * @param endX Coordinate x of the end point (end of the vector).
 * @param endY Coordinate y of the end point (end of the vector).
 */
void ComputeVectorEnd(double startX, double startY, double angle, double length, double *endX, double *endY)
{
    ComputeVectorEnd(startX, startY, cos(angle), sin(angle), length, endX, endY);
}

/**
 * @brief This function computes the end of a vector given a start point a cosinus and sinus of an angle and length of the vector.
 *
 * @param startX Coordinate x of the start point (origin of the vector).
 * @param startY Coordinate y of the start point (origin of the vector).
 * @param cosAngle Cosinus of the angle between the vector and the x axis (clockwise).
 * @param sinAngle Sinus of the angle between the vector and the x axis (clockwise).
 * @param length Length of the vector.
 * @param endX Coordinate x of the end point (end of the vector).
 * @param endY Coordinate y of the end point (end of the vector).
 */
void ComputeVectorEnd(double startX, double startY, double cosAngle, double sinAngle, double length, double *endX, double *endY)
{
    *endX = startX + cosAngle * length;
    *endY = startY + sinAngle * length;
}

/**
 * @brief This function computes the angle between a vector and the x axis.
 *
 * @param startX x coordinate of the origin of the vector.
 * @param startY y coordinate of the origin of the vector.
 * @param endX x coordinate of the end of the vector.
 * @param endY y coordinate of the end of the vector.
 * @param angle Pointer to receive the angle between the vector and the x axis. NULL will lead to segfault.
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
 * @brief  This function computes the cosinus and sinus of the angle between a vector and the x axis.
 *
 * @param startX x coordinate of the origin of the vector.
 * @param startY y coordinate of the origin of the vector.
 * @param endX x coordinate of the end of the vector.
 * @param endY y coordinate of the end of the vector.
 * @param cosAngle Pointer to receive the cosinus of the angle between the vector and x axis. NULL will lead to segfault.
 * @param sinAngle Pointer to receive the sinus of the angle between the vector and x axis. NULL will lead to segfault.
 */
void ComputeLineAngle(double startX, double startY, double endX, double endY, double *cosAngle, double *sinAngle)
{
    double d = sqrt((endX-startX)*(endX-startX) + (endY-startY)*(endY-startY));
    *cosAngle = (endX - startX) / d;
    *sinAngle = (endY - startY) / d;
}

/**
 * @brief This function computes the coordinates of the 4 corners of a rectangle.
 *
 * @param startX Coordinate x of the first corner in in the rectangle.
 * @param startY Coordinate y of the first corner in in the rectangle.
 * @param endX Coordinate x of the last corner in in the rectangle.
 * @param endY Coordinate y of the first corner in in the rectangle.
 * @param thickness Thickness of the lines that lie the corners.
 * @param (rectX)[] x coordinates of the four corners in the rectangle.
 * @param (rectY)[] y coordinates of the four corners in the rectangle.
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
 * @brief This function tells us if there is an intersection between two segments defined by two points each.
 *
 * @param x1 x coordinate of the first point in the first segment.
 * @param y1 y coordinate of the first point in the first segment.
 * @param x2 x coordinate of the second point in the first segment.
 * @param y2 y coordinate of the second point in the first segment.
 * @param x3 x coordinate of the first point in the second segment.
 * @param y3 y coordinate of the first point in the second segment.
 * @param x4 x coordinate of the second point in the second segment.
 * @param y4 y coordinate of the second point in the second segment.
 * @return bool True if there is intersection. False if not.
 */
bool DoSegmentsIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
{
    PathFinder::Segment seg1 = {PathFinder::CreatePoint(x1, y1), PathFinder::CreatePoint(x2, y2)};
    PathFinder::Segment seg2 = {PathFinder::CreatePoint(x3, y3), PathFinder::CreatePoint(x4, y4)};
    return PathFinder::DoSegmentsIntersect(seg1, seg2);
}

/**
 * @brief This function tells us if there is an intersection between two segments defined by two
 * points each and give the coordinates of the intersection point if there is an intersection.
 *
 * @param x1 x coordinate of the first point in the first segment.
 * @param y1 y coordinate of the first point in the first segment.
 * @param x2 x coordinate of the second point in the first segment.
 * @param y2 y coordinate of the second point in the first segment.
 * @param x3 x coordinate of the first point in the second segment.
 * @param y3 y coordinate of the first point in the second segment.
 * @param x4 x coordinate of the second point in the second segment.
 * @param y4 y coordinate of the second point in the second segment.
 * @param isectX Pointer to receive the x coordinate of the intersection point, if any. NULL can lead to segfault.
 * @param isectY Pointer to receive the y coordinate of the intersection point, if any. NULL can lead to segfault.
 * @return bool True if there is an intersection, False if not (in this case the data stays unchanged).
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
 * @brief Computes the intersection of a line with a rectangle.
 *
 * @param xMin The x coordinate of the upper-left corner of the rectangle.
 * @param yMin The y coordinate of the upper-left corner of the rectangle.
 * @param xMax The x coordinate of the lower-right corner of the rectangle.
 * @param yMax The y coordinate of the lower-right corner of the rectangle.
 * @param a The slope of the line (y = a*x + b)
 * @param b The offset of the line (y = a*x + b)
 * @param (isectX)[] The x coordinates of the intersection points, if any.
 * @param (isectY)[] The y coordinates of the intersection points, if any.
 * @return bool True if there is an intersection, False if no intersection.
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
 * @brief Computes the intersection of a line segment with a rectangle.
 *
 * @param ulX The x coordinate of the upper-left corner of the rectangle.
 * @param ulY The x coordinate of the upper-left corner of the rectangle.
 * @param lrX The x coordinate of the lower-right corner of the rectangle.
 * @param lrY The y coordinate of the lower-right corner of the rectangle.
 * @param startX The x coordinate of the first point of the line segment.
 * @param startY The y coordinate of the first point of the line segment.
 * @param endX The x coordinate of the second point of the line segment.
 * @param endY The y coordinate of the second point of the line segment.
 * @param (isectX)[] The x coordinates of the intersection points, if any.
 * @param (isectY)[] The y coordinates of the intersection points, if any.
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
