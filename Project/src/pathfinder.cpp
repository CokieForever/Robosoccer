#include "pathfinder.h"
#include <stdlib.h>
#include <vector>
#include <time.h>
#include <algorithm>
#include <iostream>
#include <math.h>
#include "coordinates.h"
#include "matrix.h"
#include "geometry.h"
//#include "mutexdebug.h"

using namespace std;


/**
 * @brief Creates a @ref PathFinder::Point "point". This function should always be used to create points.
 * @param x The x coordinate of the point to create.
 * @param y The y coordinate of the point to create.
 * @return PathFinder::Point The created point. It does not need to be freed.
 */
PathFinder::Point PathFinder::CreatePoint(double x, double y)
{
    Point pt = {x, y, vector<Point*>(), INFINI_TY, NULL};
    return pt;
}

/**
 * @brief Creates a @ref PathFinder::Path "path" with the two given @ref PathFinder::Point "points".
 * @param start The first point of the path to create.
 * @param end The second (and last) point of the path to create.
 * @return PathFinder::Path The created path. It must be freed with delete.
 */
PathFinder::Path PathFinder::CreatePath(Point start, Point end)
{
    Path path = new vector<Point>;
    path->push_back(CreatePoint(start.x, start.y));
    path->push_back(CreatePoint(end.x, end.y));
    return path;
}

/**
 * @brief Converts a @ref PathFinder::Path "path" to a list of Positions.
 *
 * This function takes a path, typically given by the @ref PathFinder::ComputePath() function, and converts it
 * to a list of positions. Every position is unnormalized by the given @ref CoordinatesCalibrer "coordinates calibrer".
 *
 * @param path The path to convert.
 * @param calibrer The coordinates calibrer to use to unnormalize the positions.
 * @return vector<Position> The positions list. It must be freed with delete.
 */
vector<Position>* PathFinder::ConvertPathToReal(const Path path, const CoordinatesCalibrer *calibrer)
{
    vector<Position>* posList = new vector<Position>;
    for (vector<Point>::const_iterator it = path->begin() ; it != path->end() ; it++)
    {
        const Point *pt = &(*it);
        Position pos(pt->x, pt->y);
        if (calibrer)
            posList->push_back(calibrer->UnnormalizePosition(pos));
        else
            posList->push_back(pos);
    }

    return posList;
}

/**
 * @brief Frees every @ref PathFinder::ConvexPolygon "polygon" in a given list.
 * @param list The list of polygons to free.
 */
void PathFinder::DestroyPolygonsList(PolygonsList list)
{
    for (PolygonsList::iterator it = list.begin() ; it != list.end() ; it++)
    {
        ConvexPolygon *poly = *it;
        for (PointsList::iterator it2 = poly->points.begin() ; it2 != poly->points.end() ; it2++)
        {
            Point *pt = *it2;
            delete pt;
        }
        delete poly;
    }
}


/**
 * @brief Default (and unique) constructor for the path finder.
 */
PathFinder::PathFinder()
{
    pthread_mutex_init(&m_mutex, NULL);
}

/**
 * @brief Destructor of the path finder.
 */
PathFinder::~PathFinder()
{
    pthread_mutex_destroy(&m_mutex);
}


/**
 * @brief Tells if a given @ref PathFinder::ConvexPolygon "polygon" is currenlty in the path finder's world.
 *
 * Note that this does not depend on the polygon's coordinates, but only on the polygon's location in memory.
 * Another polygon with the same coordinates can give a different result.
 *
 * @param poly The polygon to check.
 * @return bool True if the polygon is in the world, False if not.
 */
bool PathFinder::IsPolygonRegistered(const ConvexPolygon* poly) const
{
    pthread_mutex_lock((pthread_mutex_t*)&m_mutex);
    bool result = find(m_polygons.begin(), m_polygons.end(), poly) != m_polygons.end();
    pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);
    return result;
}

/**
 * @brief Tells if a given @ref PathFinder::Point "point" is currently in the path finder's world.
 *
 * Note that this does not depend on the point's coordinates, but only on the point's location in memory.
 * Another point with the same coordinates can give a different result.
 *
 * @param pt The point to check.
 * @return bool True if the point is in the world, False if not.
 */
bool PathFinder::IsPointRegistered(const Point* pt) const
{
    pthread_mutex_lock((pthread_mutex_t*)&m_mutex);
    bool result = find(m_points.begin(), m_points.end(), pt) != m_points.end();
    pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);
    return result;
}

/**
 * @brief Returns a list of every @ref PathFinder::ConvexPolygon "polygon" currently in the path finder's world.
 * @return const PathFinder::PolygonsList & The list of polygons. It should not be modified, nor the polygons altered.
                                            If you want a mutable list, use @ref PathFinder::GetPolygonsCopy().
 */
const PathFinder::PolygonsList& PathFinder::GetPolygons() const
{
    return m_polygons;
}

/**
 * @brief Returns a list of every @ref PathFinder::ConvexPolygon "polygon" currently in the path finder's world.
 * @return PathFinder::PolygonsList The list of polygons. It is deeply copied and can therefore be modified at will.
 *                                  For an immutable but faster version, look at @ref PathFinder::GetPolygons().
 */
PathFinder::PolygonsList PathFinder::GetPolygonsCopy() const
{
    pthread_mutex_lock((pthread_mutex_t*)&m_mutex);

    PolygonsList list;
    for (PolygonsList::const_iterator it = m_polygons.begin() ; it != m_polygons.end() ; it++)
    {
        ConvexPolygon *poly = *it;
        ConvexPolygon *newPoly = new ConvexPolygon;
        for (PointsList::iterator it2 = poly->points.begin() ; it2 != poly->points.end() ; it2++)
        {
            Point *pt = *it2;
            newPoly->points.push_back(new Point(*pt));
        }
        list.push_back(newPoly);
    }

    pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);
    return list;
}


/**
 * @brief Adds a horizontal rectangle to the path finder's world.
 * @param ul0 The upper left corner of the rectangle to add.
 * @param lr0 The lower right corner of the rectangle to add.
 * @return const PathFinder::ConvexPolygon * The @ref PathFinder::ConvexPolygon "polygon" actually added to the path finder's world. It should not be modified.
 */
const PathFinder::ConvexPolygon* PathFinder::AddRectangle(const Point& ul0, const Point& lr0)
{
    ConvexPolygon polygon;
    Point *ur = new Point(CreatePoint(ul0.x, lr0.y));
    Point *ll = new Point(CreatePoint(lr0.x, ul0.y));
    Point *ul = new Point(CreatePoint(ul0.x, ul0.y));
    Point *lr = new Point(CreatePoint(lr0.x, lr0.y));

    polygon.points.push_back(ul);
    polygon.points.push_back(ur);
    polygon.points.push_back(lr);
    polygon.points.push_back(ll);

    return AddPolygon(polygon);
}

/**
 * @brief Adds a parallelogram to the path finder's world.
 * @param ul0 The "upper left" corner of the parallelogram to add.
 * @param ur0 The "upper right" corner of the parallelogram to add.
 * @param ll0 The "lower left" corner of the parallelogram to add.
 * @return const PathFinder::ConvexPolygon * The @ref PathFinder::ConvexPolygon "polygon" actually added to the path finder's world. It should not be modified.
 */
const PathFinder::ConvexPolygon* PathFinder::AddParallelogram(const Point& ul0, const Point& ur0, const Point& ll0)
{
    ConvexPolygon polygon;
    Point *ur = new Point(CreatePoint(ur0.x, ur0.y));
    Point *ll = new Point(CreatePoint(ll0.x, ll0.y));
    Point *ul = new Point(CreatePoint(ul0.x, ul0.y));
    Point *lr = new Point(CreatePoint(ll0.x+ur0.x-ul0.x, ll0.y+ur0.y-ul0.y));

    polygon.points.push_back(ul);
    polygon.points.push_back(ur);
    polygon.points.push_back(lr);
    polygon.points.push_back(ll);

    return AddPolygon(polygon);
}

/**
 * @brief Adds a "thick line" (rotated rectangle) to the path finder's world.
 * @param pt1 The first point of the line segment.
 * @param pt2 The second point of the line segment.
 * @param thickness The thickness of the line segment.
 * @return const PathFinder::ConvexPolygon * The @ref PathFinder::ConvexPolygon "polygon" actually added to the path finder's world. It should not be modified.
 */
const PathFinder::ConvexPolygon* PathFinder::AddThickLine(const Point& pt1, const Point& pt2, double thickness)
{
    double rectX[4], rectY[4];
    ComputeThickLineRect(pt1.x, pt1.y, pt2.x, pt2.y, thickness, rectX, rectY);

    ConvexPolygon polygon;
    Point *ur = new Point(CreatePoint(rectX[1], rectY[1]));
    Point *ll = new Point(CreatePoint(rectX[3], rectY[3]));
    Point *ul = new Point(CreatePoint(rectX[0], rectY[0]));
    Point *lr = new Point(CreatePoint(rectX[2], rectY[2]));

    polygon.points.push_back(ul);
    polygon.points.push_back(ur);
    polygon.points.push_back(lr);
    polygon.points.push_back(ll);

    return AddPolygon(polygon);
}

/**
 * @brief Adds a @ref PathFinder::ConvexPolygon "convex polygon" to the path finder's world.
 * @param p The polygon to add. The polygon itself is copied, but not the points it contains.
            Consequently, the points should have been allocated with new: Point *pt = new Point(PathFinder::CreatePoint(x,y))
 * @return const PathFinder::ConvexPolygon * The @ref PathFinder::ConvexPolygon "polygon" actually added to the path finder's world. It should not be modified.
 */
const PathFinder::ConvexPolygon* PathFinder::AddPolygon(const ConvexPolygon& p)
{
    pthread_mutex_lock((pthread_mutex_t*)&m_mutex);

    ConvexPolygon *polygon = new ConvexPolygon(p);

    m_polygons.push_back(polygon);

    for (PointsList::iterator it = polygon->points.begin() ; it != polygon->points.end() ; it++)
    {
        Point *point = *it;
        m_points.push_back(point);
    }

    for (PointsList::iterator it = m_points.begin() ; it != m_points.end() ; it++)
    {
        Point *pt1 = *it;
        for (PointsList::iterator it2 = it+1 ; it2 != m_points.end() ; it2++)
        {
            Point *pt2 = *it2;
            if (ReadPointsVisibility(pt1, pt2) && !CheckPointsVisibility_p(pt1, pt2))
            {
                pt1->visMap.erase(remove(pt1->visMap.begin(), pt1->visMap.end(), pt2), pt1->visMap.end());
                pt2->visMap.erase(remove(pt2->visMap.begin(), pt2->visMap.end(), pt1), pt2->visMap.end());
            }
        }
    }

    for (PointsList::iterator it = polygon->points.begin() ; it != polygon->points.end() ; it++)
    {
        Point *point = *it;
        ComputeVisibilityMap(point);
    }

    pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);
    return polygon;
}

/**
 * @brief Removes and fully frees a given @ref PathFinder::ConvexPolygon "polygon" from the path finder's world.
 * @param poly The polygon to remove.
 * @return bool True on succes or if poly is NULL, False on failure (e.g. not found).
 */
bool PathFinder::RemovePolygon(const ConvexPolygon* poly)
{
    if (!poly)
        return true;

    pthread_mutex_lock((pthread_mutex_t*)&m_mutex);

    PolygonsList::iterator it = find(m_polygons.begin(), m_polygons.end(), poly);
    if (it == m_polygons.end())
    {
        pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);
        return false;
    }

    for (PointsList::const_iterator it3 = poly->points.begin() ; it3 != poly->points.end() ; it3++)
    {
        Point *pt2 = *it3;
        for (PointsList::iterator it2 = m_points.begin() ; it2 != m_points.end() ; it2++)
        {
            Point *pt1 = *it2;
            pt1->visMap.erase(remove(pt1->visMap.begin(), pt1->visMap.end(), pt2), pt1->visMap.end());
        }

        m_points.erase(remove(m_points.begin(), m_points.end(), pt2), m_points.end());
        delete pt2;
    }

    m_polygons.erase(remove(m_polygons.begin(), m_polygons.end(), poly), m_polygons.end());
    delete poly;

    for (PolygonsList::iterator it2 = m_polygons.begin() ; it2 != m_polygons.end() ; it2++)
    {
        ConvexPolygon *poly1 = *it2;
        int n = poly1->points.size();
        Point *pt1 = poly1->points[0];
        for (int i=0 ; i < n ; i++)
        {
            Point *pt2 = poly1->points[(i+1)%n];
            if (!ReadPointsVisibility(pt1, pt2) && CheckPointsVisibility_p(pt1, pt2))
            {
                pt1->visMap.push_back(pt2);
                pt2->visMap.push_back(pt1);
            }

            for (PolygonsList::iterator it3 = it2+1 ; it3 != m_polygons.end() ; it3++)
            {
                ConvexPolygon *poly2 = *it3;
                for (PointsList::iterator it4 = poly2->points.begin() ; it4 != poly2->points.end() ; it4++)
                {
                    Point *pt3 = *it4;
                    if (!ReadPointsVisibility(pt1, pt3) && CheckPointsVisibility_p(pt1, pt3))
                    {
                        pt1->visMap.push_back(pt3);
                        pt3->visMap.push_back(pt1);
                    }
                }
            }

            pt1 = pt2;
        }
    }

    pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);
    return true;
}

/**
 * @brief Removes and fully frees a list of @ref PathFinder::ConvexPolygon "polygons" from the path finder's world.
 *
 * This function calls @ref PathFinder::RemovePolygon() internally.
 *
 * @param polys[] The list of polygons to remove.
 * @param n The number of polygons in the list.
 * @return bool True on success of every removal operation, False if one or more failed.
 */
bool PathFinder::RemovePolygons(const ConvexPolygon* polys[], int n)
{
    bool success = true;
    for (int i=0 ; i < n ; i++)
        success &= RemovePolygon(polys[i]);
    return success;
}

/**
 * @brief Computes the shortest path between two given @ref PathFinder::Point "points" in the path finder's world.
 * @param start The starting point.
 * @param end The final point.
 * @return PathFinder::Path The resulting path, or NULL if no path was found. If not NULL, the path contains the starting point.
 *                          Note that the end point can be different from the one specified in the case the starting point is inside an obstacle.
 */
PathFinder::Path PathFinder::ComputePath(Point start, Point end)
{
    pthread_mutex_lock((pthread_mutex_t*)&m_mutex);

    //If the start point is inside a polygon, immediately go to the closest "allowed" point
    const ConvexPolygon *polygon = IsPointInsideSomePolygon(start);
    if (polygon)
    {
        Point exitPoint;
        sqDistToPolygon(start, *polygon, &exitPoint);

        double d = sqrt(sqDistBetweenPoints(start, exitPoint));
        double cosAngle, sinAngle;
        ComputeLineAngle(start.x, start.y, exitPoint.x, exitPoint.y, &cosAngle, &sinAngle);
        ComputeVectorEnd(start.x, start.y, cosAngle, sinAngle, d+0.05, &(exitPoint.x), &(exitPoint.y));

        Path path = new vector<Point>;
        path->push_back(CreatePoint(start.x, start.y));
        path->push_back(CreatePoint(exitPoint.x, exitPoint.y));

        pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);
        return path;
    }

    //Check if point is directly accessible
    if (CheckPointsVisibility_p(&start, &end))
    {
        PathFinder::Path path = new vector<Point>();
        path->push_back(CreatePoint(start.x, start.y));
        path->push_back(CreatePoint(end.x, end.y));

        pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);
        return path;
    }

    //Determine points which are accessible from start / end
    PointsList accessiblePoints;
    start.visMap.clear();
    for (PointsList::iterator it = m_points.begin() ; it != m_points.end() ; it++)
    {
        Point *pt = *it;
        pt->prevPoint = NULL;
        pt->score = INFINI_TY;

        if (CheckPointsVisibility_p(&start, pt))
            start.visMap.push_back(pt);

        if (CheckPointsVisibility_p(pt, &end))
            accessiblePoints.push_back(pt);
    }

    //Dijkstra
    double currentScore = 0;
    Point *currentPoint = &start;
    start.prevPoint = NULL;
    end.score = INFINI_TY;
    PointsList allPoints = m_points;
    allPoints.push_back(&end);

    while (currentPoint != &end)
    {
        currentPoint->score = -1;

        PointsList visMap = currentPoint->visMap;
        if (find(accessiblePoints.begin(), accessiblePoints.end(), currentPoint) != accessiblePoints.end())
            visMap.push_back(&end);

        for (vector<Point*>::iterator it = visMap.begin() ; it != visMap.end() ; it++)
        {
            Point *pt = *it;
            if (pt->score >= 0)
            {
                double d = sqrt(sqDistBetweenPoints(*currentPoint, *pt));
                if (d+currentScore < pt->score)
                {
                    pt->score = d + currentScore;
                    pt->prevPoint = currentPoint;
                }
            }
        }

        Point *bestPoint = NULL;
        double bestScore = INFINI_TY;

        for (vector<Point*>::iterator it = allPoints.begin() ; it != allPoints.end() ; it++)
        {
            Point *pt = *it;
            if (pt->score >= 0 && pt->score < bestScore)
            {
                bestScore = pt->score;
                bestPoint = pt;
            }
        }

        if (bestScore == INFINI_TY)
        {
            pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);
            return NULL;
        }

        currentPoint = bestPoint;
        currentScore = currentPoint->score;
    }

    //Computing path
    PathFinder::Path path = new vector<Point>();
    do
    {
        path->push_back(CreatePoint(currentPoint->x, currentPoint->y));
        currentPoint = currentPoint->prevPoint;
    } while(currentPoint);

    reverse(path->begin(), path->end());

    pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);
    return path;
}

/**
 * @brief Thread-safe wrapper for @ref PathFinder::CheckPointsVisibility_p().
 */
bool PathFinder::CheckPointsVisibility(const Point *p1, const Point *p2)
{
    pthread_mutex_lock((pthread_mutex_t*)&m_mutex);
    bool result = CheckPointsVisibility_p(p1, p2);
    pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);
    return result;
}

/**
 * @brief Computes the closest @ref PathFinder::Point "point" to a given point, which is accessible from another point.
 * @param start The point to be accessible from.
 * @param end The point to get close to.
 * @return PathFinder::Point The closest accessible point.
 */
PathFinder::Point PathFinder::ComputeClosestAccessiblePoint(const Point &start, const Point &end)
{
    pthread_mutex_lock((pthread_mutex_t*)&m_mutex);

    //Check if point is directly accessible
    if (CheckPointsVisibility_p(&start, &end))
    {
        pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);
        return CreatePoint(end.x, end.y);
    }

    //Convert polygons to map
    const int height = 200, width = 200;
    Matrix matrix(height, width);
    for (PolygonsList::iterator it = m_polygons.begin() ; it != m_polygons.end() ; it++)
    {
        ConvexPolygon *polygon = *it;
        int n = polygon->points.size();
        Matrix::Point *points = new Matrix::Point[n];

        for (int i=0 ; i < n ; i++)
        {
            points[i].x = std::max(0., std::min((double)width-1, (width-1) * (polygon->points[i]->x + 1) / 2));
            points[i].y = std::max(0., std::min((double)height-1, (height-1) * (polygon->points[i]->y + 1) / 2));
        }

        matrix.DrawPolygon(points, n, 1);
        delete points;
    }

    pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);

    //Flood map from start
    int sx = std::max(0., std::min((double)width-1, (width-1) * (start.x + 1) / 2));
    int sy = std::max(0., std::min((double)height-1, (height-1) * (start.y + 1) / 2));
    if (matrix[sx][sy] == 1)
        return CreatePoint(start.x, start.y);
    matrix.FloodFill(Matrix::CreatePoint(sx,sy), 2);

    //Look for closest point to target
    int ex = std::max(0., std::min((double)width-1, (width-1) * (end.x + 1) / 2));
    int ey = std::max(0., std::min((double)height-1, (height-1) * (end.y + 1) / 2));
    int bestX=0, bestY=0;
    double dMin = INFINI_TY;
    for (int x=0 ; x < width ; x++)
    {
        for (int y=0 ; y < height ; y++)
        {
            if (matrix[x][y] == 2)
            {
                double d = (x-ex)*(x-ex) + (y-ey)*(y-ey);
                if (d < dMin)
                {
                    bestX = x;
                    bestY = y;
                    dMin = d;
                }
            }
        }
    }

    if (dMin == INFINI_TY)
        return CreatePoint(start.x, start.y);

    return CreatePoint(2 * (bestX / (double)(width-1) - 0.5), 2 * (bestY / (double)(height-1) - 0.5));
}

/**
 * @brief Compare the score of two @ref PathFinder::Point "points" for the Dijkstra's Algorithm.
 * @param pt1 The first point to compare.
 * @param pt2 The second point to compare.
 * @return bool True if pt1 has a bigger score than pt2, else False.
 */
bool PathFinder::ComparePoints(Point *pt1, Point *pt2)
{
    return pt1->score > pt2->score;
}


/**
 * @brief Tells if two @ref PathFinder::Point "points" are visible from each other in the path finder's world.
 *
 * The points have to be registered in the path finder's world already. If they are not, use @ref PathFinder::CheckPointsVisibility_p().
 *
 * @param p1 The first point to check.
 * @param p2 The second point to check.
 * @return bool True if the points can see each other, else False.
 */
bool PathFinder::ReadPointsVisibility(const Point* p1, const Point* p2)
{
    return find(p1->visMap.begin(), p1->visMap.end(), p2) != p1->visMap.end();
}

/**
 * @brief Tells if two @ref PathFinder::Point "points" are visible from each other in the path finder's world.
 *
 * The points do not have to be registered in the path finder's world. If both are, you can use @ref PathFinder::ReadPointsVisibility()
 * for a faster computation.
 *
 * @param p1 The first point to check.
 * @param p2 The second point to check.
 * @return bool True if the points can see each other, else False.
 */
bool PathFinder::CheckPointsVisibility_p(const Point *p1, const Point *p2)
{
    Segment segment = {*p1, *p2};
    for (PolygonsList::iterator it = m_polygons.begin() ; it != m_polygons.end() ; it++)
    {
        ConvexPolygon *poly = *it;

        if ((DoesPointBelongToPolygon(p1, poly) < 0 && isPointInsidePolygon(*p1, *poly))
                || (DoesPointBelongToPolygon(p2, poly) < 0 && isPointInsidePolygon(*p2, *poly)))
            return false;

        int n = poly->points.size();
        Point *point = poly->points[0];
        for (int i=0 ; i < n ; i++)
        {
            Point *next = poly->points[(i+1) % n];
            Segment seg = {*point, *next};

            if (point != p1 && point != p2 && next != p1 && next != p2)
            {
                if (DoSegmentsIntersect(segment, seg))
                    return false;
            }

            point = next;
        }
    }
    return true;
}

/**
 * @brief Computes and registers the list of all @ref PathFinder::Point "points" visible from a given point in the path finder's world.
 *
 * The point should be registered in the path finder's world as well.
 *
 * @param point The point to check.
 */
void PathFinder::ComputeVisibilityMap(Point *point)
{
    for (PolygonsList::iterator it = m_polygons.begin() ; it != m_polygons.end() ; it++)
    {
        ConvexPolygon *poly = *it;
        int i;
        if ((i = DoesPointBelongToPolygon(point, poly)) >= 0)
        {
            int n = poly->points.size();

            Point *pt = poly->points[(i+1) % n];
            if (CheckPointsVisibility_p(point, pt))
            {
                point->visMap.push_back(pt);
                pt->visMap.push_back(point);
            }

            pt = poly->points[(i-1+n) % n];
            if (CheckPointsVisibility_p(point, pt))
            {
                point->visMap.push_back(pt);
                pt->visMap.push_back(point);
            }
        }
        else
        {
            for (PointsList::iterator it2 = poly->points.begin() ; it2 != poly->points.end() ; it2++)
            {
                Point *pt = *it2;
                if (CheckPointsVisibility_p(point, pt))
                {
                    point->visMap.push_back(pt);
                    pt->visMap.push_back(point);
                }
            }
        }
    }
}

/**
 * @brief Tells if a given @ref PathFinder::Point "point" belongs to a given @ref PathFinder::ConvexPolygon "polygon".
 *
 * Note that this does not depend on the point's coordinates, but only on the point's location in memory.
 * Another point with the same coordinates can give a different result.
 *
 * @param point The point to check.
 * @param polygon The polygon to check.
 * @return int The index of the point in the polygon, or -1 if the point does not belong to the polygon.
 */
int PathFinder::DoesPointBelongToPolygon(const Point *point, const ConvexPolygon *polygon)
{
    PointsList::iterator it = find(((ConvexPolygon*)polygon)->points.begin(), ((ConvexPolygon*)polygon)->points.end(), point);
    return it == polygon->points.end() ? (-1) : (it - polygon->points.begin());
}

/**
 * @brief Tells if a given @ref PathFinder::Point "point" is inside any @ref PathFinder::ConvexPolygon "polygon" in the path finder's world.
 *
 * The point does not have to be registered in the path finder's world.
 *
 * @param point The point to check.
 * @return const PathFinder::ConvexPolygon * The first polygon in which the point lies, or NULL if the point is outside all registered polygons.
 */
const PathFinder::ConvexPolygon* PathFinder::IsPointInsideSomePolygon(const Point &point)
{
    for (PolygonsList::iterator it = m_polygons.begin() ; it != m_polygons.end() ; it++)
    {
        ConvexPolygon *polygon = *it;
        if (isPointInsidePolygon(point, *polygon))
            return polygon;
    }
    return NULL;
}

/**
 * @brief Gets the bounding box of a given @ref PathFinder::Segment "line segment".
 * @param seg The line segment.
 * @return PathFinder::Rectangle The bounding box.
 */
PathFinder::Rectangle PathFinder::getBoundingBox(Segment seg)
{
    Point ul = CreatePoint(std::min(seg.start.x, seg.end.x), std::min(seg.start.y, seg.end.y));
    Point lr = CreatePoint(std::max(seg.start.x, seg.end.x), std::max(seg.start.y, seg.end.y));
    Rectangle bb = {ul, lr};
    return bb;
}

/**
 * @brief Tells if two @ref PathFinder::Rectangle "rectangles" intersect each other.
 * @param a The first rectangle to check.
 * @param b The second rectangle to check.
 * @return bool True if the two rectangles intersect each other, else False.
 */
bool PathFinder::doRectanglesIntersect(PathFinder::Rectangle a, PathFinder::Rectangle b)
{
    return a.ul.x <= b.lr.x
        && a.lr.x >= b.ul.x
        && a.ul.y <= b.lr.y
        && a.lr.y >= b.ul.y;
}

/**
 * @brief Computes the "orientation" (internal measurement) of a @ref PathFinder::Point "point" against a @ref PathFinder::Segment "line segment".
 *
 * The "orientation" is defined as the sign of the cross product of the vectors AB and PB, where A and B are the line segment
 * start and end (respectively) points, and P is the stand alone point. -1 means a negative orientation, +1 means a positive orientation,
 * and 0 means colinearity.
 *
 * @param seg The line segment.
 * @param pt The stand alone point.
 * @return int The orientation. Can be -1, 0 or +1.
 */
int PathFinder::orientation(Segment seg, const Point& pt)
{
    double p = (seg.end.x - seg.start.x) * (pt.y - seg.end.y) - (seg.end.y - seg.start.y) * (pt.x - seg.end.x);
    if (fabs(p) <= 1e-6)
        return 0;
    else
        return p > 0 ? 1 : -1;
}

/**
 * @brief Tells if two @ref PathFinder::Segment "line segments" intersect each other.
 * @param seg1 The first line segment to check.
 * @param seg2 The second line segment to check.
 * @return bool True if the two line segments intersect each other, else False.
 */
bool PathFinder::DoSegmentsIntersect(Segment seg1, Segment seg2)
{
    if (!doRectanglesIntersect(getBoundingBox(seg1), getBoundingBox(seg2)))
        return false;

    int o1 = orientation(seg1, seg2.start);
    int o2 = orientation(seg1, seg2.end);
    int o3 = orientation(seg2, seg1.start);
    int o4 = orientation(seg2, seg1.end);

    return (o1 == 0 || o2 == 0 || o1 != o2) && (o3 == 0 || o4 == 0 || o3 != o4);
}

/**
 * @brief Computes the square of the distance between two @ref PathFinder::Point "points".
 * @param a The first point.
 * @param b The second point.
 * @return double The squared distance.
 */
double PathFinder::sqDistBetweenPoints(const Point &a, const Point &b)
{
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

/**
 * @brief Tells if a @ref PathFinder::Point "point" is inside a given @ref PathFinder::ConvexPolygon "polygon".
 * @param point The point to check.
 * @param polygon The polygon to check.
 * @return bool True if the point is inside the polygon, else False.
 */
bool PathFinder::isPointInsidePolygon(const Point& point, const ConvexPolygon& polygon)
{
    int nbIsects = 0;
    Segment segment = {point, CreatePoint(point.x, INFINI_TY)};

    int n = polygon.points.size();
    Point *pt1 = polygon.points[0];
    for (int i=0 ; i < n ; i++)
    {
        Point *pt2 = polygon.points[(i+1)%n];
        Segment seg = {*pt1, *pt2};
        if (DoSegmentsIntersect(segment, seg))
            nbIsects++;
        pt1 = pt2;
    }

    return nbIsects == 1;   //Should be nbIsects%2 == 1 but this does not deal good with special cases, and as we only have convex polygons...
}

/**
 * @brief Computes the square of the distance of a @ref PathFinder::Point "point" to a given @ref PathFinder::Segment "line segment".
 * @param a The point.
 * @param seg The line segment.
 * @param isect A pointer to a point to store the closest point on the line segment. Can be NULL.
 * @return double The squared distance.
 */
double PathFinder::sqDistToSegment(const Point &a, Segment seg, Point *isect)
{
    double p = (seg.end.x - seg.start.x) * (a.x - seg.start.x) + (seg.end.y - seg.start.y) * (a.y - seg.start.y);
    double l = sqDistBetweenPoints(seg.start, seg.end);

    if (p < 0)
    {
        if (isect)
            *isect = CreatePoint(seg.start.x, seg.start.y);
        return sqDistBetweenPoints(a, seg.start);
    }
    else if (p > l)
    {
        if (isect)
            *isect = CreatePoint(seg.end.x, seg.end.y);
        return sqDistBetweenPoints(a, seg.end);
    }
    else
    {
        if (isect)
        {
            double cosAngle, sinAngle;
            ComputeLineAngle(seg.start.x, seg.start.y, seg.end.x, seg.end.y, &cosAngle, &sinAngle);
            ComputeVectorEnd(seg.start.x, seg.start.y, cosAngle, sinAngle, p/sqrt(l), &(isect->x), &(isect->y));
        }
        return sqDistBetweenPoints(a, seg.start) - p*p/l;
    }
}

/**
 * @brief Computes the square of the distance of a @ref PathFinder::Point "point" to a given @ref PathFinder::ConvexPolygon "polygon".
 * @param a The point.
 * @param polygon The polygon.
 * @param isect A pointer to a point to store the closest point on the polygon. Can be NULL.
 * @return double The squared distance.
 */
double PathFinder::sqDistToPolygon(const Point &a, const ConvexPolygon &polygon, Point *isect)
{
    int n = polygon.points.size();
    Point *pt1 = polygon.points[0];
    double minD = INFINI_TY;

    for (int i=0 ; i < n ; i++)
    {
        Point *pt2 = polygon.points[(i+1)%n];
        Segment seg = {*pt1, *pt2};
        Point it;

        double d = sqDistToSegment(a, seg, isect ? &it : NULL);

        if (d < minD)
        {
            if (isect)
                *isect = CreatePoint(it.x, it.y);
            minD = d;
        }

        pt1 = pt2;
    }

    return minD;
}
