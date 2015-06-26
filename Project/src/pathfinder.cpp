#include "pathfinder.h"
#include <stdlib.h>
#include <vector>
#include <time.h>
#include <algorithm>
#include <iostream>
#include <math.h>
#include "coordinates.h"
#include "sdlutilities.h"

using namespace std;

PathFinder::Point PathFinder::CreatePoint(double x, double y)
{
    Point pt = {x, y, vector<Point*>(), INFINI_TY, NULL};
    return pt;
}

vector<Position>* PathFinder::ConvertPathToReal(const Path path, CoordinatesCalibrer *calibrer)
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


PathFinder::PathFinder()
{
    srand(time(NULL));
}

bool PathFinder::IsPolygonRegistered(const ConvexPolygon* poly) const
{
    return find(m_polygons.begin(), m_polygons.end(), poly) != m_polygons.end();
}

const PathFinder::PolygonsList& PathFinder::GetPolygons() const
{
    return m_polygons;
}


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

const PathFinder::ConvexPolygon* PathFinder::AddPolygon(const ConvexPolygon& p)
{
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
            if (ReadPointsVisibility(pt1, pt2) && !CheckPointsVisibility(pt1, pt2))
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

    return polygon;
}

bool PathFinder::RemovePolygon(const ConvexPolygon* poly)
{
    PolygonsList::iterator it = find(m_polygons.begin(), m_polygons.end(), poly);
    if (it == m_polygons.end())
        return false;

    for (PointsList::iterator it3 = ((ConvexPolygon*)poly)->points.begin() ; it3 != ((ConvexPolygon*)poly)->points.end() ; it3++)
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
            if (!ReadPointsVisibility(pt1, pt2) && CheckPointsVisibility(pt1, pt2))
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
                    if (!ReadPointsVisibility(pt1, pt3) && CheckPointsVisibility(pt1, pt3))
                    {
                        pt1->visMap.push_back(pt3);
                        pt3->visMap.push_back(pt1);
                    }
                }
            }

            pt1 = pt2;
        }
    }

    return true;
}

PathFinder::Path PathFinder::ComputePath(Point start, Point end)
{
    bool restart = IsPointInsideSomePolygon(start);
    Point realStart = start;
    if (restart)
        start = ComputeNearestPolygonPoint(start);

    //Check if point is directly accessible
    if (CheckPointsVisibility(&start, &end))
    {
        PathFinder::Path path = new vector<Point>();
        path->push_back(CreatePoint(start.x, start.y));
        path->push_back(CreatePoint(end.x, end.y));
        return path;
    }

    //Determine points which are accesible from start / end
    PointsList accessiblePoints;
    start.visMap.clear();
    for (PointsList::iterator it = m_points.begin() ; it != m_points.end() ; it++)
    {
        Point *pt = *it;
        pt->prevPoint = NULL;
        pt->score = INFINI_TY;

        if (CheckPointsVisibility(&start, pt))
            start.visMap.push_back(pt);

        if (CheckPointsVisibility(pt, &end))
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
                double d = distBetweenPoints(*currentPoint, *pt);
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
            return NULL;

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

    if (restart)
        path->push_back(CreatePoint(realStart->x, realStart->y));

    reverse(path->begin(), path->end());
    return path;
}



bool PathFinder::ReadPointsVisibility(const Point* p1, const Point* p2)
{
    return find(p1->visMap.begin(), p1->visMap.end(), p2) != p1->visMap.end();
}

bool PathFinder::CheckPointsVisibility(const Point *p1, const Point *p2)
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
                if (doSegmentsIntersect(segment, seg))
                    return false;
            }

            point = next;
        }
    }
    return true;
}

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
            if (CheckPointsVisibility(point, pt))
            {
                point->visMap.push_back(pt);
                pt->visMap.push_back(point);
            }

            pt = poly->points[(i-1+n) % n];
            if (CheckPointsVisibility(point, pt))
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
                if (CheckPointsVisibility(point, pt))
                {
                    point->visMap.push_back(pt);
                    pt->visMap.push_back(point);
                }
            }
        }
    }
}

int PathFinder::DoesPointBelongToPolygon(const Point *point, const ConvexPolygon *polygon)
{
    PointsList::iterator it = find(((ConvexPolygon*)polygon)->points.begin(), ((ConvexPolygon*)polygon)->points.end(), point);
    return it == polygon->points.end() ? (-1) : (it - polygon->points.begin());
}

PathFinder::Rectangle PathFinder::getBoundingBox(Segment seg)
{
    Point ul = CreatePoint(std::min(seg.start.x, seg.end.x), std::min(seg.start.y, seg.end.y));
    Point lr = CreatePoint(std::max(seg.start.x, seg.end.x), std::max(seg.start.y, seg.end.y));
    Rectangle bb = {ul, lr};
    return bb;
}

bool PathFinder::doRectanglesIntersect(PathFinder::Rectangle a, PathFinder::Rectangle b)
{
    return a.ul.x <= b.lr.x
        && a.lr.x >= b.ul.x
        && a.ul.y <= b.lr.y
        && a.lr.y >= b.ul.y;
}

int PathFinder::orientation(Segment seg, const Point& pt)
{
    double p = (seg.end.x - seg.start.x) * (pt.y - seg.end.y) - (seg.end.y - seg.start.y) * (pt.x - seg.end.x);
    if (fabs(p) <= 1e-6)
        return 0;
    else
        return p > 0 ? 1 : -1;
}

bool PathFinder::doSegmentsIntersect(Segment seg1, Segment seg2)
{
    if (!doRectanglesIntersect(getBoundingBox(seg1), getBoundingBox(seg2)))
        return false;

    int o1 = orientation(seg1, seg2.start);
    int o2 = orientation(seg1, seg2.end);
    int o3 = orientation(seg2, seg1.start);
    int o4 = orientation(seg2, seg1.end);

    return (o1 == 0 || o2 == 0 || o1 != o2) && (o3 == 0 || o4 == 0 || o3 != o4);
}

double PathFinder::distBetweenPoints(Point &a, Point &b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

bool PathFinder::isPointInsidePolygon(const Point& point, const ConvexPolygon& polygon)
{
    int nbIsects = 0;
    Segment segment = {point, CreatePoint(point.x, 100)};

    int n = polygon.points.size();
    Point *pt1 = polygon.points[0];
    for (int i=0 ; i < n ; i++)
    {
        Point *pt2 = polygon.points[(i+1)%n];
        Segment seg = {*pt1, *pt2};
        if (doSegmentsIntersect(segment, seg))
            nbIsects++;
        pt1 = pt2;
    }

    return nbIsects % 2 == 1;
}
