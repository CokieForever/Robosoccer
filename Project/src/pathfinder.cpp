#include "pathfinder.h"
#include <stdlib.h>
#include <vector>
#include <time.h>
#include <algorithm>

using namespace std;

PathFinder::Point PathFinder::CreatePoint(double x, double y)
{
    Point pt = {x, y, vector<Point*>(), INFINI_TY, NULL};
    return pt;
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

const PathFinder::ConvexPolygon* PathFinder::AddPolygon(const ConvexPolygon& p)
{
    ConvexPolygon *polygon = new ConvexPolygon(p);

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

    m_polygons.push_back(polygon);

    for (PointsList::iterator it = polygon->points.begin() ; it != polygon->points.end() ; it++)
    {
        Point *point = *it;
        m_points.push_back(point);
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

        delete pt2;
    }

    delete poly;
    return true;
}

PathFinder::Path PathFinder::ComputePath(Point &start, Point &end)
{
    double bestScore = INFINI_TY;
    Point *bestPoint = NULL;
    vector<Point*> accessiblePoints;

    //Determine points which are accesible from start / end
    for (PolygonsList::iterator it = m_polygons.begin() ; it != m_polygons.end() ; it++)
    {
        ConvexPolygon *poly = *it;
        for (PointsList::iterator it2 = poly->points.begin() ; it2 != poly->points.end() ; it2++)
        {
            Point *pt = *it2;
            if (CheckPointsVisibility(&start, pt))
            {
                pt->score = distBetweenPoints(start, *pt);
                if (pt->score < bestScore)
                {
                    bestScore = pt->score;
                    bestPoint = pt;
                }
            }
            else
                pt->score = INFINI_TY;
            pt->prevPoint = NULL;

            if (CheckPointsVisibility(pt, &end))
                accessiblePoints.push_back(pt);
        }
    }

    if (bestScore == INFINI_TY)
        return NULL;

    //Dijkstra
    bool done = false;
    while (!done)
    {
        double currentScore = bestScore;
        Point *currentPoint = bestPoint;

        bestScore = INFINI_TY;
        currentPoint->score = INFINI_TY;

        for (vector<Point*>::iterator it = currentPoint->visMap.begin() ; it != currentPoint->visMap.end() ; it++)
        {
            Point *pt = *it;
            double d = distBetweenPoints(*currentPoint, *pt);
            if (d+currentScore < pt->score)
            {
                pt->score = d + currentScore;
                pt->prevPoint = currentPoint;
            }

            if (pt->score < bestScore)
            {
                bestScore = d + pt->score;
                bestPoint = pt;
            }
        }

        if (bestScore == INFINI_TY)
            return NULL;

        if (find(accessiblePoints.begin(), accessiblePoints.end(), bestPoint) != accessiblePoints.end())
            done = 1;
    }

    //Computing path
    PathFinder::Path path = new vector<Point>();
    path->push_back(CreatePoint(end.x, end.y));
    Point *currentPoint = bestPoint;
    while(currentPoint->prevPoint)
    {
        path->push_back(CreatePoint(currentPoint->x, currentPoint->y));
        currentPoint = currentPoint->prevPoint;
    }
    path->push_back(CreatePoint(start.x, start.y));

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
        int n = poly->points.size();
        Point *point = poly->points[0];
        for (int i=0 ; i < n ; i++)
        {
            Point *next = poly->points[(i+1) % n];
            Segment seg = {*point, *next};

            if ((point != p1 || next != p2) && (point != p2 || next != p1))
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
    Point ul = CreatePoint(min(seg.start.x, seg.end.x), min(seg.start.y, seg.end.y));
    Point lr = CreatePoint(max(seg.start.x, seg.end.x), max(seg.start.y, seg.end.y));
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

int PathFinder::crossProduct(Point a, Point b)
{
    return a.x * b.y - a.y * b.x;
}

bool PathFinder::isPointRightOfLine(Segment a, Point b)
{
    Point p1 = CreatePoint(0,0);
    Point p2 = CreatePoint(a.end.x - a.start.x, a.end.y - a.start.y);
    Segment aTmp = {p1, p2};
    Point bTmp = CreatePoint(b.x - a.start.x, b.y - a.start.y);
    return crossProduct(aTmp.end, bTmp) < 0;
}

bool PathFinder::doSegmentCrossLine(Segment a, Segment b)
{
    return (isPointRightOfLine(a, b.start) ^ isPointRightOfLine(a, b.end));
}

bool PathFinder::doSegmentsIntersect(Segment a, Segment b)
{
    Rectangle box1 = getBoundingBox(a);
    Rectangle box2 = getBoundingBox(b);
    return doRectanglesIntersect(box1, box2)
            && doSegmentCrossLine(a, b)
            && doSegmentCrossLine(b, a);
}

double PathFinder::distBetweenPoints(Point &a, Point &b)
{
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}
