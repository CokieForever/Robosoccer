#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <vector>
#include <map>
#include "coordinates.h"

class PathFinder
{

public:

    struct ConvexPolygon;
    struct Point;

    typedef std::vector<ConvexPolygon*> PolygonsList;
    typedef std::vector<Point*> PointsList;
    typedef std::vector<Point>* Path;

    struct Point
    {
        double x, y;

        PointsList visMap;

        double score;
        Point *prevPoint;
    };

    struct ConvexPolygon
    {
        PointsList points;
    };

    static const double INFINI_TY = 10e6;   //INFINITY does not compile. Don't ask me why.
    static Point CreatePoint(double x, double y);
    static Path CreatePath(Point start, Point end);
    static std::vector<Position>* ConvertPathToReal(const Path path, CoordinatesCalibrer *calibrer = NULL);
    static void DestroyPolygonsList(PolygonsList list);

    PathFinder();
    ~PathFinder();

    const ConvexPolygon* AddRectangle(const Point &ul, const Point &lr);
    const ConvexPolygon* AddParallelogram(const Point& ul0, const Point& ur0, const Point& ll0);
    const ConvexPolygon* AddThickLine(const Point& pt1, const Point& pt2, double thickness);
    const ConvexPolygon* AddPolygon(const ConvexPolygon& p);
    bool RemovePolygon(const ConvexPolygon* poly);
    bool RemovePolygons(const ConvexPolygon* polys[], int n);
    bool IsPolygonRegistered(const ConvexPolygon* poly) const;
    bool IsPointRegistered(const Point* pt) const;
    const std::vector<ConvexPolygon*>& GetPolygons() const;
    PolygonsList GetPolygonsCopy() const;
    std::vector<Point>* ComputePath(Point start, Point end);
    bool CheckPointsVisibility(const Point *p1, const Point *p2);
    Point ComputeClosestAccessiblePoint(const Point &start, const Point &end);

private:
    struct Segment
    {
        const Point &start, &end;
    };

    struct Rectangle
    {
        Point ul, lr;
    };

    static bool ComparePoints(Point *pt1, Point *pt2);
    static Rectangle getBoundingBox(Segment seg);
    static bool doRectanglesIntersect(PathFinder::Rectangle a, PathFinder::Rectangle b);
    static int orientation(Segment seg, const Point& pt);
    static bool doSegmentsIntersect(Segment seg1, Segment seg2);
    static double sqDistBetweenPoints(const Point &a, const Point &b);
    static bool isPointInsidePolygon(const Point& point, const ConvexPolygon& polygon);
    static double sqDistToSegment(const Point &a, Segment seg, Point *isect);
    static double sqDistToPolygon(const Point &a, const ConvexPolygon &polygon, Point *isect);

    PolygonsList m_polygons;
    PointsList m_points;
    pthread_mutex_t m_mutex;

    bool ReadPointsVisibility(const Point* p1, const Point* p2);
    void ComputeVisibilityMap(Point *point);
    int DoesPointBelongToPolygon(const Point *point, const ConvexPolygon *polygon);
    const ConvexPolygon* IsPointInsideSomePolygon(const Point &point);
    bool CheckPointsVisibility_p(const Point *p1, const Point *p2);
};

#endif // PATHFINDER_H
