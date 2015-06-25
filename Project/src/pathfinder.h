#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <vector>
#include <map>

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

    struct Segment
    {
        const Point &start, &end;
    };

    struct Rectangle
    {
        const Point &ul, &lr;
    };

    struct ConvexPolygon
    {
        PointsList points;
    };

    static const double INFINI_TY = 10e6;
    static Point CreatePoint(double x, double y);

    PathFinder();

    const ConvexPolygon* AddRectangle(const Point &ul, const Point &lr);
    const ConvexPolygon* AddPolygon(const ConvexPolygon& p);
    bool RemovePolygon(const ConvexPolygon* poly);
    bool IsPolygonRegistered(const ConvexPolygon* poly) const;
    const std::vector<ConvexPolygon*>& GetPolygons() const;
    std::vector<Point>* ComputePath(Point &start, Point &end);

private:
    static Rectangle getBoundingBox(Segment seg);
    static bool doRectanglesIntersect(PathFinder::Rectangle a, PathFinder::Rectangle b);
    static int crossProduct(Point a, Point b);
    static bool isPointRightOfLine(Segment a, Point b);
    static bool doSegmentCrossLine(Segment a, Segment b);
    static bool doSegmentsIntersect(Segment a, Segment b);
    static double distBetweenPoints(Point &a, Point &b);

    PolygonsList m_polygons;
    PointsList m_points;

    bool ReadPointsVisibility(const Point* p1, const Point* p2);
    bool CheckPointsVisibility(const Point *p1, const Point *p2);
    void ComputeVisibilityMap(Point *point);
    int DoesPointBelongToPolygon(const Point *point, const ConvexPolygon *polygon);

};

#endif // PATHFINDER_H
