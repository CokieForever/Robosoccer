#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <vector>
#include <map>
#include "coordinates.h"

/**
 * @brief Class used to find the shortest path in a given environment.
 *
 * The path finder works in a theoritically infinite world which can be populated by obstacles represented as
 * convex polygons (see @ref PathFinder::AddPolygon()).
 * It can find the shortest path between two random points in this world by computing a visibility map for every point and
 * using the Dijkstra's algorithm.
 * The path finding itself is very quick if there are not too many polygons in the worlds, but the computation of the
 * visibility map is slower: consequently, it is better not to modify the polygons too often.
 * In practice, the world's dimensions should not exceed @ref PathFinder::INFINI_TY.
 */
class PathFinder
{

public:

    struct ConvexPolygon;
    struct Point;

    /**
     * @brief Type to store a list of @ref PathFinder::ConvexPolygon "convex polygons". Stores pointers to the polygons.
     */
    typedef std::vector<ConvexPolygon*> PolygonsList;

    /**
     * @brief Type to store a list of @ref PathFinder::Point "points". Stores pointers to the points.
     */
    typedef std::vector<Point*> PointsList;

    /**
     * @brief Type used to represent a full path. Pointer to a list of @ref PathFinder::Point "points".
     *
     * As this type is a pointer, it must be instantiated with new and freed with delete.
     */
    typedef std::vector<Point>* Path;

    /**
     * @brief Structure to represent a point in the path finder.
     *
     * A point should always be created with the static method PathFinder::CreatePoint().
     */
    struct Point
    {
        double x;           /**< x coordinate of the point. */
        double y;           /**< y coordinate of the point. */

        PointsList visMap;  /**< List of points which are visible from this point. */

        double score;       /**< The score of the point when executing the Dijkstra's Algorithm. */
        Point *prevPoint;   /**< The parent of the point when excuting the Dijkstra's Algorithm. */
    };

    /**
     * @brief Structure used to represent a convex polygon.
     *
     * Note: Nothing prevents the creation of a concave polygon. However, the name of this structure should
     * remind the user not to create such a polygon, as it is not supported by the path finder. If you
     * wish to use a concave polygon, create several convex polygons to describe it.
     */
    struct ConvexPolygon
    {
        PointsList points;  /**< The points of the polygon. */
    };

    /**
     * @brief Structure used to describe a line segment.
     */
    struct Segment
    {
        const Point &start;     /**< The first point of the line segment. */
        const Point &end;       /**< The last point of the line segment. */
    };

    static const double INFINI_TY = 10e6;   /**< Number used to represent the infinity in the path finder. */

    static Point CreatePoint(double x, double y);
    static Path CreatePath(Point start, Point end);
    static std::vector<Position>* ConvertPathToReal(const Path path, const CoordinatesCalibrer *calibrer = NULL);
    static void DestroyPolygonsList(PolygonsList list);
    static bool DoSegmentsIntersect(Segment seg1, Segment seg2);

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
    /**
     * @brief Structure used to represent a horizontal rectangle.
     */
    struct Rectangle
    {
        Point ul;   /**< The upper left corner of the rectangle. */
        Point lr;   /**< The lower right corner of the rectangle. */
    };

    static bool ComparePoints(Point *pt1, Point *pt2);
    static Rectangle getBoundingBox(Segment seg);
    static bool doRectanglesIntersect(PathFinder::Rectangle a, PathFinder::Rectangle b);
    static int orientation(Segment seg, const Point& pt);
    static double sqDistBetweenPoints(const Point &a, const Point &b);
    static bool isPointInsidePolygon(const Point& point, const ConvexPolygon& polygon);
    static double sqDistToSegment(const Point &a, Segment seg, Point *isect);
    static double sqDistToPolygon(const Point &a, const ConvexPolygon &polygon, Point *isect);

    PolygonsList m_polygons;    /**< List of the polygons currently in the path finder's world. */
    PointsList m_points;        /**< List of all the points currently in the path finder's world. */
    pthread_mutex_t m_mutex;    /**< Internal mutex used to prevent race conditions. */

    bool ReadPointsVisibility(const Point* p1, const Point* p2);
    void ComputeVisibilityMap(Point *point);
    int DoesPointBelongToPolygon(const Point *point, const ConvexPolygon *polygon);
    const ConvexPolygon* IsPointInsideSomePolygon(const Point &point);
    bool CheckPointsVisibility_p(const Point *p1, const Point *p2);
};

#endif // PATHFINDER_H
