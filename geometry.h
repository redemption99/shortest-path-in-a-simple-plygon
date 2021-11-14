#pragma once
#include <vector>

namespace geometry {
    const double pi = 3.14159265358979;

    struct Point {
        double x, y;
        int idx;
    };

    struct Triangle {
        union
        {
            struct {Point a,b,c;};
            Point points[3];
        };
        std::vector <Triangle*> neigh;
        int idx;
    };

    struct LineSegment {
        union
        {
            struct {Point a, b;};
            Point points[2];
        };

    };


    // Returns euclidean distance
    double Dist(Point a, Point b);

    // Returns the turn given by (a, b, c)
    // 1=left, -1=right, 0=collinear
    int Turn(Point a, Point b, Point c);

    // Checks if a and b are on the same side of (c, d)
    // 1=same, -1=diff, 0=a/b on cd
    int SameSide(Point a, Point b, Point c, Point d);

    // Checks if line segments ab and cd intersect
    bool DoIntersect(Point a, Point b, Point c, Point d);

    // Checks if a polygon is self intersecting
    bool IsSelfIntersectingPoly(const std::vector<Point>& poly);

    // Calculates the area of a polygon by summing intersections
    double PolyArea(const std::vector<Point>& poly);

    // Return angle between vectors AB i AC
    double Angle(Point a, Point b, Point c);

    // Return area of a triangle given by (a, b, c)
    double TriangleArea(Point a, Point b, Point c);

    // Checks if a triangle contains point
    bool InsideTriangle(Triangle t, Point p);
}
