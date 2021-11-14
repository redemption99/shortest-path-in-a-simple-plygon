#include <math.h>
#include <vector>
#include <algorithm>
#include "geometry.h"

double geometry::Dist(Point a, Point b) {
    double x_dist = (b.x - a.x) * (b.x - a.x);
    double y_dist = (b.y - a.y) * (b.y - a.y);
    return sqrt(x_dist + y_dist);
}

int geometry::Turn(Point a, Point b, Point c) {
    double cross = (b.x - a.x) * (c.y - b.y) - (c.x - b.x) * (b.y - a.y);
    if (cross > 0) {
        return 1;
    } else if (cross < 0) {
        return -1;
    } else {
        return 0;
    }
}

int geometry::SameSide(Point a, Point b, Point c, Point d) {
    return Turn(c, d, a) * Turn(c, d, b);
}

bool geometry::DoIntersect(Point a, Point b, Point c, Point d) {
    // Lines intersect with segments + the projections intersect
    return SameSide(a, b, c, d) <= 0 &&
           SameSide(c, d, a, b) <= 0 &&
           std::min(a.x, b.x) <= std::max(c.x, d.x) &&
           std::min(c.x, d.x) <= std::max(a.x, b.x) &&
           std::min(a.y, b.y) <= std::max(c.y, d.y) &&
           std::min(c.y, d.y) <= std::max(a.y, b.y);
}

bool geometry::IsSelfIntersectingPoly(const std::vector<Point>& poly) {
    int n = poly.size();
	for (int i = 0; i < n; i++) {
        for (int j = i+2; j < n; j++) {
            if (i == 0 && j == n-1) continue;
            int i_nxt = (i+1) % n;
            int j_nxt = (j+1) % n;
            if (DoIntersect(poly[i], poly[i_nxt], poly[j], poly[j_nxt])) {
                return true;
            }
        }
    }
    return false;
}

double geometry::PolyArea(const std::vector<Point>& poly) {
    int n = poly.size();
	double area = 0;
	for (int i = 0; i < n; i++) {
        int nxt = (i+1)%n;
        area += (poly[i].x + poly[ nxt ].x) * (poly[ nxt ].y - poly[i].y);
    }
    return fabs(area / 2);
}

double geometry::Angle(Point a, Point b, Point c) {
    double angle1 = atan2(b.y - a.y, b.x - a.x);
    double angle2 = atan2(c.y - a.y, c.x - a.x);
    double alpha = angle1 - angle2;
    if (alpha < 0) alpha += 2*pi;
    return alpha;
}

double geometry::TriangleArea(Point a, Point b, Point c) {
    return (abs(a.x*(b.y-c.y) + b.x*(c.y-a.y) + c.x*(a.y-b.y)) / 2.0);
}

bool geometry::InsideTriangle(Triangle t, Point p) {
    double a0 = TriangleArea(t.a, t.b, t.c);
    double a1 = TriangleArea(p, t.b, t.c);
    double a2 = TriangleArea(t.a, p, t.c);
    double a3 = TriangleArea(t.a, t.b, p);
    return (abs(a0 - (a1+a2+a3)) < 0.0001);
}


