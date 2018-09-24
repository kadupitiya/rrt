//
// Created by kadupitiya on 9/7/18.
//

#include "geometry.h"

/*
 * Check if a point is inside a rectangle simple way
 * */
bool Geometry::checkInsideRectSimple(Point &p1, Point &p2, Point &p3, Point &p4, Point &p) {

    bool x_condition =false;
    bool y_condition =false;

    if(p1.x >= p3.x)
        x_condition = p1.x >= p.x && p.x >= p3.x;
    else
        x_condition = p1.x <= p.x && p.x <= p3.x;

    if(p1.y >= p3.y)
        y_condition = p1.y >= p.y && p.y >= p3.y;
    else
        y_condition = p1.y <= p.y && p.y <= p3.y;

    return x_condition && y_condition;

}

/**
 * Calculate the cross product of two points.
 */
double Geometry::crossProduct(Point &a, Point &b) {
    return a.x * b.y - b.x * a.y;
}

/**
 * Check if bounding boxes do intersect. If one bounding box
 * touches the other, they do intersect.
 * a1 and a2: 1st bounding box
 * b1 and b2: 2nd bounding box
 */
bool Geometry::doBoundingBoxesIntersect(LineSegment &a, LineSegment &b) {
    return a.first.x <= b.second.x && a.second.x >= b.first.x && a.first.y <= b.second.y && a.second.y >=b.first.y;
}

/**
 * Checks if a Point is on a line
 */
bool Geometry:: isPointOnLine(LineSegment &a, Point &b) {
    LineSegment aTmp;
    Point tmp1, tmp2;
    tmp1.x =0.0;
    tmp1.y =0.0;
    tmp2.x =a.second.x - a.first.x;
    tmp2.y =a.second.y - a.first.y;
    aTmp.first = tmp1;
    aTmp.second = tmp2;
    Point bTmp;
    bTmp.x = b.x - a.first.x;
    bTmp.y = b.y - a.first.y;
    double r = crossProduct(aTmp.second, bTmp);
    return abs(r) < EPSILON;
}

/**
 * Checks if a point is right of a line. If the point is on the
 * line, it is not right of the line.
 */
bool Geometry::isPointRightOfLine(LineSegment &a, Point &b) {
    LineSegment aTmp;
    Point tmp1, tmp2;
    tmp1.x =0.0;
    tmp1.y =0.0;
    tmp2.x =a.second.x - a.first.x;
    tmp2.y =a.second.y - a.first.y;
    aTmp.first = tmp1;
    aTmp.second = tmp2;
    Point bTmp;
    bTmp.x = b.x - a.first.x;
    bTmp.y = b.y - a.first.y;
    return crossProduct(aTmp.second, bTmp) < 0;
}

/**
 * Check if line segment first touches or crosses the line that is
 * defined by line segment second.
 */
bool Geometry::lineSegmentTouchesOrCrossesLine(LineSegment &a, LineSegment &b) {
    return isPointOnLine(a, b.first)
           || isPointOnLine(a, b.second)
           || (!isPointRightOfLine(a, b.first) != !isPointRightOfLine(a, b.second));
}

/**
 * Get the bounding box of this line by two points. The first point is in
 * the lower left corner, the second one at the upper right corner.
 */
Geometry::LineSegment Geometry::getBoundingBox(LineSegment &lineSeg) {
    Point p1, q1;
    p1.x = min(lineSeg.first.x, lineSeg.second.x);
    p1.y = min(lineSeg.first.y, lineSeg.second.y);
    q1.x = max(lineSeg.first.x, lineSeg.second.x);
    q1.y = max(lineSeg.first.y, lineSeg.second.y);

    LineSegment box_bounds;
    box_bounds.first = p1;
    box_bounds.second= q1;

    return box_bounds;
}



/**
 * Check if line segments intersect
 * a first line segment
 * b second line segment
 */
bool Geometry::doLinesIntersect(LineSegment &a, LineSegment &b) {
    LineSegment box1 = getBoundingBox(a);
    LineSegment box2 = getBoundingBox(b);
    return doBoundingBoxesIntersect(box1, box2)
           && lineSegmentTouchesOrCrossesLine(a, b)
           && lineSegmentTouchesOrCrossesLine(b, a);
}

/*
 * Returns a Line segment for a given two points
 */
Geometry::LineSegment Geometry::getLineSegment(Point &a, Point &b) {
    LineSegment aTmp;
    aTmp.first = a;
    aTmp.second = b;
    return aTmp;
}
