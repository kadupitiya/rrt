//
// Created by kadupitiya on 9/7/18.
//

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <iostream>
#include <algorithm>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

using namespace std;
using namespace geometry_msgs;

using namespace geometry_msgs;

class Geometry {

public:

    struct LineSegment {
        Point first;
        Point second;
    };

    static constexpr const double EPSILON = 0.000001;

    //Member functions

    bool checkInsideRectSimple(Point &, Point &, Point &, Point &, Point &);

    double crossProduct(Point &, Point &);

    bool doBoundingBoxesIntersect(LineSegment &, LineSegment &);

    bool isPointOnLine(LineSegment &, Point &);

    bool isPointRightOfLine(LineSegment &, Point &);

    bool lineSegmentTouchesOrCrossesLine(LineSegment &a, LineSegment &b);

    LineSegment getBoundingBox(LineSegment &lineSeg);

    bool doLinesIntersect(LineSegment &, LineSegment &);

    LineSegment getLineSegment(Point &, Point &);

};


#endif //GEOMETRY_H
