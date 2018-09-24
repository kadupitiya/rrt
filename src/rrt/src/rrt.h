//
// Created by kadupitiya on 9/6/18.
//

#ifndef PROJECT_RRT_H
#define PROJECT_RRT_H

#include "node.h"

class RRT {

public:
    // members

    //Global Boundary parameters
    double xLeft;
    double xRight;
    double yBottom;
    double yUp;
    double delta;

    Node* start;
    Node* end;
    vector<Node*> allConfigurations;
    vector<Node> finalConfiguration;
    vector<Node> smoothedFinalConfiguration;
    vector<Node> plannedPath;
    map<double, Node*> distances;
    Geometry geometry;

    // Constructors
    RRT();

    RRT(int, int, int, int, double, Node *, Node *);

    // member functions

    void addNewNode(Node &, Node *);

    Point getRanddomConfiguration();

    Point getRanddomConfiguration(double, double, double, double );

    double getRanddomDouble(double, double);

    double shiftCordinate(double, double, double, double);

    Node* getNearestNode(Point &);

    bool boundryCheck(Point &);

    double getDistance(Point &p1, Point &p2);

    bool checkFeasibility(Node &);

    Node* rrtExpand(Node &, Point &, int);

    Node* getGoalBiasedExpand(Node &, Point &,  Point &, int id, double );

    void retriveFinalPath();

    double getGradient(Point &, Point &);

    void generatePlannedPath(double );

    void generateGradientBasedSmoothedPlannedPath(double, int, int);

    void generatePointBasedSmoothedPlannedPath(int, int);


};

#endif //PROJECT_RRT_H
