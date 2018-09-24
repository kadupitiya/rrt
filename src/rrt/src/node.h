//
// Created by kadupitiya on 9/6/18.
//

#ifndef PROJECT_NODE_H
#define PROJECT_NODE_H

#include "geometry.h"
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

class Node {

public:

    // members
    int id;
    int parentId;
    vector<Node*> children;
    Node* parent;
    bool validity;

    Point point;
    Point linearVelocity;
    Point linearAcceleration;
    Point eularAngles;  //roll, pitch, yaw : eularAngles.x, eularAngles.y, eularAngles.z
    Point angularVelocity;
    Point angularAcceleration;
    Quaternion orienttation;

    // Constructors
    Node();

    Node(int, int, Point &);

    // member functions

    void generateQuaternion();

};

#endif //PROJECT_NODE_H
