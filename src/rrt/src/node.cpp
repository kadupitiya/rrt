//
// Created by kadupitiya on 9/6/18.
//


#include "node.h"

Node::Node() {

}

Node::Node(int id, int parentId, Point &point) : id(id), parentId(parentId), point(point) {

}

void Node::generateQuaternion(){

    tf::Quaternion quar;
    //roll, pitch, yaw : eularAngles.x, eularAngles.y, eularAngles.z
    quar = tf::createQuaternionFromRPY(this->eularAngles.x, this->eularAngles.y, this->eularAngles.z);
    this->orienttation.x = quar.x();
    this->orienttation.y = quar.y();
    this->orienttation.z = quar.z();
    this->orienttation.w = quar.w();

}


