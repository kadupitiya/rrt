//
// Created by kadupitiya on 9/6/18.
//

#include "rrt.h"

RRT::RRT() {}

/*
 * Initialize RRT
 **/
RRT::RRT(int xLeft, int xRight, int yBottom, int yUp, double delta, Node *start, Node *end) : xLeft(xLeft),
                                                                                              xRight(xRight),
                                                                                              yBottom(yBottom),
                                                                                              yUp(yUp), delta(delta),
                                                                                              start(start), end(end) {
    allConfigurations.reserve(500);
    allConfigurations.push_back(start);

}

/*
 * Add new node to the tree
 **/
void RRT::addNewNode(Node &parent, Node *child) {
    child->parentId = parent.id;
    parent.children.push_back(child);
    child->parent = &parent;
}

/*
 * Get a random double between two numbers
 **/
double RRT::getRanddomDouble(double xLeft, double xRight) {

    Point randomPoint;

    //Introduce a uniform random distribution
    //randomPoint.x = (this->xRight-this->xLeft) * ( rand() / (double)RAND_MAX ) + this->xLeft;
    //randomPoint.y = (this->yUp-this->yBottom) * ( rand() / (double)RAND_MAX ) + this->yBottom;

    random_device rand_dev;
    mt19937 generator(rand_dev());
    uniform_int_distribution<int> xRand(xLeft, xRight);

    return xRand(generator);

}


/*
 * Get a random configuration inside the boundary
 **/
Point RRT::getRanddomConfiguration() {

    Point randomPoint;

    randomPoint.x = getRanddomDouble(this->xLeft, this->xRight);
    randomPoint.y = getRanddomDouble(this->yBottom, this->yUp);
    randomPoint.z = 0.0;

    return randomPoint;

}

/*
 * Get a random configuration inside a given dimention
 **/
Point RRT::getRanddomConfiguration(double xLeft, double xRight, double yBottom, double yUp) {

    Point randomPoint;

    randomPoint.x = getRanddomDouble(xLeft, xRight);
    randomPoint.y = getRanddomDouble(yBottom, yUp);
    randomPoint.z = 0.0;

    return randomPoint;

}

/*
 * Shift a cordinate based on boundry conditions
 **/
double RRT::shiftCordinate(double lower, double upper, double cordinate, double scale) {

    double extraUpper = (cordinate + scale / 2) - upper;
    double extraLower = (cordinate - scale / 2) - lower;

    if (extraUpper > 0)
        cordinate = upper - scale / 2;

    if (extraLower < 0)
        cordinate = lower + scale / 2;

    return cordinate;

}


/*
 * Return the nearest node pointer to a given point
 **/
Node *RRT::getNearestNode(Point &p) {

    distances.clear();

    int size = allConfigurations.size();
    if (size == 1) {
        return (allConfigurations[0]);
    } else {

        for (int i = 0; i < size; i++)
            distances[getDistance(p, allConfigurations[i]->point)] = allConfigurations[i];

        return distances.begin()->second;
    }
}


/*
 * This function calculate euclidean distance between two given points
 **/
double RRT::getDistance(Point &p1, Point &p2) {
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}

/*
 * This function checks whether the given point is inside the defined boundary
 **/
bool RRT::boundryCheck(Point &p) {
    return (p.x > this->xLeft && p.x < this->xRight && p.y > this->yBottom && p.y < this->yUp);
}

bool RRT::checkFeasibility(Node &node) {

    //Add more checks here 1. Object 2. if point exists 3. Forward/Backward Bias
    return boundryCheck(node.point);
}

/**
     *
     * This returns a P which is in delta distance to the pRand from the pNear
     */
Node *RRT::rrtExpand(Node &pNear, Point &pRand, int id) {

    double grad = getGradient(pNear.point, pRand);

    //Create a new node
    Point tempPoint;
    tempPoint.x = delta * cos(grad) + pNear.point.x;
    tempPoint.y = delta * sin(grad) + pNear.point.y;
    tempPoint.z = 0.0;
    Node *temp = new Node(id, -1, tempPoint);

    // Check all the conditions for new nodes validity
    if (checkFeasibility(*temp)) {
        temp->validity = true;
    } else
        temp->validity = false;

    return temp;
}


/**
     *
     * This returns a the gradient of a given two points
     */
double RRT::getGradient(Point &p1, Point &p2) {

    double grad = 0.0;
    double deltaX = p2.x - p1.x;
    double deltaY = p2.y - p1.y;

    if (deltaX != 0) {

        grad = atan(deltaY / deltaX); //1st region is default

        if (deltaY == 0 && deltaX < 0)//deltaY zero case and deltaX < 0
            grad = M_PI;
        else if (deltaY > 0 && deltaX < 0)// 2nd region
            grad += M_PI;
        else if (deltaY < 0 && deltaX < 0)//3rd region
            grad += M_PI;
        else if (deltaY < 0 && deltaX > 0)// 4th region
            grad += 2 * M_PI;

    } else {
        //deltaX zero case
        if (deltaY > 0)
            grad = M_PI / 2;
        else if (delta < 0)
            grad = -M_PI / 2;
        else
            grad = 0.0;
    }

    return grad;
}

/**
     *
     * This decides to go towards the goal or the random point
     */

Node *RRT::getGoalBiasedExpand(Node &pNear, Point &pRand, Point &end, int id, double goalBias) {

    //Toss a coin to decide to go towards the goal or the random point
    double r = getRanddomDouble(0.0, 1.0);
    if (r < goalBias)
        return rrtExpand(pNear, pRand, id); //Extend towards random node
    else
        return rrtExpand(pNear, end, id); //Extend towards goal
}


/**
 * This retrive the final path
 */
void RRT::retriveFinalPath() {

    finalConfiguration.push_back(*end);
    int id = end->parentId;
    Node *temp = end->parent;
    while (id != start->parentId) {
        finalConfiguration.push_back(*temp);
        temp = temp->parent;
        id = temp->parentId;
    }
    finalConfiguration.push_back(*temp);
}


void RRT::generatePlannedPath(double smoothness) {


    for (int i = 0; i < smoothedFinalConfiguration.size() - 1; i++) {

        Point now = smoothedFinalConfiguration[i].point;
        Point next = smoothedFinalConfiguration[i + 1].point;

        double gradient = getGradient(now, next);

        bool isPartionOver = false;

        smoothedFinalConfiguration[i].eularAngles.z = gradient;
        smoothedFinalConfiguration[i].generateQuaternion();

        plannedPath.push_back(smoothedFinalConfiguration[i]);

        double xValueCheck = now.x;
        double smoothIncrement = smoothness;

        while (!isPartionOver) {

            Node temp;
            //Positions
            temp.point.x = smoothIncrement * cos(gradient) + now.x;
            temp.point.y = smoothIncrement * sin(gradient) + now.y;
            temp.point.z = 0.0;
            //Orienttation X Y Z
            temp.eularAngles.x = 0.0;
            temp.eularAngles.y = 0.0;
            temp.eularAngles.z = gradient;
            //Add acceleration and velocities here.

            //Assign Quaternion
            temp.generateQuaternion();
            smoothIncrement += smoothness;

            if (temp.point.x < next.x)
                plannedPath.push_back(temp);
            else
                isPartionOver = true;

        }

        smoothedFinalConfiguration[i + 1].eularAngles.z = gradient;
        smoothedFinalConfiguration[i + 1].generateQuaternion();
        plannedPath.push_back(smoothedFinalConfiguration[i + 1]);

    }


}


void RRT::generateGradientBasedSmoothedPlannedPath(double smoothIncrement, int smoothSpan, int repeatition) {

    if (repeatition < 1)
        repeatition = 1;

    if (smoothSpan < 1)
        smoothSpan = 1;


    //Gradient based Smoothing to the path

    for (int k = 0; k < repeatition; k++) {

        for (int i = 0; i < plannedPath.size() - 1 - smoothSpan; i++) {

            double averageGrad = 0.0;

            for (int j = 0; j < smoothSpan; j++)
                averageGrad += plannedPath[i + j].eularAngles.z;

            averageGrad = averageGrad / smoothSpan;

            //Orienttation X Y Z
            plannedPath[i].eularAngles.x = 0.0;
            plannedPath[i].eularAngles.y = 0.0;
            plannedPath[i].eularAngles.z = averageGrad;
            plannedPath[i].generateQuaternion();


        }

    }

    //Position Update
    for (int i = 0; i < plannedPath.size() - 2; i++) {

        plannedPath[i + 1].point.x = cos(plannedPath[i].eularAngles.z) * smoothIncrement + plannedPath[i].point.x;
        plannedPath[i + 1].point.y = sin(plannedPath[i].eularAngles.z) * smoothIncrement + plannedPath[i].point.y;
        plannedPath[i + 1].point.z = 0.0;

    }


}

void RRT::generatePointBasedSmoothedPlannedPath(int smoothSpan, int repeatition) {

    if (repeatition < 1)
        repeatition = 1;

    if (smoothSpan < 3)
        smoothSpan = 3;

    int midElement = (smoothSpan+1)/2;

    //Piont based Smoothing to the path
    for (int k = 0; k < repeatition; k++) {

        for (int i = 0; i < plannedPath.size() - smoothSpan; i++) {

            double averageX = 0.0, averageY = 0.0;

            for(int j=0; j < smoothSpan ; j++){

                averageX += plannedPath[i+j].point.x;
                averageY += plannedPath[i+j].point.y;

            }

            averageX = averageX / (double)smoothSpan;
            averageY = averageY / (double)smoothSpan;

            //Orienttation X Y Z
            plannedPath[i+midElement-1].point.x = averageX;
            plannedPath[i+midElement-1].point.y = averageY;
            plannedPath[i+midElement-1].point.z = 0.0;
        }

    }

    double gradient = 0.0;
    //Gradient Update
    for (int i = 0; i < plannedPath.size() - 2; i++) {

        Point now = plannedPath[i].point;
        Point next = plannedPath[i + 1].point;
        gradient = getGradient(now, next);
        plannedPath[i].eularAngles.z = gradient;
        plannedPath[i].eularAngles.y = 0.0;
        plannedPath[i].eularAngles.x = 0.0;
        plannedPath[i].generateQuaternion();

    }

    //Final element Gradient Update
    plannedPath[plannedPath.size()-1].eularAngles.z = gradient;
    plannedPath[plannedPath.size()-1].eularAngles.y = 0.0;
    plannedPath[plannedPath.size()-1].eularAngles.x = 0.0;
    plannedPath[plannedPath.size()-1].generateQuaternion();


}
