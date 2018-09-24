//
// Created by kadupitiya on 9/6/18.
//

/*
 * Test class
 * */

#include "rrt.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <sstream>


void initRviz(ros::Publisher, RRT &, vector<visualization_msgs::Marker> &, int, int);

void drawLines(ros::Publisher, Node &, Node &, float);

void drawObstacles(ros::Publisher, vector<visualization_msgs::Marker> &, int, int, RRT &);

void generateRandomObstacles(vector<visualization_msgs::Marker> &, int, RRT &);

bool checkForObjectIntersection(Point &, Point &, vector<visualization_msgs::Marker> &, double, RRT &);

bool checkForComplexObjectIntersection(Point &, Point &, vector<visualization_msgs::Marker> &, double, RRT &);

bool validateNewPoint(Node *, Point &, Point &, vector<visualization_msgs::Marker> &, double, RRT &);

bool validateEndPoint(Point &, Point &, vector<visualization_msgs::Marker> &, double, RRT &, double);

void drawRetrivedConfiguration(ros::Publisher, Node &, Node &, float);

void getshortestPath(vector<visualization_msgs::Marker> &, double, RRT &);

void drawShortestConfiguration(ros::Publisher, Node &, Node &, float);

void drawSmoothedConfiguration(ros::Publisher, Node &, Node &, float);

void printToRviz(ros::Publisher , double, double, string );

void drawRobot(ros::Publisher , Node &);

int main(int argc, char **argv) {

    ros::init(argc, argv, "rrt");
    //create a ros handle (pointer) so that you can call and use it
    ros::NodeHandle n;
    //in <>, it specified the type of the message to be published
    //in (), first param: topic name; second param: size of queued messages, at least 1
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("some_chatter", 10);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    //each second, ros "spins" and draws 20 frames
    ros::Rate loop_rate(20);
    int frame_count = 0;
    //ObstacleVector
    static vector<visualization_msgs::Marker> obstacles;
    int numberOfObs = 10;
    double paddingForObjectAvoidance = 0.8;
    double endPointVisibility = 0.9;
    double goalBias = 0.5;
    double isGoalBias = true;
    int smoothingRepeatition = 100;
    int smoothingWindow = 5;

    //RRT members
    double xLeft = 0.0;
    double xRight = 20.0;
    double yBottom = 0.0;
    double yUp = 20.0;
    double delta = 0.5;
    double robotMovingFrequency = 0.05;
    double xMid = (xLeft + xRight)/2;
    double yMid = (yBottom + yUp)/2;
    Point startP, endP;
    startP.x = 0.5;
    startP.y = 1.0;
    endP.x = 19.7;
    endP.y = 18.8;
    Node *startN = new Node(0, -1, startP);
    Node *EndN = new Node(1, -101, endP);
    RRT rrt(xLeft, xRight, yBottom, yUp, delta, startN, EndN);
    bool isRRTfinished = false;
    bool isPathRetrived= false;


    while (ros::ok()) {

        /******************** From here, we are defining and drawing two obstacles in the workspace **************************/

        initRviz(marker_pub, rrt, obstacles, frame_count, numberOfObs);

        if (frame_count <= 2){
            //first create a string typed (std_msgs) message

            std_msgs::String msg;
            std::stringstream ss;
            ss << "Frame ID: " << frame_count << ", Initiating..";
            msg.data = ss.str();
            ROS_INFO("%s", msg.data.c_str()); //printing on screen

            //publisher publishes messages, the msg type must be consistent with definition advertise<>();
            chatter_pub.publish(msg);

            printToRviz(marker_pub, xMid, yMid,ss.str());

        }
        //Ignore RRT for first 2 iterations
        if (!isRRTfinished && frame_count > 2) {

            std_msgs::String msg;
            std::stringstream ss;
            ss << "Frame ID: " << frame_count << ", Searching for the Endpoint";
            msg.data = ss.str();
            ROS_INFO("%s", msg.data.c_str()); //printing on screen
            //publisher publishes messages, the msg type must be consistent with definition advertise<>();
            chatter_pub.publish(msg);
            printToRviz(marker_pub, xMid, yMid,ss.str());


            Point randomPoint = rrt.getRanddomConfiguration();
            Node *nearNode = rrt.getNearestNode(randomPoint);
            Node *newNode = NULL;

            //Check to swich between RRT and Goal-Baised RRT
            if(isGoalBias)
                newNode = rrt.getGoalBiasedExpand(*nearNode, randomPoint, EndN->point, frame_count,goalBias);
            else
                newNode = rrt.rrtExpand(*nearNode, randomPoint, frame_count);

            //Node should be valid
            if (validateNewPoint(newNode, nearNode->point, newNode->point, obstacles, paddingForObjectAvoidance, rrt)) {
                rrt.addNewNode(*nearNode, newNode);
                rrt.allConfigurations.push_back(newNode);
                //Drawing edges
                drawLines(marker_pub, *nearNode, *newNode, frame_count);

                //check whether endpoint is near and feasible or not
                if (validateEndPoint(newNode->point, EndN->point, obstacles, paddingForObjectAvoidance, rrt,
                                     endPointVisibility)) {
                    //Ros update
                    ros::spinOnce();
                    loop_rate.sleep();
                    frame_count++;
                    rrt.addNewNode(*newNode, EndN);
                    rrt.allConfigurations.push_back(EndN);
                    //Drawing Last Edge
                    drawLines(marker_pub, *newNode, *EndN, frame_count);
                    isRRTfinished = true;
                }

            }


            //If path is not found for 3000 frames, abort the program
            if(frame_count>3000){
                std_msgs::String msg1;
                std::stringstream ss1;
                ss1 << "Frame ID: " << frame_count << ", Path could not be reached within the given time limit...";
                msg1.data = ss1.str();
                ROS_INFO("%s", msg1.data.c_str()); //printing on screen
                //publisher publishes messages, the msg type must be consistent with definition advertise<>();
                chatter_pub.publish(msg1);
                printToRviz(marker_pub, xMid, yMid,ss1.str());
                //Final render
                ros::spinOnce();
                loop_rate.sleep();
                ++frame_count;

                return 0.0;


            }

        } else if (isRRTfinished && !isPathRetrived) {
            /******************** From here, we are defining and drawing a simple robot **************************/
            //Now adjust the path, draw, create a passage, move the robot

            std_msgs::String msg;
            std::stringstream ss;
            ss << "Frame ID: " << frame_count << ", Endpoint found";
            msg.data = ss.str();
            ROS_INFO("%s", msg.data.c_str()); //printing on screen
            //publisher publishes messages, the msg type must be consistent with definition advertise<>();
            chatter_pub.publish(msg);
            printToRviz(marker_pub, xMid, yMid,ss.str());

            //Retrieving and ploting the final path
            rrt.retriveFinalPath();
            for(int i = rrt.finalConfiguration.size()-1 ; i > 0; i--){
                drawRetrivedConfiguration(marker_pub, rrt.finalConfiguration[i], rrt.finalConfiguration[i-1], frame_count);
                //inside ros update
                ros::spinOnce();
                loop_rate.sleep();
                ++frame_count;

                std_msgs::String msg;
                std::stringstream ss;
                ss << "Frame ID: " << frame_count << ", Retrieving final path";
                msg.data = ss.str();
                ROS_INFO("%s", msg.data.c_str()); //printing on screen
                //publisher publishes messages, the msg type must be consistent with definition advertise<>();
                chatter_pub.publish(msg);
                printToRviz(marker_pub, xMid, yMid,ss.str());
            }


            //Shortest path final path
            getshortestPath(obstacles, paddingForObjectAvoidance, rrt);

            for(int i = rrt.smoothedFinalConfiguration.size()-1 ; i > 0; i--){
                drawShortestConfiguration(marker_pub, rrt.smoothedFinalConfiguration[i], rrt.smoothedFinalConfiguration[i-1], frame_count);
                //inside ros update
                ros::spinOnce();
                loop_rate.sleep();
                ++frame_count;

                std_msgs::String msg;
                std::stringstream ss;
                ss << "Frame ID: " << frame_count << ", Retrieving shortest path";
                msg.data = ss.str();
                ROS_INFO("%s", msg.data.c_str()); //printing on screen
                //publisher publishes messages, the msg type must be consistent with definition advertise<>();
                chatter_pub.publish(msg);
                printToRviz(marker_pub, xMid, yMid,ss.str());
            }

            isPathRetrived= true;



        }else if(isPathRetrived){

            rrt.generatePlannedPath(robotMovingFrequency);
            //rrt.generateSmoothedPlannedPath(int smoothSpan, int repeatition);
            //rrt.generateGradientBasedSmoothedPlannedPath(robotMovingFrequency, smoothingWindow, smoothingRepeatition);

            rrt.generatePointBasedSmoothedPlannedPath(smoothingWindow, smoothingRepeatition);//smoothingRepeatition

            //Plotting smoothed path
            for(int i = 0 ; i < rrt.plannedPath.size()-1; i++){

                drawSmoothedConfiguration(marker_pub, rrt.plannedPath[i], rrt.plannedPath[i+1], frame_count);

                //inside ros update
                if(frame_count %5) {

                    ros::spinOnce();
                    loop_rate.sleep();
                    ++frame_count;

                    std_msgs::String msg;
                    std::stringstream ss;
                    ss << "Frame ID: " << frame_count << ", Smoothing final path";
                    msg.data = ss.str();
                    ROS_INFO("%s", msg.data.c_str()); //printing on screen
                    //publisher publishes messages, the msg type must be consistent with definition advertise<>();
                    chatter_pub.publish(msg);
                    printToRviz(marker_pub, xMid, yMid, ss.str());

                }
            }


            //Moving the Robot
            for(int i =0; i < rrt.plannedPath.size(); i++){

                drawRobot(marker_pub, rrt.plannedPath[i]);
                //inside ros update
                ros::spinOnce();
                loop_rate.sleep();
                ++frame_count;

                std_msgs::String msg;
                std::stringstream ss;
                ss << "Frame ID: " << frame_count << ", Moving the Robot";
                msg.data = ss.str();
                ROS_INFO("%s", msg.data.c_str()); //printing on screen
                //publisher publishes messages, the msg type must be consistent with definition advertise<>();
                chatter_pub.publish(msg);
                printToRviz(marker_pub, xMid, yMid,ss.str());


            }


            std_msgs::String msg1;
            std::stringstream ss1;
            ss1 << "Frame ID: " << frame_count << ", Finished..";
            msg1.data = ss1.str();
            ROS_INFO("%s", msg1.data.c_str()); //printing on screen
            //publisher publishes messages, the msg type must be consistent with definition advertise<>();
            chatter_pub.publish(msg1);
            printToRviz(marker_pub, xMid, yMid,ss1.str());
            //Final render
            ros::spinOnce();
            loop_rate.sleep();
            ++frame_count;

            return 0.0;


        }


        /******************** To here, we finished displaying our components **************************/


        // check if there is a subscriber. Here our subscriber will be Rviz
        while (marker_pub.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return 0;
            }
            ROS_WARN_ONCE("Please run Rviz in another terminal.");
            sleep(1);
        }

        //ros spins, force ROS frame to refresh/update once
        ros::spinOnce();
        loop_rate.sleep();
        ++frame_count;
    }


    return 0;


}

/*
 * Initiate the Rviz
 * */
void initRviz(ros::Publisher marker_pub, RRT &rrt, vector<visualization_msgs::Marker> &obstacles, int frame_count,
              int numberOfObs) {

    // Define two obstacles of point type
    visualization_msgs::Marker startM, endM, startText, endText;
    startM.type = endM.type = visualization_msgs::Marker::POINTS;
    startText.type = endText.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    startM.header.frame_id = endM.header.frame_id = startText.header.frame_id = endText.header.frame_id = "map";
    startM.header.stamp = endM.header.stamp = startText.header.stamp = endText.header.stamp = ros::Time::now();


    startM.lifetime =  endM.lifetime = startText.lifetime = endText.lifetime =ros::Duration();

    // Set the namespace and id
    startM.ns = "Start Point";
    endM.ns = "End Point";
    startM.id = rrt.start->id;
    endM.id = rrt.end->id;
    startText.ns = "Start Point Text";
    endText.ns = "End Point Text";
    startText.id = rrt.start->id;
    endText.id = rrt.end->id;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    startM.action = endM.action = startText.action = endText.action = visualization_msgs::Marker::ADD;

    startText.text = "Start";
    endText.text = "Goal";

    // Set the color red, green, blue. if not set, by default the value is 0
    //be sure to set alpha to something non-zero, otherwise it is transparent
    startM.color.a = 1.0f;
    startM.color.b = 1.0f;
    startText.color.a = 1.0f;
    startText.color.b = 1.0f;
    endM.color.a = 1.0f;
    endM.color.r = 1.0f;
    endText.color.a = 1.0f;
    endText.color.r = 1.0f;


    // Set the scale of the markers
    startM.scale.x = startM.scale.y = 0.25;
    endM.scale.x = endM.scale.y = 0.25;
    startText.scale.x = startText.scale.y = startText.scale.z =0.6;
    endText.scale.x = endText.scale.y = endText.scale.z =0.6;

    // Set the pose of the marker. since a side of the obstacle obst1 is 1m as defined above, now we place the obst1 center at (1, 2, 0.5). z-axis is height
    /*
    startM.pose.position.x = rrt.start->point.x;
    startM.pose.position.y = rrt.start->point.y;
    startM.pose.position.z = 0.0;
    startM.pose.orientation.x = 0.0;
    startM.pose.orientation.y = 0.0;
    startM.pose.orientation.z = 0.0;
    startM.pose.orientation.w = 1.0;	//(x, y, z, w) is a quaternion, ignore it here
    endM.pose.position.x = rrt.end->point.x;;
    endM.pose.position.y = rrt.end->point.x;;
    endM.pose.position.z = 0.0;
    endM.pose.orientation = startM.pose.orientation;
    */

    //Text positions
    startText.pose.position.x = rrt.start->point.x;
    startText.pose.position.y = rrt.start->point.y;
    startText.pose.position.z = 1.5;
    startText.pose.orientation.x = 0.0;
    startText.pose.orientation.y = 0.0;
    startText.pose.orientation.z = 0.0;
    startText.pose.orientation.w = 1.0;	//(x, y, z, w) is a quaternion, ignore it here
    endText.pose.position.x = rrt.end->point.x;
    endText.pose.position.y = rrt.end->point.y;
    endText.pose.position.z = 1.5;
    endText.pose.orientation.x = 0.0;
    endText.pose.orientation.y = 0.0;
    endText.pose.orientation.z = 0.0;
    endText.pose.orientation.w = 1.0;	//(x, y, z, w) is a quaternion, ignore it here

    startM.points.push_back(rrt.start->point);
    endM.points.push_back(rrt.end->point);

    //publish edge and vertices
    marker_pub.publish(startM);
    marker_pub.publish(endM);
    marker_pub.publish(startText);
    marker_pub.publish(endText);

    //
    /*
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = iteration;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0.0 + iteration;
    marker.pose.position.y = 1.0;
    marker.pose.position.z = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.text = "blablabla";

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.1;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    */

    drawObstacles(marker_pub, obstacles, frame_count, numberOfObs, rrt);

}


/*
 * Print infomation message in the Rviz
 * */
void printToRviz(ros::Publisher marker_pub, double xMid, double yMid, string messageObj) {

    // Define two obstacles of point type
    static visualization_msgs::Marker inforMSG;
    inforMSG.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    inforMSG.header.frame_id = "map";
    inforMSG.header.stamp = ros::Time::now();


    inforMSG.lifetime =ros::Duration();

    // Set the namespace and id
    inforMSG.ns = "Information MSG";
    inforMSG.id = 1000345;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    inforMSG.action = visualization_msgs::Marker::ADD;

    inforMSG.text = messageObj;

    // Set the color red, green, blue. if not set, by default the value is 0
    //be sure to set alpha to something non-zero, otherwise it is transparent
    inforMSG.color.a = 1.0f;
    inforMSG.color.r = inforMSG.color.g = 10.0f;
    inforMSG.color.b = 0.2f;


    // Set the scale of the markers
    inforMSG.scale.x = inforMSG.scale.y = inforMSG.scale.z =0.7;

    //Text positions
    inforMSG.pose.position.x = xMid;
    inforMSG.pose.position.y = yMid;
    inforMSG.pose.position.z = 3.5;
    inforMSG.pose.orientation.x = 0.0;
    inforMSG.pose.orientation.y = 0.0;
    inforMSG.pose.orientation.z = 0.0;
    inforMSG.pose.orientation.w = 1.0;	//(x, y, z, w) is a quaternion, ignore it here

    //publish edge and vertices
    marker_pub.publish(inforMSG);

}

/*
 * This draws obstacles in rviz
 * */
void drawObstacles(ros::Publisher marker_pub, vector<visualization_msgs::Marker> &obstacles, int frame_count,
                   int numberOfObs, RRT &rrt) {

    if (frame_count == 0)
        generateRandomObstacles(obstacles, numberOfObs, rrt);

    for (int i = 0; i < obstacles.size(); i++)
        marker_pub.publish(obstacles[i]);

}

/*
 * This function generate random obstacles
 * */
void generateRandomObstacles(vector<visualization_msgs::Marker> &obstacles, int numberOfObs, RRT &rrt) {

    double height = 0.6;

    //Space for obstacles: avoid start and end points
    double xleft_ = rrt.xLeft + 4.0;
    double xRight_ = rrt.xRight;
    double yBottom_ = rrt.yBottom;
    double yUp_ = rrt.yUp - 4.0;



    for (int i = 0; i < numberOfObs-3; i++) {

        visualization_msgs::Marker obstacle;
        obstacle.type = visualization_msgs::Marker::CUBE;
        obstacle.header.frame_id = "map";
        obstacle.header.stamp = ros::Time::now();
        obstacle.ns = "obstacle" + to_string(i + 1);
        obstacle.lifetime = ros::Duration();
        obstacle.action = visualization_msgs::Marker::ADD;
        obstacle.id = i;
        obstacle.scale.x = rrt.getRanddomDouble(1.0, 6.0);
        obstacle.scale.y = rrt.getRanddomDouble(1.0, 6.0);
        obstacle.scale.z = height;

        //randomPoint.x = (this->xRight-this->xLeft) * ( rand() / (double)RAND_MAX ) + this->xLeft;
        //randomPoint.y = (this->yUp-this->yBottom) * ( rand() / (double)RAND_MAX ) + this->yBottom;

        obstacle.pose.position.x = rrt.shiftCordinate(xleft_, xRight_, rrt.getRanddomDouble(xleft_, xRight_),
                                                      obstacle.scale.x);
        obstacle.pose.position.y = rrt.shiftCordinate(yBottom_, yUp_, rrt.getRanddomDouble(yBottom_, yUp_),
                                                      obstacle.scale.y);

        obstacle.pose.position.z = 0.2;
        obstacle.pose.orientation.x = 0.0;
        obstacle.pose.orientation.y = 0.0;
        obstacle.pose.orientation.z = 0.0;
        obstacle.pose.orientation.w = 1;
        obstacle.color.a = 1.0;
        obstacle.color.b = 0.75f;
        obstacle.color.r = obstacle.color.g = 0.70f;
        obstacles.push_back(obstacle);

    }

    //generate one or two obstacles near start area
    xleft_ = rrt.xLeft;
    xRight_ = rrt.xLeft + 1.5;
    yBottom_ = rrt.yBottom + 6.0;
    yUp_ = rrt.yUp;


    for (int i=numberOfObs-3; i < numberOfObs-2; i++) {

        visualization_msgs::Marker obstacle;
        obstacle.type = visualization_msgs::Marker::CUBE;
        obstacle.header.frame_id = "map";
        obstacle.header.stamp = ros::Time::now();
        obstacle.ns = "obstacle" + to_string(i + 1);
        obstacle.lifetime = ros::Duration();
        obstacle.action = visualization_msgs::Marker::ADD;
        obstacle.id = i;
        obstacle.scale.x = rrt.getRanddomDouble(1.0, 1.0);
        obstacle.scale.y = rrt.getRanddomDouble(1.0, 5.0);
        obstacle.scale.z = height;

        //randomPoint.x = (this->xRight-this->xLeft) * ( rand() / (double)RAND_MAX ) + this->xLeft;
        //randomPoint.y = (this->yUp-this->yBottom) * ( rand() / (double)RAND_MAX ) + this->yBottom;

        obstacle.pose.position.x = rrt.shiftCordinate(xleft_, xRight_, rrt.getRanddomDouble(xleft_, xRight_),
                                                      obstacle.scale.x);
        obstacle.pose.position.y = rrt.shiftCordinate(yBottom_, yUp_, rrt.getRanddomDouble(yBottom_, yUp_),
                                                      obstacle.scale.y);

        obstacle.pose.position.z = 0.2;
        obstacle.pose.orientation.x = 0.0;
        obstacle.pose.orientation.y = 0.0;
        obstacle.pose.orientation.z = 0.0;
        obstacle.pose.orientation.w = 1;
        obstacle.color.a = 1.0;
        obstacle.color.b = 0.75f;
        obstacle.color.r = obstacle.color.g = 0.70f;
        obstacles.push_back(obstacle);

    }



    //generate one or two obstacles near the goal area
    xleft_ = rrt.xLeft;
    xRight_ = rrt.xRight - 6.0;
    yBottom_ = rrt.yUp - 1.5;
    yUp_ = rrt.yUp;

    for (int i = numberOfObs-2; i< numberOfObs; i++) {

        visualization_msgs::Marker obstacle;
        obstacle.type = visualization_msgs::Marker::CUBE;
        obstacle.header.frame_id = "map";
        obstacle.header.stamp = ros::Time::now();
        obstacle.ns = "obstacle" + to_string(i + 1);
        obstacle.lifetime = ros::Duration();
        obstacle.action = visualization_msgs::Marker::ADD;
        obstacle.id = i;
        obstacle.scale.x = rrt.getRanddomDouble(1.0, 5.0);
        obstacle.scale.y = rrt.getRanddomDouble(1.0, 1.0);
        obstacle.scale.z = height;

        //randomPoint.x = (this->xRight-this->xLeft) * ( rand() / (double)RAND_MAX ) + this->xLeft;
        //randomPoint.y = (this->yUp-this->yBottom) * ( rand() / (double)RAND_MAX ) + this->yBottom;

        obstacle.pose.position.x = rrt.shiftCordinate(xleft_, xRight_, rrt.getRanddomDouble(xleft_, xRight_),
                                                      obstacle.scale.x);
        obstacle.pose.position.y = rrt.shiftCordinate(yBottom_, yUp_, rrt.getRanddomDouble(yBottom_, yUp_),
                                                      obstacle.scale.y);

        obstacle.pose.position.z = 0.2;
        obstacle.pose.orientation.x = 0.0;
        obstacle.pose.orientation.y = 0.0;
        obstacle.pose.orientation.z = 0.0;
        obstacle.pose.orientation.w = 1;
        obstacle.color.a = 1.0;
        obstacle.color.b = 0.75f;
        obstacle.color.r = obstacle.color.g = 0.70f;
        obstacles.push_back(obstacle);

    }




}


/*
 * Draw lines in Rviz
 *
 * */
void drawLines(ros::Publisher marker_pub, Node &n1, Node &n2, float frame_count) {

    //we use static here since we want to incrementally add contents in these mesgs, otherwise contents in these msgs will be cleaned in every ros spin.
    static visualization_msgs::Marker vertices, edges;

    vertices.type = visualization_msgs::Marker::POINTS;
    edges.type = visualization_msgs::Marker::LINE_LIST;

    vertices.header.frame_id = edges.header.frame_id = "map";
    vertices.header.stamp = edges.header.stamp = ros::Time::now();
    vertices.ns = "vertices";
    edges.ns = "lines";
    vertices.action = edges.action = visualization_msgs::Marker::ADD;
    vertices.pose.orientation.w = edges.pose.orientation.w = 1.0;

    vertices.id = frame_count;
    edges.id = frame_count;

    // POINTS markers use x and y scale for width/height respectively
    vertices.scale.x = 0.04;
    vertices.scale.y = 0.04;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    edges.scale.x = 0.02; //tune it yourself

    // Points are green
    vertices.color.g = 1.0f;
    vertices.color.a = 1.0;

    // Line list is red
    edges.color.r = 1.0;
    edges.color.a = 1.0;

    //adding vertices and edges
    vertices.points.push_back(n1.point);    //for drawing vertices
    //vertices.points.push_back(n2.point);	//for drawing vertices
    edges.points.push_back(n1.point);    //for drawing edges. The line list needs two points for each line
    edges.points.push_back(n2.point);

    //publish msgs
    marker_pub.publish(vertices);
    marker_pub.publish(edges);

}


/**
 * This function checks whether new point is coliding with an obstacle
 */

bool checkForObjectIntersection(Point &p1, Point &p2, vector<visualization_msgs::Marker> &obstacles,
                                double paddingForObjectAvoidance, RRT &rrt) {

    Geometry::LineSegment line_seg;
    line_seg.first = p1;
    line_seg.second = p2;

    for (int i = 0; i < obstacles.size(); i++) {

        // Creating points for boundary edges
        double xLower = obstacles[i].pose.position.x - obstacles[i].scale.x / 2.0 - paddingForObjectAvoidance;
        double xUpper = obstacles[i].pose.position.x + obstacles[i].scale.x / 2.0 + paddingForObjectAvoidance;
        double yLower = obstacles[i].pose.position.y - obstacles[i].scale.y / 2.0 - paddingForObjectAvoidance;
        double yUpper = obstacles[i].pose.position.y + obstacles[i].scale.y / 2.0 + paddingForObjectAvoidance;
        Point lB, rB, tL, tR;
        lB.x = xLower;
        lB.y = yLower;
        rB.x = xUpper;
        rB.y = yLower;
        tL.x = xLower;
        tL.y = yUpper;
        tR.x = xUpper;
        tR.y = yUpper;

        //Uncomment this block for better accuracy for higher delta
        /*
        //Line segments from obs
        Geometry::LineSegment segment1 = rrt.geometry.getLineSegment(lB, rB);
        Geometry::LineSegment segment2 = rrt.geometry.getLineSegment(rB, tR);
        Geometry::LineSegment segment3 = rrt.geometry.getLineSegment(tR, tL);
        Geometry::LineSegment segment4 = rrt.geometry.getLineSegment(tL, lB);

        // line intersetcs
        bool inter1 = rrt.geometry.doLinesIntersect(line_seg, segment1);
        bool inter2 = rrt.geometry.doLinesIntersect(line_seg, segment2);
        bool inter3 = rrt.geometry.doLinesIntersect(line_seg, segment3);
        bool inter4 = rrt.geometry.doLinesIntersect(line_seg, segment4);
        bool inside = rrt.geometry.checkInsideRectSimple(lB, rB, tR, tL, p2);
         //Boundry intersect check and inside rectangle check
        if (inter1 || inter2 || inter3 || inter4 || inside )
            return true;

        */
        bool inside = rrt.geometry.checkInsideRectSimple(lB, rB, tR, tL, p2);

        //Boundry intersect check and inside rectangle check
        if (inside)
            return true;
    }
    return false;

}

/**
 * This function checks whether new point is coliding with an obstacle using additional contrains
 */

bool checkForComplexObjectIntersection(Point &p1, Point &p2, vector<visualization_msgs::Marker> &obstacles,
                                double paddingForObjectAvoidance, RRT &rrt) {

    Geometry::LineSegment line_seg;
    line_seg.first = p1;
    line_seg.second = p2;

    for (int i = 0; i < obstacles.size(); i++) {

        // Creating points for boundary edges
        double xLower = obstacles[i].pose.position.x - obstacles[i].scale.x / 2.0 - paddingForObjectAvoidance;
        double xUpper = obstacles[i].pose.position.x + obstacles[i].scale.x / 2.0 + paddingForObjectAvoidance;
        double yLower = obstacles[i].pose.position.y - obstacles[i].scale.y / 2.0 - paddingForObjectAvoidance;
        double yUpper = obstacles[i].pose.position.y + obstacles[i].scale.y / 2.0 + paddingForObjectAvoidance;
        Point lB, rB, tL, tR;
        lB.x = xLower;
        lB.y = yLower;
        rB.x = xUpper;
        rB.y = yLower;
        tL.x = xLower;
        tL.y = yUpper;
        tR.x = xUpper;
        tR.y = yUpper;

        //Line segments from obs
        Geometry::LineSegment segment1 = rrt.geometry.getLineSegment(lB, rB);
        Geometry::LineSegment segment2 = rrt.geometry.getLineSegment(rB, tR);
        Geometry::LineSegment segment3 = rrt.geometry.getLineSegment(tR, tL);
        Geometry::LineSegment segment4 = rrt.geometry.getLineSegment(tL, lB);

        // line intersetcs
        bool inter1 = rrt.geometry.doLinesIntersect(line_seg, segment1);
        bool inter2 = rrt.geometry.doLinesIntersect(line_seg, segment2);
        bool inter3 = rrt.geometry.doLinesIntersect(line_seg, segment3);
        bool inter4 = rrt.geometry.doLinesIntersect(line_seg, segment4);
        bool inside = rrt.geometry.checkInsideRectSimple(lB, rB, tR, tL, p2);

        //Boundry intersect check and inside rectangle check
        if (inter1 || inter2 || inter3 || inter4 || inside )
            return true;

    }
    return false;

}

/**
 * This function validate a new points for all criteria
 */
bool validateNewPoint(Node *newNode, Point &p1, Point &p2, vector<visualization_msgs::Marker> &obstacles,
                      double paddingForObjectAvoidance, RRT &rrt) {

    return newNode->validity && (!checkForObjectIntersection(p1, p2, obstacles, paddingForObjectAvoidance, rrt));

}

/**
 * This function validate the end points for all criteria
 */
bool validateEndPoint(Point &p1, Point &p2, vector<visualization_msgs::Marker> &obstacles,
                      double paddingForObjectAvoidance, RRT &rrt, double endPointVisibility) {

    return (rrt.getDistance(p1, p2) <= endPointVisibility) &&
           (!checkForObjectIntersection(p1, p2, obstacles, paddingForObjectAvoidance, rrt));

}

/*
 * Draw Final Configuration in Rviz
 *
 * */
void drawRetrivedConfiguration(ros::Publisher marker_pub, Node &n1, Node &n2, float frame_count) {

    //we use static here since we want to incrementally add contents in these mesgs, otherwise contents in these msgs will be cleaned in every ros spin.
    static visualization_msgs::Marker vertices, edges;

    vertices.type = visualization_msgs::Marker::POINTS;
    edges.type = visualization_msgs::Marker::LINE_LIST;

    vertices.header.frame_id = edges.header.frame_id = "map";
    vertices.header.stamp = edges.header.stamp = ros::Time::now();
    vertices.ns = "vertices";
    edges.ns = "lines";
    vertices.action = edges.action = visualization_msgs::Marker::ADD;
    vertices.pose.orientation.w = edges.pose.orientation.w = 1.0;

    vertices.id = frame_count;
    edges.id = frame_count;

    // POINTS markers use x and y scale for width/height respectively
    vertices.scale.x = 0.05;
    vertices.scale.y = 0.05;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    edges.scale.x = 0.03; //tune it yourself

    // Points are green + red
    vertices.color.g = vertices.color.r = 1.0f;
    vertices.color.a = 1.0;

    // Line list is red + blue
    edges.color.r = edges.color.b = 1.0;
    edges.color.a = 1.0;

    //adding vertices and edges
    vertices.points.push_back(n1.point);    //for drawing vertices
    //vertices.points.push_back(n2.point);	//for drawing vertices
    edges.points.push_back(n1.point);    //for drawing edges. The line list needs two points for each line
    edges.points.push_back(n2.point);

    //publish msgs
    marker_pub.publish(vertices);
    marker_pub.publish(edges);

}


/**
 * This funtion smooth the final path
 */
void getshortestPath(vector<visualization_msgs::Marker> &obstacles, double paddingForObjectAvoidance, RRT &rrt){

    //Adding start node
    rrt.smoothedFinalConfiguration.push_back( rrt.finalConfiguration[rrt.finalConfiguration.size()-1]);
    //Dummy node for validation pass
    Node* test = new Node();
    test->validity = true;

    for(int i = rrt.finalConfiguration.size()-1 ; i > 0; i--){
        for(int j = 0 ; j < i; j++)
            if(!checkForComplexObjectIntersection(rrt.finalConfiguration[i].point, rrt.finalConfiguration[j].point, obstacles,paddingForObjectAvoidance, rrt)) {
                rrt.smoothedFinalConfiguration.push_back(rrt.finalConfiguration[j]);
                i = j+1;
                break;
            }
    }
}


/*
 * Drawing shortest path in Rviz
 *
 * */
void drawShortestConfiguration(ros::Publisher marker_pub, Node &n1, Node &n2, float frame_count) {

    //we use static here since we want to incrementally add contents in these mesgs, otherwise contents in these msgs will be cleaned in every ros spin.
    static visualization_msgs::Marker vertices, edges;

    vertices.type = visualization_msgs::Marker::POINTS;
    edges.type = visualization_msgs::Marker::LINE_LIST;

    vertices.header.frame_id = edges.header.frame_id = "map";
    vertices.header.stamp = edges.header.stamp = ros::Time::now();
    vertices.ns = "vertices";
    edges.ns = "lines";
    vertices.action = edges.action = visualization_msgs::Marker::ADD;
    vertices.pose.orientation.w = edges.pose.orientation.w = 1.0;

    vertices.id = frame_count;
    edges.id = frame_count;

    // POINTS markers use x and y scale for width/height respectively
    vertices.scale.x = 0.05;
    vertices.scale.y = 0.05;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    edges.scale.x = 0.05; //tune it yourself

    // Points are green + red
    vertices.color.g = vertices.color.r = 1.0f;
    vertices.color.a = 0.9;

    // Line list is red + blue
    edges.color.g = 0.0f;
    edges.color.b = 1.0f;
    edges.color.r = 0.0f;
    edges.color.a = 0.9;

    //adding vertices and edges
    vertices.points.push_back(n1.point);    //for drawing vertices
    //vertices.points.push_back(n2.point);	//for drawing vertices
    edges.points.push_back(n1.point);    //for drawing edges. The line list needs two points for each line
    edges.points.push_back(n2.point);

    //publish msgs
    marker_pub.publish(vertices);
    marker_pub.publish(edges);

}


/*
 * Drawing smoothed path in Rviz
 *
 * */
void drawSmoothedConfiguration(ros::Publisher marker_pub, Node &n1, Node &n2, float frame_count) {

    //we use static here since we want to incrementally add contents in these mesgs, otherwise contents in these msgs will be cleaned in every ros spin.
    static visualization_msgs::Marker vertices, edges;

    vertices.type = visualization_msgs::Marker::POINTS;
    edges.type = visualization_msgs::Marker::LINE_LIST;

    vertices.header.frame_id = edges.header.frame_id = "map";
    vertices.header.stamp = edges.header.stamp = ros::Time::now();
    vertices.ns = "vertices";
    edges.ns = "lines";
    vertices.action = edges.action = visualization_msgs::Marker::ADD;
    vertices.pose.orientation.w = edges.pose.orientation.w = 1.0;

    vertices.id = frame_count;
    edges.id = frame_count;

    // POINTS markers use x and y scale for width/height respectively
    vertices.scale.x = 0.08;
    vertices.scale.y = 0.08;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    edges.scale.x = 0.08; //tune it yourself

    // Points are green
    vertices.color.g = 1.0f;
    vertices.color.a = 1.0;

    // Line list is red + blue
    edges.color.g = 1.0;
    edges.color.a = 1.0;

    //adding vertices and edges
    vertices.points.push_back(n1.point);    //for drawing vertices
    //vertices.points.push_back(n2.point);	//for drawing vertices
    edges.points.push_back(n1.point);    //for drawing edges. The line list needs two points for each line
    edges.points.push_back(n2.point);

    //publish msgs
    marker_pub.publish(vertices);
    marker_pub.publish(edges);

}

void drawRobot(ros::Publisher marker_pub, Node &node) {

    static visualization_msgs::Marker robot;
    robot.type = visualization_msgs::Marker::CUBE;


    robot.header.frame_id = "map";
    robot.header.stamp = ros::Time::now();
    robot.ns = "Robot";
    robot.id = 10000014;
    robot.action = visualization_msgs::Marker::ADD;
    robot.lifetime = ros::Duration();
    robot.scale.x = 1.0;
    robot.scale.y = 0.5;
    robot.scale.z = 0.4;
    robot.pose.orientation = node.orienttation;
    //robot.pose.orientation.w = 1;
    //robot.pose.orientation.x = robot.pose.orientation.y = robot.pose.orientation.z = 0;
    robot.color.r = 1.0f;
    robot.color.g = 1.0f;
    robot.color.b = 0.2f;
    robot.color.a = 1.0;
    robot.pose.position = node.point;

    marker_pub.publish(robot);

}
