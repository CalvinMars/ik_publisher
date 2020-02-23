#include <stdlib.h>
#include <ctime>
#include <chrono>
#include <iostream>
#include "IKSolve.h"
// #include "ros/ros.h"

#include <sstream>

using namespace std;

void ikCallback(const sensor_msgs::Joy::ConstPtr& msgJoy){
    
    static ros::Time lastMsgTime = 0;
    static ros::Time lastRecvTime = 0;

    // Record timestamp from joy message
    double recvTime = ros::Time::now().toSec();
    double msgTime = msgJoy->header.stamp.toSec();

    // Filter old/delayed joy messages messages
    if (msgTime < (recvTime - 0.25)){
        ROS_INFO("Ignoring " + (recvTime - msgTime) + " second old message");
        exit();
    }

    // Filter messages at frequency
    if (msgTime < (this.lastMsgTime + 1.0 / this.MSG_PER_SECOND)){
        exit();
    } else {
        lastRecvTime = recvTime;
        lastMsgTime = msgTime;
    }



    sensor_msgs::JointState msgNew;

    msgNew = joyParse(msgJoy);
    ik_pub.publish(msgNew);
}

int main(int argc, char **argv){

    // Initialize the ROS node
    ros::init(argc, argv, "ik_publisher")

    ros::NodeHandle n;
    ros::Publisher ik_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10)
    ros::Rate loop_rate(10);

    ros::Subscriber sub = n.subscribe("joy", 1000, ikCallback);



    int MSG_PER_SECOND = 60;

    ros::spin();
    }
}