#include <stdlib.h>
#include <ctime>
#include <chrono>
#include <iostream>
#include "IKSolve.h"

#include <sstream>

using namespace std;

void IKSolve::ikCallback(const std_msgs::Joy::ConstPtr &msgJoy){
    
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
        this.lastRecvTime = recvTime;
        this.lastMsgTime = msgTime;
    }



    std_msgs::JointState msgNew;

    msgNew = joyParse(msgJoy);
    ik_pub.publish(msgNew);
}

int main(int arc, cahr **argv){


    // Initialize the ROS node
    ros::init(argc, argv, "ik_publisher")

    ros::NodeHandle n;
    ros::Publisher ik_pub = n.advertise<std_msgs::JointState>("joint_states", 10)
    ros::Rate loop_rate(10);

    ros::Subscriber sub = n.subscribe("joy", 1000, ikCallback);

    time_t lastMsgTime = 0;
    time_t lastRecvTime = 0;

    int MSG_PER_SECOND = 60;

    ros::spin();
    }
}