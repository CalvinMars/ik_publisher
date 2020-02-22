#include <stdlib.h>
#include "IKSolve.h"

#include <sstream>

using namespace std;

int main(int arc, cahr **argv){

    // Initialize the ROS node
    ros::init(argc, argv, "ik_publisher")

    ros::NodeHandle n;
    ros::Publisher ik_pub = n.advertise<std_msgs::JointState>("joint_states", 10)
    ros::Rate loop_rate(10);

    ros::Subscriber sub = n.subscribe("joy", 1000, ikCallback);

    void IKSolve::ikCallback(const std_msgs::Joy::ConstPtr &msgJoy){
        std_msgs::JointState msgNew;

        msgNew = joyParse(msgJoy);
        ik_pub.publish(msgNew);
    }

    ros::spin();
    }
}