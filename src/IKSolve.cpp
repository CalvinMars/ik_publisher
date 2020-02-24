#include "IKSolve.h"



void IKSolve::ikCallback(const sensor_msgs::Joy::ConstPtr& msgJoy){
    
    ROS_INFO("callback");

    // Record timestamp from joy message
    double recvTime = ros::Time::now().toSec();
    double msgTime = msgJoy->header.stamp.toSec();

    // Filter old/delayed joy messages messages
    if (msgTime < (recvTime - 0.25)){
        // ROS_INFO("Ignoring " + (recvTime - msgTime) + " second old message");
        exit(1);
    }

    // Filter messages at frequency
    if (msgTime < (lastMsgTime + 1.0 / MSG_PER_SECOND)){
        exit(1);
    } else {
        lastRecvTime = recvTime;
        lastMsgTime = msgTime;
    }



    sensor_msgs::JointState msgNew;
    double* msgArray;
    msgArray = IKSolve::joyParse(msgJoy);
    for (int i = 0; i < 6; i++){
        msgNew.position[i] = msgArray[i];
    }
    ik_pub.publish(msgNew);
}

IKSolve::IKSolve(int argc, char **argv){
    // Initialize the ROS node
    ros::init(argc, argv, "ik_publisher");

    ros::NodeHandle n;
    ik_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
    ros::Rate loop_rate(10);

    ros::Subscriber sub = n.subscribe("joy", 1000, &IKSolve::ikCallback, this);
    ROS_INFO("test");
    ros::spin();
}

double* IKSolve::joyParse(const sensor_msgs::Joy::ConstPtr& msg)
{    
    ROS_INFO("Solving IK");

    double jointVals[6] = {};
    double* ikJoints;

    // TODO: find 'a' button number
    if (msg->buttons[1] == 1) // If 'a' button is pushed, reset
    {
        for (double angle: jointVals)
            {
                angle = 0.0;
            }
    }
    else
    {
        // Recieve data from message
        coords[0] += msg->axes[0] / 100; //TODO: set axes to the right axis!!!
        coords[1] += msg->axes[1] / 100; // <---- the same here (and below)
        alpha += msg->axes[3] / 100;
        turretAngle += msg->axes[5] / 100;
        turretAngle -= msg->axes[6] / 100;
        wristAngle += msg->buttons[1];
        wristAngle -= msg->buttons[2];

        ikJoints = ikSolve(coords, alpha);
        ROS_INFO("From coords x=[%f], y=[%f]" , coords[0], coords[1]);
        ROS_INFO("test");
        ROS_INFO("Sending values: phi1=[%lf], phi2=[%lf], phi3=[%lf]", ikJoints[0] , ikJoints[1], ikJoints[2]);
        return jointVals;
    }
}

double *IKSolve::ikSolve(double* coords, double alpha)
{
    double j = coords[0];
    double k = coords[1];

    double m = j - WRIST_LEN * cos(alpha);
    double n = k - WRIST_LEN * sin(alpha);

    double theta112 = atan2(n,m);
    double l = sqrt(pow(m,2)+pow(n,2));
    double theta13 = acos( ( pow(ELBOW_LEN,2) + pow(SHOULDER_LEN,2) + pow(l,2)) / ( -2 * l * SHOULDER_LEN ) );
    double phi1 = theta112 + theta13;
    double phi2 = acos( (pow(l,2) - pow(ELBOW_LEN,2) - pow(SHOULDER_LEN,2)) / (-2 * ELBOW_LEN * SHOULDER_LEN) );
    double theta2 = phi1 + phi2 - pi;
    double phi3 = (pi - theta2) + alpha;
    double returnArray [3] = {phi1, phi2, phi3};
    ROS_INFO("working...");
    ROS_INFO("phi1 = %f", phi1);
    ROS_INFO("still working...");
    return returnArray;
}
