#include "IKSolve.h"

// using namespace std;

void IKSolve::ikCallback(const sensor_msgs::Joy::ConstPtr& msgJoy){
    
    ROS_INFO("callback");

    // Record timestamp from joy message
    double recvTime = ros::Time::now().toSec();
    double msgTime = msgJoy->header.stamp.toSec();

    // Filter old/delayed joy messages messages
    if (msgTime < (recvTime - 0.25)){
        // ROS_INFO("Ignoring " + (recvTime - msgTime) + " second old message");
    } // Filter messages at frequency
    else if (msgTime < (lastMsgTime + 1.0 / MSG_PER_SECOND)){
    } else {
        lastRecvTime = recvTime;
        lastMsgTime = msgTime;
    }



    sensor_msgs::JointState msgNew;
    double* msgArray;
    msgArray = IKSolve::joyParse(msgJoy); 
    msgNew.header = std_msgs::Header();
    msgNew.position = {msgArray[0],msgArray[1],msgArray[2],msgArray[3],msgArray[4]};
    msgNew.name = {"turret", "shoulder", "elbow", "wrist_tilt", "wrist_spin"};
    msgNew.velocity = {};
    msgNew.effort = {};
    for (double angle: msgNew.position)
    {
        ROS_INFO("joinVals = %f", angle);
    }
    ik_pub.publish(msgNew);
    ROS_INFO("Sent!");
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
 
    static double* ikJoints;

    // TODO: find 'a' button number
    if (msg->buttons[1] == 1) // If 'a' button is pushed, reset
    {
        for (int i = 0; i < 5; i++){
            ikJoints[i] = 0.0;
        }
        coords[0] = 13;
        coords[1] =  10;
        alpha = 0;
        ROS_INFO("Returned array [%f, %f, %f, %f, %f]", ikJoints[0], ikJoints[1], ikJoints[2], ikJoints[3], ikJoints[4]);

        return jointVals;

    }
    else
    {
        // Recieve data from message
        coords[0] += msg->axes[0] / 100; //TODO: set axes to the right axis!!!
        coords[1] += msg->axes[1] / 100; // <---- the same here (and below)
        alpha += msg->axes[5] / 100;
        turretAngle += msg->axes[3] / 200;
        turretAngle -= msg->axes[4] / 200;
        wristAngle += msg->axes[6];
        // wristAngle -= msg->buttons[2];

        ikJoints = ikSolve(coords, alpha);
        ROS_INFO("From coords x=[%f], y=[%f]" , coords[0], coords[1]);
        // ROS_INFO("test 2");
        // ROS_INFO("testing value: %f", ikJoints[0]);
        ROS_INFO("Sending values: phi1=[%lf], phi2=[%lf], phi3=[%lf]", ikJoints[0] , ikJoints[1], ikJoints[2]);
        jointVals[0] = turretAngle;
        jointVals[1] = ikJoints[0];
        jointVals[2] = ikJoints[1];
        jointVals[3] = ikJoints[2];
        jointVals[4] = wristAngle;
        // for (double angle: jointVals)
        // {
        //     ROS_INFO("joinVals = %f", angle);
        // }
        return jointVals;
    }

}

double *IKSolve::ikSolve(double* coords, double alpha)
{
    ROS_INFO("working...");

    double j = coords[0];
    ROS_INFO("j = %f", j);
    double k = coords[1];
    ROS_INFO("k = %f", k);
    double m = j - WRIST_LEN * cos(alpha);
    ROS_INFO("m = %f", m);


    double n = k - WRIST_LEN * sin(alpha);
    // ROS_INFO("n = %f", n);
    double theta112 = atan(n/m);
    // ROS_INFO("theta112 = %f", theta112);
    double l = sqrt(pow(m,2)+pow(n,2));
    // ROS_INFO("l = %f", l);
    double theta13 = acos( ( pow(ELBOW_LEN,2) - pow(SHOULDER_LEN,2) - pow(l,2)) / ( -2 * l * SHOULDER_LEN ) );
    // ROS_INFO("theta13 = %f", theta13);
    double phi1 = theta112 + theta13;
    // ROS_INFO("phi1 = %f", phi1);
    double phi2 = acos( (pow(l,2) - pow(ELBOW_LEN,2) - pow(SHOULDER_LEN,2)) / (-2 * ELBOW_LEN * SHOULDER_LEN) );
    // ROS_INFO("phi2 = %f", phi2);
    double theta2 = phi1 + phi2 - pi;
    // ROS_INFO("theta2 = %f", theta2);
    double phi3 = (pi - theta2) + alpha;
    // ROS_INFO("phi3 = %f", phi3);
    returnArray[0] = phi1 - pi/2;
    returnArray[1] = phi2 - pi/2;
    returnArray[2] = phi3 - pi;
    // for (double angle: returnArray)
    // {
        // 
        // ROS_INFO("joinVals = %f", angle);
    // }
    // ROS_INFO("working...");
    // ROS_INFO("still working...");
    return returnArray;
}
