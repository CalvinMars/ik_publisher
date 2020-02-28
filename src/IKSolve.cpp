#include "IKSolve.h"

// using namespace std;

void IKSolve::ikCallback(const sensor_msgs::Joy::ConstPtr& msgJoy){
    
    //ROS_INFO("callback");

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
    msgNew.header.stamp = ros::Time::now();
    msgNew.position = {msgArray[0],msgArray[1],msgArray[2],msgArray[3],msgArray[4],msgArray[5]};
    msgNew.name = {"turret", "shoulder", "elbow", "wrist_tilt", "wrist_spin", "claw"};
    msgNew.velocity = {};
    msgNew.effort = {};
    // ROS_INFO("The claw has value ")
    for (double angle: msgNew.position)
    {
        //ROS_INFO("joinVals = %f", angle);
    }
    ik_pub.publish(msgNew);
    //ROS_INFO("Sent!");
}

IKSolve::IKSolve(int argc, char **argv){
    // Initialize the ROS node
    ros::init(argc, argv, "ik_publisher");

    ros::NodeHandle n;
    ik_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
    ros::Rate loop_rate(10);

    ros::Subscriber sub = n.subscribe("joy", 1000, &IKSolve::ikCallback, this);
    //ROS_INFO("test");
    ros::spin();
}

double* IKSolve::joyParse(const sensor_msgs::Joy::ConstPtr& msg)
{    
    //ROS_INFO("Solving IK");
 
    static double* ikJoints;

    // TODO: find 'a' button number
    if (msg->buttons[1] == 1) // If 'a' button is pushed, reset
    {
        for (int i = 0; i < 6; i++){
            ikJoints[i] = 0.0;
        }
        coords[0] = SHOULDER_LEN;
  ;      coords[1] = ELBOW_LEN+WRIST_LEN;
        alpha = 0;
        turretAngle = 0.0;
        clawAngle = 0.0;
        //ROS_INFO("Returned array [%f, %f, %f, %f, %f]", ikJoints[0], ikJoints[1], ikJoints[2], ikJoints[3], ikJoints[4]);

        return jointVals;

    }
    else
    {
        // Recieve data from message
        coords[0] -= zeroRound(msg->axes[0] / 100); 
        coords[1] += zeroRound(msg->axes[1] / 100); 
        alpha += zeroRound(msg->axes[4] / 100);
        turretAngle = msg->axes[2] - msg->axes[5];
        // turretAngle -= zeroRound(msg->axes[5] / 200);
        wristAngle += zeroRound(msg->axes[6] / 100);
        clawAngle -= double(msg->buttons[4])/100;
        clawAngle += double(msg->buttons[5])/100;
        // for (int i = 0; i < 8; i++) {
        //     ROS_INFO("Axis %i has value: %f", i, msg->axes[i]);
        //     ROS_INFO("Button %i has value: %i", i, msg->buttons[i]);
        // }

        ikJoints = ikSolve(coords, alpha);
        for (int i = 0; i < 3; i++){
            if (ikJoints[i] != ikJoints[i]){
                ROS_INFO("Joint %i is not feeling so good...", i);
            }
        } else {
            if (ikJoints[0] == nan || ikJoints[1] == nan || ikJoints[2] == nan){
                ROS_ERROR("Joint is not feeling so good...")
            }
            jointVals[0] = turretAngle;
            jointVals[1] = ikJoints[0];
            jointVals[2] = ikJoints[1];
            jointVals[3] = ikJoints[2];
            jointVals[4] = wristAngle;
            jointVals[5] = clawAngle;

            ROS_INFO("%f\t\t%f", jointVals[1], jointVals[2]);
        }

        // for (double angle: jointVals)
        // {
        //     ROS_INFO("joinVals = %f", angle);
        // }
        return jointVals;
    }

}

double IKSolve::zeroRound(double axis) const {
    //ROS_INFO("Deadzone starting...");
    if (fabs(axis) > 0.001) {
        //ROS_INFO("Returned axis...");
        return axis;
    } else {    
        //ROS_INFO("Axis: Fabs=%f, Value=%f", fabs(axis), axis);
        //ROS_INFO("Axis in deadzone..." );

        return 0.00;
    }
}

double *IKSolve::ikSolve(double* coords, double alpha)
{
    //ROS_INFO("working...");

    double j = coords[0];
    //ROS_INFO("j = %f", j);
    double k = coords[1];
    //ROS_INFO("k = %f", k);
    double m = j - WRIST_LEN * cos(alpha);
    //ROS_INFO("m = %f", m);


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
