#ifndef IKSOLVE_H_
#define IKSOLVE_H_

#include <stdlib.h>
#include <math.h> 
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"



using namespace std;
class IKSolve {
    public:
        IKSolve(int argc, char **argv);
        void ikCallback(const sensor_msgs::Joy::ConstPtr& msgJoy);
        double* joyParse(const sensor_msgs::Joy::ConstPtr& msg);
        double* ikSolve(double* coords, double alpha);
    private:
        ros::Publisher ik_pub;
        double lastMsgTime;
        double lastRecvTime;
        static constexpr int MSG_PER_SECOND = 60;
        static constexpr double pi = 3.1415926535897932384626433;
        static constexpr double SHOULDER_LEN = 10;
        static constexpr double ELBOW_LEN = 10;
        static constexpr double WRIST_LEN = 4;
        double coords[2] = {13, 10};
        double alpha = 0;
        double turretAngle = 0;
        double wristAngle = 0;

};

#endif
