#ifndef IKSOLVE_H_
#define IKSOLVE_H_

#include <stdlib.h>
#include <math.h> 
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"


using namespace std;
class IKSolve {
    public:
        void joyParse(const sensor_msgs::Joy::ConstPtr& msg);
        double * ikSolve(double coords[], double alpha);
    private:
        static const double pi = 3.1415926535897932384626433;
        static const double SHOULDER_LEN = 10;
        static const double ELBOW_LEN = 10;
        static const double WRIST_LEN = 4;
        double coords[2] = {13, 10};
        double alpha = 0;
        double turretAngle = 0;
        double wristAngle = 0;

};

#endif
