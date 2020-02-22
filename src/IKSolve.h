#ifndef IKSOLVE_H_
#define IKSOLVE_H_

#include <stdlib.h>
#include <math.h> 
#include "ros/ros.h"
#include "std_msgs/Joy.h"
#include "std_msgs/JointState.h"

#endif

using namespace std;
class IKSolve {
    public:
        void joyParse(const std_msgs::Joy::ConstPtr& msg);
        double * ikSolve(double coords[], double alpha);
    private:
        const double pi = 3.1415926535897932384626433;
        const double SHOULDER_LEN = 10;
        const double ELBOW_LEN = 10;
        const double WRIST_LEN = 4;
        double coords[2] = {13, 10};
        double alpha = 0;
        double turretAngle = 0;
        double wristAngle = 0;

}

