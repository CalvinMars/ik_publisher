#include "IKSolve.h"

void IKSolve::joyParse(const std_msgs::Joy::ConstPtr &msg)
{
    double jointVals[6] = {};
    double *ikJoints = new double[]();

    // TODO: find 'a' button number
    if (msg->button[1] == 1) // If 'a' button is pushed, reset
    {
        for (double angle: jointVals)
            {
                angle = 0.0;
            }
    }
    else
    {
        // Recieve data from message
        coords[0] += msg->axis[0] / 100; //TODO: set axes to the right axis!!!
        coords[1] += msg->axis[1] / 100; // <---- the same here (and below)
        alpha += msg->axis[3] / 100;
        turretAngle += msg->axis[5] / 100;
        turretAngle -= msg->axis[6] / 100;
        wristAngle += msg->button[1];
        wristAngle -= msg->button[2];

        ikJoints = ikSolve(coords, alpha);

    }
}

double *IKSolve::ikSolve(double coords[], double alpha)
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
    double theta2 = phi1 + phi2 - pi();
    double phi3 = (pi() - theta2) + alpha;
    double returnArray [3] = {phi1, phi2, phi3};
    return returnArray;
}
