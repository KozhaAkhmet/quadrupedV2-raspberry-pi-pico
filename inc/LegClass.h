#ifndef QUADRUPEDV2_LEGCLASS_H
#define QUADRUPEDV2_LEGCLASS_H

#include "Servo.h"
#include "Eigen"

#define PI 3.14

#define J1 83                                          //Defining Leg`s length in mm
#define J2 55
#define J3 33


class Leg {                                                        //Creating Leg class and its ingredients
public:
    Eigen::Vector3f lastAng;
    Eigen::Vector3f lastPos;
    Servo servo[3];
    void toPos(Eigen::Vector3f* pos);
    void toAng(Eigen::Vector3f* ang);
    bool stepCycle(float direction, float stepLength, float freq);
    void slide(Eigen::Vector3f* targetPos);
};

#endif //QUADRUPEDV2_LEGCLASS_H
