#ifndef QUADRUPEDV2_LEGCLASS_H
#define QUADRUPEDV2_LEGCLASS_H

#include "Servo.h"
#include "Matrix.h"

#define PI 3.14

#define J1 83                                          //Defining Leg`s length in mm
#define J2 55
#define J3 33


float map(float x, float in_min, float in_max, float out_min, float out_max);

class Leg {                                                        //Creating Leg class and its ingredients
public:
    Angle lastAng;
    Vector lastPos;
    Servo servo[3];
    void toPos(Vector pos);
    void toAng(Angle ang);
    void stepCycle(float dis, float omega, float freq);
    void slide(Vector targetPos);
};

#endif //MAIN_LEGCLASS_H
