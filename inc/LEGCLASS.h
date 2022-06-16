#ifndef MAIN_LEGCLASS_H
#define MAIN_LEGCLASS_H

#include "MYSERVO.h"

#define PI 3.14

#define J1 83                                          //Defining Leg`s length in mm
#define J2 55
#define J3 33


float map(float x, float in_min, float in_max, float out_min, float out_max);

struct Ang {
    float al;
    float bet;
    float gam;
};
struct Pos {
    float x;
    float y;
    float z;
};
class Leg {                                                        //Creating Leg class and its ingredients
public:
    Ang lastAng;
    Pos lastPos;
    Servo servo[3];
    void toPos(float posX, float posY, float posZ);
    void toAng(float al, float bet , float gam );
    void stepCycle(float dis, float omega, float freq);
    void slide(float posX, float posY ,float posZ);
};

#endif //MAIN_LEGCLASS_H
