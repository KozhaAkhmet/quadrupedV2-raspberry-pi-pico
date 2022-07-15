#ifndef QUADRUPEDV2_PID_H
#define QUADRUPEDV2_PID_H

#include "MPU6050.h"

double setBounds(double input, const double min , const double max){
    if (input >= max) input = max;
    if (input <= min) input = min;
    return input;
}
//class Bounds{
//public:
//    double lower;
//    double upper;
//    Bounds(){};
//};
//Bounds integralBound;

class PID {
private:
    double present{};
    double error{};
    double Ke = 1;
    double pBounds[2];

    double past{};
    double integral{};
    double Ki = 1;
    double iBounds[2];

    double future{};
    double derivative{};
    double dBounds[2];
    double Kd = 1;

public:
    PID(){};

    void setP(double Ke, double lowerBound, double upperBound);
    void setI(double Ki, double lowerBound, double upperBound);
    void setD(double Kd, double lowerBound, double upperBound);

    void calculatePID(double value, double der);
    void calculatePID(double value, double targetValue, double der);

    double getP();

    double getI();

    double getD();

    double getPID();

    double getPD();
};


#endif //QUADRUPEDV2_PID_H
