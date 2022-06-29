#ifndef QUADRUPEDV2_PID_H
#define QUADRUPEDV2_PID_H

#include "MPU6050.h"

class PID {
    double past;
    double integral;
    double Ki = 0.00001;

    double present;
    double error;
    double Ke = 0.5;

    double future;
    double derivative;
    double last;
    double now;
    double Kd = 0.001;


    absolute_time_t lastTime;
    long double errorSum;
    float bound = 0.2;
    int skip = 0;
};


#endif //QUADRUPEDV2_PID_H
