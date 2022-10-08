#ifndef QUADRUPEDV2__MYSERVO_H_
#define QUADRUPEDV2__MYSERVO_H_

#pragma once

struct Servo{                                                  //Defining Servo class
    public:
    int servoPin;
    float range[2];
    void init() const ;
    void write(float pusle) const ;
};
#endif