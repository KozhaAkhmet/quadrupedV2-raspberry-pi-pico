#ifndef __MYSERVO_H_
#define __MYSERVO_H_
#include "pico/stdlib.h"
#include "hardware/pwm.h"

struct Servo{                                                  //Defining Servo class
    public:
    int servoPin;
    int range[2];
    void init();
    void write(float pusle); 
};
#endif