#ifndef __MYSERVO_H_
#define __MYSERVO_H_

struct Servo{                                                  //Defining Servo class
    public:
    int servoPin;
    float range[2];
    void init() const;
    void write(float pusle) const;
};
#endif