#ifndef QUADRUPEDV2__MPU6050_H_
#define QUADRUPEDV2__MPU6050_H_
#include "hardware/i2c.h"
#include "Matrix.h"

#define times 10

static int addr = 0x68;
Angle sumRawAcceleration;
Angle sumRawGyroscope;

class MPU6050{
public:
    int16_t acceleration[3], gyroscope[3],temp;
    void initMPU();
    double getRollRaw();
    double getPitchRaw();
    void reset();
    void readRaw(int16_t *accel, int16_t *gyro, int16_t *temp);

    void calculateAverageAcceleration();
    void calculateAverageGyro();

    double getPitch();
    double getRoll();
};
#endif