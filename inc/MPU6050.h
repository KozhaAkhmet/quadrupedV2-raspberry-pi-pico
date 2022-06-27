#ifndef QUADRUPEDV2__MPU6050_H_
#define QUADRUPEDV2__MPU6050_H_
#include "hardware/i2c.h"


static int addr = 0x68;

void mpu6050_reset();
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) ;

class MPU6050{
public:
    int16_t acceleration[3], gyroscope[3];
    void initMPU();
    double getRoll();
    double getPitch();
};
#endif