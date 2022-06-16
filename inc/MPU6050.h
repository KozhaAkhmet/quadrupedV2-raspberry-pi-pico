#ifndef __MPU6050_H_
#define __MPU6050_H_
#include "hardware/i2c.h"


static int addr = 0x68;

void mpu6050_reset();
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) ;

#endif