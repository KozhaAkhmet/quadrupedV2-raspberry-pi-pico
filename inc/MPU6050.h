#ifndef QUADRUPEDV2__MPU6050_H_
#define QUADRUPEDV2__MPU6050_H_
#include "hardware/i2c.h"
#include "Eigen"

#define updateTimes 10
static int addr = 0x68;

void mpu6050_reset() {
uint8_t buf[] = {0x6B, 0x00};
i2c_write_blocking(i2c1, addr, buf, 2, false);
}

void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
uint8_t buffer[6];
uint8_t val = 0x3B;
i2c_write_blocking(i2c1, addr, &val, sizeof(val), false); 
i2c_read_blocking(i2c1, addr, buffer, sizeof(buffer), false);
  for (int i = 0; i < 3; i++) {
    accel[i] = int16_t  (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
  }
val = 0x43;
i2c_write_blocking(i2c1, addr, &val, sizeof(val), false); 
i2c_read_blocking(i2c1, addr, buffer, sizeof(buffer), false);
  for (int i = 0; i < 3; i++) {
    gyro[i] = int16_t  (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
  }
val = 0x41;
i2c_write_blocking(i2c1, addr, &val, sizeof(val), false); 
i2c_read_blocking(i2c1, addr, buffer, sizeof(buffer), false);

*temp =  int16_t (buffer[0] << 8 | buffer[1]);
}

class MPU6050{
public:
    int16_t acceleration[3], gyroscope[3],temp;
    Eigen::Vector3f sumRawAcceleration;
    Eigen::Vector3f sumRawGyroscope;

    static void initMPU(int gpio1 = 15, int gpio2 = 14);
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