#include "MPU6050.h"
#include "cmath"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

void MPU6050::initMPU(int gpio1 , int gpio2 ) {
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(gpio1, GPIO_FUNC_I2C);
    gpio_set_function(gpio2, GPIO_FUNC_I2C);
    gpio_pull_up(gpio1);
    gpio_pull_up(gpio2);
    bi_decl(bi_2pins_with_func(gpio1, gpio2, GPIO_FUNC_I2C));

    reset();
}
void MPU6050::reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c1, addr, buf, 2, false);
}
void MPU6050::readRaw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c1, addr, &val, sizeof(val), true);
    i2c_read_blocking(i2c1, addr, buffer, sizeof(buffer), false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
    val = 0x43;
    i2c_write_blocking(i2c1, addr, &val, sizeof(val), true);
    i2c_read_blocking(i2c1, addr, buffer, sizeof(buffer), false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
    val = 0x41;
    i2c_write_blocking(i2c1, addr, &val, sizeof(val), true);
    i2c_read_blocking(i2c1, addr, buffer, sizeof(buffer), false);

    *temp =  (buffer[0] << 8 | buffer[1]);
}

double MPU6050::getRollRaw(){
    return atanf( - acceleration[0] / sqrtf( acceleration[1] * acceleration[1] + acceleration[2] * acceleration[2] ) );
}
double MPU6050::getPitchRaw() {
    return atanf( - acceleration[1] / sqrtf( acceleration[0] * acceleration[0] + acceleration[2] * acceleration[2] ) );
}

void MPU6050::calculateAverageAcceleration() {
    sumRawAcceleration.al  = 0;
    sumRawAcceleration.bet = 0;
    sumRawAcceleration.gam = 0;

    for (int i = 0; i < updateTimes; ++i)
    {
        sumRawAcceleration.al  += acceleration[0];
        sumRawAcceleration.bet += acceleration[1];
        sumRawAcceleration.gam += acceleration[2];
    }

    sumRawAcceleration.al  /= updateTimes;
    sumRawAcceleration.bet /= updateTimes;
    sumRawAcceleration.gam /= updateTimes;
}
void MPU6050::calculateAverageGyro() {
    sumRawGyroscope.al  = 0;
    sumRawGyroscope.bet = 0;
    sumRawGyroscope.gam = 0;

    for (int i = 0; i < updateTimes; ++i)
    {
        sumRawGyroscope.al  += gyroscope[0];
        sumRawGyroscope.bet += gyroscope[1];
        sumRawGyroscope.gam += gyroscope[2];
    }

    sumRawGyroscope.al  /= updateTimes;
    sumRawGyroscope.bet /= updateTimes;
    sumRawGyroscope.gam /= updateTimes;
}
double MPU6050::getRoll(){
    return atanf( - sumRawAcceleration.al  / sqrtf( sumRawAcceleration.gam * sumRawAcceleration.gam + sumRawAcceleration.bet * sumRawAcceleration.bet ) );
}
double MPU6050::getPitch(){
    return atanf( - sumRawAcceleration.gam / sqrtf( sumRawAcceleration.al * sumRawAcceleration.al + sumRawAcceleration.bet * sumRawAcceleration.bet ) );
}