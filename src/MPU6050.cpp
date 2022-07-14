#include "MPU6050.h"
#include "hardware/i2c.h"
#include "cmath"
#include "pico/stdio.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

void MPU6050::initMPU() {
    stdio_init_all();

    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(15, GPIO_FUNC_I2C);
    gpio_set_function(14, GPIO_FUNC_I2C);
    gpio_pull_up(15);
    gpio_pull_up(14);

    reset();
}
void MPU6050::reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c1, addr, buf, 2, false);
}

void MPU6050::readRaw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
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


double MPU6050::getRollRaw(){
    return atanf( - acceleration[0] / sqrtf( acceleration[1] * acceleration[1] + acceleration[2] * acceleration[2] ) );
}
double MPU6050::getPitchRaw() {
    return atanf( - acceleration[1] / sqrtf( acceleration[0] * acceleration[0] + acceleration[2] * acceleration[2] ) );
}
 