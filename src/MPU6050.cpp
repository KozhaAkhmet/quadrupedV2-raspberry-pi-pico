#include "MPU6050/MPU6050.h"
#include "hardware/i2c.h"

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
    accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
  }
val = 0x43;
i2c_write_blocking(i2c1, addr, &val, sizeof(val), false); 
i2c_read_blocking(i2c1, addr, buffer, sizeof(buffer), false);
  for (int i = 0; i < 3; i++) {
    gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
  }
val = 0x41;
i2c_write_blocking(i2c1, addr, &val, sizeof(val), false); 
i2c_read_blocking(i2c1, addr, buffer, sizeof(buffer), false);

*temp = buffer[0] << 8 | buffer[1];
}
 