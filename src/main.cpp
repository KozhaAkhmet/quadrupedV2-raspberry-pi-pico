/*
* Made by Kozha Akhmet Abdramanov
* Quadruped Robot V2 on Raspberry Pi Pico
*
*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "cmath"


#include "NRF24.h"
#include "MPU6050.h"
#include "Motions.h"

void MPUTest();
void NRFTest();


float roll, pitch;

int main() {                                                         //Main Function
    //multicore_launch_core1(walkCycle); todo reorganize multicore

    MPUTest();

    while (true) {
        //walk();
        //test();
        //leg[1].slide(80,40,60);
        //walkCycle();
        //rotationCycle(1)
        //walkCycle();
        //bodyCircularMotion();
    }
}


void NRFTest() {
    uint8_t addr[6] = "Node1";
    NRF24 nrf(spi0, 16, 17);
    nrf.config();
    nrf.modeTX();
    //todo fix address. Can`t recognize controller address.
    char buffer[32];
    while (1) {
        sprintf(buffer, "60");
        buffer[30] = 'R';
        buffer[31] = 'O'; // not a zero.
        nrf.sendMessage(buffer);
        sleep_ms(1000);

        sprintf(buffer, "-60");
        buffer[30] = 'R';
        buffer[31] = 'O'; // not a zero.
        nrf.sendMessage(buffer);
        sleep_ms(1000);

    }
}

void MPUTest(){
    //todo merge MPU6050 lib with new roll, pitch calculation functions.
    stdio_init_all();
    //printf("Hello, MPU6050! Reading raw data from registers...\n");

    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(15, GPIO_FUNC_I2C);
    gpio_set_function(14, GPIO_FUNC_I2C);
    gpio_pull_up(15);
    gpio_pull_up(14);

    //bi_decl(bi_2pins_with_func(15, 14,GPIO_FUNC_I2C));
    mpu6050_reset();

    int16_t acceleration[3], gyro[3], temp;

    defineServo();
    defaultPos();
    float offset = 20;
    while (1) {

        pitch = (atanf( - acceleration[1] / sqrtf(acceleration[0]*acceleration[0] + acceleration[2]*acceleration[2]))*180.0)/PI;
//        roll =  (atanf( - acceleration[0] / sqrtf(acceleration[1]*acceleration[1] + acceleration[2]*acceleration[2]))*180.0)/PI;
        roll =  atanf( - acceleration[0] / sqrtf(acceleration[1]*acceleration[1] + acceleration[2]*acceleration[2]));
        mpu6050_read_raw(acceleration, gyro, &temp);
        printf("Acc. X = %6.2d, Y = %6.2d, Z = %6.2d ", acceleration[0], acceleration[1],acceleration[2]);
        printf("Angles. Roll = %6.2f, Pitch = %6.2f\n",roll , pitch);
        mpu6050_reset();

        sleep_ms(180 + offset * 1.3);
        for (int i = 0; i <= 3; ++i) {
            float y = leg[i].lastPos.y,
                    z = leg[i].lastPos.z;
            leg[i].toPos( Vector(60, 60, offset + 60 /
            tanf(
                    (PI / 3 + roll * powf(-1,i))
                    ) / (PI/2)
                          )
            );
        }
    }

}



