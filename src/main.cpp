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

[[maybe_unused]] void NRFTest();



float roll = 0, pitch;

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
        //rotateBody();
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

    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(15, GPIO_FUNC_I2C);
    gpio_set_function(14, GPIO_FUNC_I2C);
    gpio_pull_up(15);
    gpio_pull_up(14);

    mpu6050_reset();

    int16_t acceleration[3], gyro[3], temp;


    //printf("Acc. X = %6.2d, Y = %6.2d, Z = %6.2d ", acceleration[0], acceleration[1],acceleration[2]);
    //printf("Angles. Roll = %6.2f, Pitch = %6.2f\n",roll , pitch);
    mpu6050_reset();

    defineServo();


    float offset = 20;

    double past;
    double integral = 0;
    double intConst = 1;

    double present;
    double error;
    double errorConst = 0.5;

    double future;
    double derivative;
    double last;
    double now;
    double derConst = 0.001;


    absolute_time_t lastTime;
    long double errorSum;
    float bound = 0.2;
    int skip = 0;

    while (1) {
        mpu6050_read_raw(acceleration, gyro, &temp);

        roll =  atanf( - acceleration[0] / sqrtf(acceleration[1]*acceleration[1] + acceleration[2]*acceleration[2]));
        pitch = (atanf( - acceleration[1] / sqrtf(acceleration[0]*acceleration[0] + acceleration[2]*acceleration[2])));

        error = ( 0 - roll ) ;
        present = errorConst * error;

        integral += error;
        past = intConst * integral;

        derivative = gyro[0];
        future = derConst * derivative;

        errorSum = present  ;

        printf("Error sum: %.3f Future: %.3f Present: %.3f Past: %.3f \n", errorSum , future, present, past );
        printf("Roll: %.3f Gyro: %.3hd Accel: %.3hd \n", roll , gyro[0], acceleration[0] );

        for (int i = 0; i <= 3; ++i) {
            float y = leg[i].lastPos.y,
                    z = leg[i].lastPos.z;
            //todo use PID intead of bound;
            leg[i].toPos( Vector(60, 60, offset + 60 /
            //tanf( TanjentWithBound( roll, i, 0.5 ) ) )
            tanf(PI / 3 + (errorSum) * powf(-1 , i)) )
            );
        }

        mpu6050_reset();
        sleep_ms(150 );
    }
}



