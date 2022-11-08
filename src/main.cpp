/*
* Made by Kozha Akhmet Abdramanov
* Quadruped Robot V2 on Raspberry Pi Pico
*
*/
#include "pico/stdlib.h"
#include "iostream"
#include "Servo.h"
#include "Motions.h"
#include "NRF24.h"



[[maybe_unused]] void NRFTest();
float TanjentWithBound(float roll, int i, float bound);


float roll = 0, pitch;

int main()                                                          //Main Function
{
    stdio_init_all();
    sleep_ms(500);
    //multicore_launch_core1(walkCycle); todo reorganize multicore

    //MPUTest();
    //NRFTest();
    defineServo();


    while (true) {
        //walk();
        //test();
        //leg[1].slide(80,40,60);
        //walkCycle();
        //rotationCycle(1);
        //walkCycle();
        //printf("Leg1`s al: %f ,  bet: %f ,  gam: %f  \n", leg[0].lastAng(1), leg[0].lastAng(2), leg[0].lastAng(3));
        //bodyCircularMotion();
        test();

    }
}


void NRFTest() {
    uint8_t addr[6] = "Node1";
    NRF24 nrf(spi0, 17, 18);
    nrf.config();
    nrf.modeTX();
    //TODO Could`t recognize controller`s address.
    char buffer[32];
    while (1) {
        sprintf(buffer, "60");
        buffer[30] = 'R';
        buffer[31] = 'O'; // not a zero.
        nrf.sendMessage(buffer);
        sleep_ms(10);

//        sprintf(buffer, "-60");
//        buffer[30] = 'R';
//        buffer[31] = 'O'; // not a zero.
//        nrf.sendMessage(buffer);
//        sleep_ms(300);

    }
}
//
//void MPUTest(){
//    //todo merge MPU6050 lib with new roll, pitch calculation functions.
//    stdio_init_all();
//
//    i2c_init(i2c1, 100 * 1000);
//    gpio_set_function(15, GPIO_FUNC_I2C);
//    gpio_set_function(14, GPIO_FUNC_I2C);
//    gpio_pull_up(15);
//    gpio_pull_up(14);
//
//    mpu6050_reset();
//
//    int16_t acceleration[3], gyro[3], temp;
//
////    defineServo();
////    defaultPos();
//
//    float offset = 20;
//
//    double past;
//    double integral;
//    double intConst = 0.00001;
//
//    double present;
//    double error;
//    double errorConst = 0.5;
//
//    double future;
//    double derivative;
//    double last;
//    double now;
//    double derConst = 0.001;
//
//
//    absolute_time_t lastTime;
//    long double errorSum;
//    float bound = 0.2;
//    int skip = 0;
//
//    while (1) {
//
//        roll =  atanf( - acceleration[0] / sqrtf(acceleration[1]*acceleration[1] + acceleration[2]*acceleration[2]));
//        pitch = (atanf( - acceleration[1] / sqrtf(acceleration[0]*acceleration[0] + acceleration[2]*acceleration[2])));
//
//        error = ( 0 - roll ) ;
//        present = errorConst * error;
//
//        integral += error;
//        past = intConst * integral;
//
//
//        roll =  atanf( - acceleration[0] / sqrtf(acceleration[1]*acceleration[1] + acceleration[2]*acceleration[2]));
////        rollAcc = atanf( - gyro[0] / sqrtf(acceleration[1]*acceleration[1] + acceleration[2]*acceleration[2]));
//
////        derivative = ( last - now ) / (double) absolute_time_diff_us(lastTime,get_absolute_time());
//        derivative = gyro[0];
//        future = derConst * derivative;
//
//        errorSum = present  ;
//
//
//
//        mpu6050_read_raw(acceleration, gyro, &temp);
//        //printf("Acc. X = %6.2d, Y = %6.2d, Z = %6.2d ", acceleration[0], acceleration[1],acceleration[2]);
//        //printf("Angles. Roll = %6.2f, Pitch = %6.2f\n",roll , pitch);
//        mpu6050_reset();
//
//
//        printf("Error sum: %.3Lf Future: %.3f Present: %.3f Past: %.3f \n", errorSum , future, present, past );
////        for (int i = 0; i <= 3; ++i) {
////            float y = leg[i].lastPos.y,
////                    z = leg[i].lastPos.z;
////            //todo use PID intead of bound;
////            leg[i].toPos( Vector(60, 60, offset + 60 /
////            //tanf( TanjentWithBound( roll, i, 0.5 ) ) )
////            tanf(PI / 3 + (errorSum) * powf(-1 , i)) )
////            );
////        }
//        sleep_ms(150 );
//    }
//
//
//}
////float TanjentWithBound(float roll, int i , float bound){
////    float al = PI / 3 + roll * powf(-1 , i);
////    if(tanf(al) - bound <= tanf(al) && tanf(al) <= tanf(al) + bound )
////        return tanf(al);
////}
//


