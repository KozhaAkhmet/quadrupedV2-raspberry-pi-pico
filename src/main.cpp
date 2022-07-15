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
#include "PID.h"

void MPUTest();

[[maybe_unused]] void NRFTest();
double Bounds(double input, double min , double max);
void Balance();



void core1(){
    MPUTest();
};

int main() {                                                         //Main Function

    multicore_launch_core1(core1);

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
MPU6050 mpu;
PID pid;
void MPUTest(){
    //todo kalman filter for MPU6050
    mpu.initMPU();

    int16_t acceleration[3], gyro[3], temp;

    double past;
    double integral = 0;
    double intConst = 0.0001;

    double present;
    double error;
    double errorConst = 0.05;

    double future;
    double derivative;
    double last;
    double now;
    double derConst = 0.000003;


    absolute_time_t lastTime;
    long double errorSum;
    int skip = 0;

    double t = get_absolute_time() , delT = 0.04, al = 0.96 ,
            gyroAngleX = 0 , gyroAngleY = 0 , gyroAngleZ = 0;


    double AngleX ,AngleY ,AngleZ ,x_accel;



    while (1) {
        mpu.readRaw(acceleration, gyro, &temp);
/*
        gyroAngleX = acceleration[0] /131 * delT + gyroAngleX;
        gyroAngleY = gyro[1] /131 * delT + gyroAngleY;
        gyroAngleZ = gyro[2] /131 * delT + gyroAngleZ;

        AngleX = al * gyroAngleX + (1 - al) * acceleration[0];
        AngleY = al * gyroAngleY + (1 - al) * acceleration[1];
        AngleZ = gyroAngleZ;
        roll =  atanf( - acceleration[0] / sqrtf(acceleration[1]*acceleration[1] + acceleration[2]*acceleration[2]));
        pitch = (atanf( - acceleration[1] / sqrtf(acceleration[0]*acceleration[0] + acceleration[2]*acceleration[2]))) / 180 * PI;

        roll = al * gyroAngleX + (1 - al) * pitch;

        filtered angle
*/
/*
        mpu.calculateAverageAcceleration();
        mpu.calculateAverageGyro();

        error = ( - mpu.getRoll() ) ;
        present =  Bounds(errorConst * error , -0.001, 0.001);

        integral += error;
        past = Bounds(intConst * integral , -0.01, 0.01);

        derivative = gyro[0];
        future = Bounds( derConst * derivative , -0.1, 0.1);

        errorSum =  future;

        printf("sum: %.3f Future: %.3f Present: %.3f Past: %.3f \n", mpu.getRoll() , future, present, past );

        mpu.reset();
        */
        mpu.calculateAverageAcceleration();
        mpu.calculateAverageGyro();

        pid.setP(0.05      , -0.001 , 0.001);
        pid.setI(0.0001    , -0.01  , 0.001);
        pid.setD(0.000003 , -0.1   , 0.1);

        pid.calculatePID(mpu.getRoll(), gyro[0]);

        mpu.reset();

    }
}

void Balance(){

    float offset = 20;
    float z;

    defineServo();

    for (int i = 0; i <= 3; ++i) {
        z = offset + 60 / tanf(PI / 3 + ( mpu.getRoll() + pid.getPID()) * powf(-1 , (float) i));
        leg[i].toPos( Vector(60, 60, z ));
    }
    sleep_ms(100 );
}

double Bounds(double *input, const double min , const double max){
    if (*input >= max) *input = max;
    if (*input <= min) *input = min;
    return *input;
}

