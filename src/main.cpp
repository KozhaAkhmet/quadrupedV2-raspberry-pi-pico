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


#include "MPU6050.h"
#include "Motions.h"
#include "PID.h"

void MPUTest();
// double Bounds(double input, double min , double max);
void Balance();
MPU6050 mpu;
float median1(float newVal);
float median2(float newVal);
float median3(float newVal);
float expRunningAverageAdaptive(float newVal);
float runMiddleArifm(float newVal);
#define NUM_READ 5

int main() {                                                         //Main Function
    stdio_init_all();
    //todo kalman filter for MPU6050
    mpu.initMPU(14,15);

    int aver_times = 10;
    int16_t acceleration[3]={0,0,0}, gyro[3]={0,0,0}, temp=0;

    float filtered_accel[3]= {}, filtered_gyro[3]={};

    while (true) {
        mpu.readRaw(acceleration, gyro, &temp);

        

        // filtered_gyro[0] /= aver_times;
        // filtered_gyro[1] /= aver_times;
        // filtered_gyro[2] /= aver_times;

        filtered_accel[0] = median1(filtered_accel[0]);
        filtered_accel[1] = median2(filtered_accel[1]);
        filtered_accel[2] = median3(filtered_accel[2]);

        // filtered_accel[0] = expRunningAverageAdaptive(filtered_accel[0]);
        for(int i=0 ; i<10; i++){
            filtered_accel[0] += acceleration[0];
            filtered_accel[1] += acceleration[1];
            filtered_accel[2] += acceleration[2];

            filtered_gyro[0] += gyro[0];
            filtered_gyro[1] += gyro[1];
            filtered_gyro[2] += gyro[2];
        }

        filtered_accel[0] /= aver_times;
        filtered_accel[1] /= aver_times;
        filtered_accel[2] /= aver_times;

        filtered_accel[0] = expRunningAverageAdaptive(filtered_accel[0]);

        
        // if(true)
        // printf("x:%d, y:%d, z:%d\n", acceleration[0], acceleration[1], acceleration[2]);
        // else
        // printf("x:%d, y:%d, z:%d \n",gyro[0],gyro[1],gyro[2]);


        if(true)
        printf("x:%f, y:%f, z:%f\n", filtered_accel[0], filtered_accel[1], filtered_accel[2]);
        else
        printf("x:%f, y:%f, z:%f\n",filtered_gyro[0],filtered_gyro[1],filtered_gyro[2]);

        sleep_ms(100);
        // mpu.reset();
    }
}
float median1(float newVal) {
  static float buf[3];
  static int count = 0;
  buf[count] = newVal;
  if (++count > 2) count = 0;

  float a = buf[0];
  float b = buf[1];
  float c = buf[2];

  float middle;
  if ((a <= b) && (a <= c)) {
    middle = (b <= c) ? b : c;
  } else {
    if ((b <= a) && (b <= c)) {
      middle = (a <= c) ? a : c;
    } else {
      middle = (a <= b) ? a : b;
    }
  }
  return middle;
}
float median2(float newVal) {
  static float buf[3];
  static int count = 0;
  buf[count] = newVal;
  if (++count > 2) count = 0;

  float a = buf[0];
  float b = buf[1];
  float c = buf[2];

  float middle;
  if ((a <= b) && (a <= c)) {
    middle = (b <= c) ? b : c;
  } else {
    if ((b <= a) && (b <= c)) {
      middle = (a <= c) ? a : c;
    } else {
      middle = (a <= b) ? a : b;
    }
  }
  return middle;
}
float median3(float newVal) {
  static float buf[3];
  static int count = 0;
  buf[count] = newVal;
  if (++count > 2) count = 0;

  float a = buf[0];
  float b = buf[1];
  float c = buf[2];

  float middle;
  if ((a <= b) && (a <= c)) {
    middle = (b <= c) ? b : c;
  } else {
    if ((b <= a) && (b <= c)) {
      middle = (a <= c) ? a : c;
    } else {
      middle = (a <= b) ? a : b;
    }
  }
  return middle;
}
float expRunningAverageAdaptive(float newVal) {
  static float filVal = 0;
  filVal += (newVal - filVal) * 0.3;
  return filVal;
}
float runMiddleArifm(float newVal) {  // принимает новое значение
  static int idx = 0;                // индекс
  static float valArray[NUM_READ];    // массив
  valArray[idx] = newVal;             // пишем каждый раз в новую ячейку
  if (++idx >= NUM_READ) idx = 0;     // перезаписывая самое старое значение
  float average = 0;                  // обнуляем среднее
  for (int i = 0; i < NUM_READ; i++) {
    average += valArray[i];           // суммируем
  }
  return (float)average / NUM_READ;   // возвращаем
}
/*
void MPUTest(){
    stdio_init_all();
    //todo kalman filter for MPU6050
    mpu.initMPU(15,14);

    int16_t acceleration[3]={0,0,0}, gyro[3]={0,0,0}, temp=0;

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

    double  delT = 0.04, al = 0.96 ,
            gyroAngleX = 0 , gyroAngleY = 0 , gyroAngleZ = 0;

    absolute_time_t t = get_absolute_time() ;
    double AngleX ,AngleY ,AngleZ ,x_accel;



    while (1) {
        mpu.readRaw(acceleration, gyro, &temp);
        */
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
        // mpu.calculateAverageAcceleration();
        // mpu.calculateAverageGyro();

        // pid.setP(0.05      , -0.001 , 0.001);
        // pid.setI(0.0001    , -0.01  , 0.001);
        // pid.setD(0.000003 , -0.1   , 0.1);

        // pid.calculatePID(mpu.getRoll(), gyro[0]);
        /*
        printf("accel %f  %f  %f  gyro %f  %f  %f  temp %f \n", acceleration[0], acceleration[1], acceleration[2],gyro[0],gyro[1],gyro[2], temp);
        sleep_ms(100);
        mpu.reset();

    }
}
*/

// void Balance(){

//     float offset = 20;
//     float z;

//     defineServo();

//     for (int i = 0; i <= 3; ++i) {
//         z = offset + 60 / tanf(PI / 3 + ( mpu.getRoll() + pid.getPID()) * powf(-1 , (float) i));
//         leg[i].toPos( Vector(60, 60, z ));
//     }
//     sleep_ms(100 );
// }

// double Bounds(double *input, const double min , const double max){
//     if (*input >= max) *input = max;
//     if (*input <= min) *input = min;
//     return *input;
// }

