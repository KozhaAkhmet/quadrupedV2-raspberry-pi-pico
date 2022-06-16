/*
* Made by Kozha Akhmet Abdramanov
* Quadruped Robot V2 on Raspberry Pi Pico
*
*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/clocks.h"
#include "hardware/spi.h"
#include "cmath"


#include "NRF24.h"
#include "MYSERVO.h"
#include "MPU6050.h"
#include "LEGCLASS.h"

void defineServo();
void walk();
void defaultPos() ;
void test();
void bodyCircularMotion();
void walkCycle();
void rotationCycle( bool dir );

Leg leg[4];

int main(){                                                         //Main Function
/*
  stdio_init_all();
  printf("Hello, MPU6050! Reading raw data from registers...\n");

  i2c_init(i2c1, 100 * 1000);
  gpio_set_function(15, GPIO_FUNC_I2C);
  gpio_set_function(14, GPIO_FUNC_I2C);
  gpio_pull_up(15);
  gpio_pull_up(14);

  //bi_decl(bi_2pins_with_func(15, 14,GPIO_FUNC_I2C));
  mpu6050_reset();

  int16_t acceleration[3], gyro[3], temp;

  while (1) {
    mpu6050_read_raw(acceleration, gyro, &temp);
    printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1],acceleration[2]);
    printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
    printf("Temp. = %f\n", (temp / 340.0) + 36.53);
    mpu6050_reset();
    sleep_ms(100);
  }*/

    /*uint8_t addr[6] = "Node1";
    NRF24 nrf(spi0, 16, 17);
    nrf.config();
    nrf.modeTX();
    
    char buffer[32];
    while(1){
        sprintf(buffer,"60");
        buffer[30] = 'R';
        buffer[31] = 'O'; // not a zero.
        nrf.sendMessage(buffer);
        sleep_ms(1000);

        sprintf(buffer,"-60");
        buffer[30] = 'R';
        buffer[31] = 'O'; // not a zero.
        nrf.sendMessage(buffer);
        sleep_ms(1000);

    }*/

  defineServo();
  defaultPos();
  while (true)
  {
      //walk();
      //test();
      //leg[1].slide(80,40,60);
      //walkCycle();
      rotationCycle(1);
      //bodyCircularMotion();
  }
}
void defineServo(){
  int i,j;
    leg[0].servo[0].servoPin=20;    leg[0].servo[0].range[0]=550.f;   leg[0].servo[0].range[1]=2400.f;    //Between 0-160 degree
    leg[0].servo[1].servoPin=19;    leg[0].servo[1].range[0]=550.f;   leg[0].servo[1].range[1]=2400.f;
    leg[0].servo[2].servoPin=18;    leg[0].servo[2].range[0]=550.f;   leg[0].servo[2].range[1]=2400.f;

    leg[1].servo[0].servoPin=9;     leg[1].servo[0].range[0]=2400.f;  leg[1].servo[0].range[1]=550.f;
    leg[1].servo[1].servoPin=10;    leg[1].servo[1].range[0]=2400.f;  leg[1].servo[1].range[1]=550.f;
    leg[1].servo[2].servoPin=11;    leg[1].servo[2].range[0]=2400.f;  leg[1].servo[2].range[1]=550.f;    //Between 0-180

    leg[2].servo[0].servoPin=28;    leg[2].servo[0].range[0]=2400.f;  leg[2].servo[0].range[1]=550.f;    //0-180
    leg[2].servo[1].servoPin=22;    leg[2].servo[1].range[0]=2400.f;  leg[2].servo[1].range[1]=550.f;    //0-160
    leg[2].servo[2].servoPin=21;    leg[2].servo[2].range[0]=2400.f;  leg[2].servo[2].range[1]=550.f;    //0-160

    leg[3].servo[0].servoPin=0;     leg[3].servo[0].range[0]=550.f;   leg[3].servo[0].range[1]=2400.f;   //0-160
    leg[3].servo[1].servoPin=1;     leg[3].servo[1].range[0]=550.f;   leg[3].servo[1].range[1]=2400.f;  //0-160
    leg[3].servo[2].servoPin=8;     leg[3].servo[2].range[0]=550.f;   leg[3].servo[2].range[1]=2400.f;  //0-160
    for(i=0; i<4 ; i++)
      for(j=0 ; j<3 ; j++)
        leg[i].servo[j].init();
}
void defaultPos() {                                                 //Default leg positions
  for(int i=0; i<4 ; i++){
    leg[i].toPos(60,60,60);
  }
}
void bodyMove(float posx, float posy, float posz) {                       //Body displacement
  leg[0].toPos(40 + posx, 40 - posy, posz);
  leg[1].toPos(40 - posx, 40 - posy, posz);
  leg[2].toPos(40 + posx, 40 + posy, posz);
  leg[3].toPos(40 - posx, 40 + posy, posz);
}
void walk() {                                                       //Manuel Walking gait (On Procsess..)
  int vel =500;
  leg[3].toPos(40, 20, 40);
  sleep_ms(vel);
  leg[3].toPos(40, 20, 60);
  sleep_ms(vel);
  leg[1].toPos(40, 120, 0);
  sleep_ms(vel);
  leg[1].toPos(40, 120, 60);
  sleep_ms(vel);
  leg[0].toPos(40, 20, 60);
  leg[1].toPos(40, 70, 60);
  leg[2].toPos(40, 120, 60);
  leg[3].toPos(40, 70, 60);
  sleep_ms(vel);
  leg[2].toPos(40, 20, 40);
  sleep_ms(vel);
  leg[2].toPos(40, 20, 60);
  sleep_ms(vel);
  leg[0].toPos(40, 120, 0);
  sleep_ms(vel);
  leg[0].toPos(40, 120, 60);
  sleep_ms(vel);
  leg[3].toPos(40, 20, 40);
  sleep_ms(vel);
  leg[3].toPos(40, 20, 60);
  sleep_ms(vel);
  leg[0].toPos(40, 70, 60);
  leg[1].toPos(40, 20, 60);
  leg[2].toPos(40, 70, 60);
  leg[3].toPos(40, 20, 60);
  sleep_ms(vel);
}
void test(){
  int i,j;
  for( i=0 ; i<4 ; i++)
    for(j=40 ; j<80 ; j++){
    leg[i].toPos(40,40,j);
    sleep_ms(50);
    }
  for( i=0 ; i<4 ; i++)
    for(j=80 ; j>40 ; j--){
    leg[i].toPos(40,40,j);
    sleep_ms(50);
    }
  for( i=0 ; i<4 ; i++)
    for(j=40 ; j<80 ; j++){
    leg[i].toPos(40,40,j);
    sleep_ms(50);
  }
  for(j=80 ; j>40 ; j--){
    leg[0].toPos(40,40,j);
    leg[1].toPos(40,40,j);
    leg[2].toPos(40,40,j);
    leg[3].toPos(40,40,j);
    sleep_ms(50);
  }
  for(double j=0 ; j < (2*PI) ; j=j+0.1){
    bodyMove(20*sin(j),0,60);
    sleep_ms(50);
  }
}
void bodyCircularMotion(){
  for(double j=0 ; j < (2*PI) ; j=j+0.1){
    bodyMove(20*sin(j),20*cos(j),60);
    sleep_ms(50);
  }
}
void walkCycle(){
  double angle = 0;

  for( double  freq ; freq > freq - 2*PI ; freq = freq - 0.1){

  leg[0].stepCycle( 70 ,   angle + 90   , freq     );
  leg[1].stepCycle( 70 , - angle + 90   , freq + PI);
  leg[2].stepCycle( 70 , - angle - 90   , freq + PI);
  leg[3].stepCycle( 70 ,   angle + 270  , freq     );

  sleep_ms(30);
  }
}
void rotationCycle( bool dir ){
  double angle = 0;

  if( dir = 1 )
    angle = -135;
  else  
    angle = 45;

  for( double  freq ; freq > freq - 2*PI ; freq = freq - 0.1){

  leg[0].stepCycle( 70 ,   angle + 90  , freq     );
  leg[1].stepCycle( 70 , - angle       , freq + PI);
  leg[2].stepCycle( 70 , - angle       , freq + PI);
  leg[3].stepCycle( 70 ,   angle + 90  , freq     );

  sleep_ms(40);
  }
}
