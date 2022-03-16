/*
* Made by Kozha Akhmet Abdramanov
* Quadruped Robot V2 on Raspberry Pi Pico
*
*/
#include <iostream>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/clocks.h"
#include "hardware/spi.h"
#include <math.h>

#include "nRF24L01/NRF24.h"
#include "MyServo/MYSERVO.h"
#include "MPU6050/MPU6050.h"

#define PI 3.14

#define claw      83                                          //Defining Leg`s length in mm
#define connecter 55
#define initial   33    

void defineServo();
void walk();
void defaultPos() ;
void test();
void bodyCircularMotion();
void walkCycle();
void rotationCycle( bool dir );




long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

struct Ang {
    double al;
    double bet;
    double gam;
};
struct Pos {
    double x;
    double y;
    double z;
};
class Leg {                                                        //Creating Leg class and its ingredients
    public:
        Ang lastAng;
        Pos lastPos;
        Servo servo[3];
        void toPos(double posX, double posY, double posZ);
        void toAng(double al, double bet , double gam );
        void stepCycle(double dis, double omega, double freq);
        void slide(double posX, double posY ,double posZ);
};
void Leg::toPos(double posX, double posY, double posZ) {           //Inverse kinematic (Needs Upgrade)
    double al, bet, gam, L, L1 = sqrt(posX * posX + posY * posY);

    L = sqrt(posZ * posZ + (L1 - initial) * (L1 - initial));

    al = (180 * (acos((claw * claw - connecter * connecter - L * L) / (-2 * connecter * L)))) / PI + (180 * (acos(posZ / L)) / PI);
    gam = (((180 * (atan(posX / posY))) / PI + 45));
    bet = (180 * acos((L * L - claw * claw - connecter * connecter) / (-2 * claw * connecter))) / PI;
    
    lastAng.al = al;
    lastAng.gam = gam;
    lastAng.bet = bet;
    
    lastPos.x = posX;
    lastPos.y = posY;
    lastPos.z = posZ;

    servo[0].write(map(gam,0,180,  servo[0].range[0],  servo[0].range[1]));
    servo[1].write(map(al,0,180,   servo[1].range[0],  servo[1].range[1]));
    servo[2].write(map(bet,0,180,  servo[2].range[0],  servo[2].range[1]));
}
void Leg::toAng(double al, double bet , double gam ){    
  servo[0].write(map(gam,0,180,  servo[0].range[0],  servo[0].range[1]));
  servo[1].write(map(al,0,180,   servo[1].range[0],  servo[1].range[1]));
  servo[2].write(map(bet,0,180,  servo[2].range[0],  servo[2].range[1]));
}
void Leg::stepCycle(double dis, double omega, double freq){              //Function for stepCycles 
  //get_absolute_time()
  double R = dis/2, tmpx = 60, tmpy = 60, tmpz = 60;
  double sinus= sin(freq) < 0 ? sin(freq) : 0 ;  

  double x = - R*cos(freq); 
  double y = 0; 
  double z = tmpz + (R*1)*sinus;  //Making a half circle on z axis

  omega = (omega * PI)/180;       //Converting to radian

  toPos( tmpx + x*cos(omega) - y*sin(omega),  tmpy + x*sin(omega) + y*cos(omega), z);
}
void Leg::slide(double posX, double posY ,double posZ){
  for( double flag = 0; flag <= 2*PI ; flag = flag + PI/4){
  toPos(40,40,60);
  double R=sqrt((posX-lastPos.x)*(posX-lastPos.x) + (posY-lastPos.y)*(posY-lastPos.y) + (posZ-lastPos.z)*(posZ-lastPos.z))/2;
  double tmpx = lastPos.x , tmpy =  posY, tmpz = lastPos.z, x,y,z;
  //absolute_time_t time = get_absolute_time();
  //while(!(lastPos.x + R*cos(millis()/500) == posX)){
  for(double j = PI ; j > -PI ; j = j - 0.5){
    //tmpx + R - R*cos(j)
    //get_absolute_time()
    //
    x = - R*cos(j);
    y = 0;
    toPos( tmpx + x*cos(flag) - y*sin(flag),  tmpy + x*sin(flag) + y*cos(flag),   tmpz);
    sleep_ms(100);
  }
  sleep_ms(1000);
  }
}
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
    leg[0].servo[0].servoPin=20;    leg[0].servo[0].range[0]=550;   leg[0].servo[0].range[1]=2400;    //Between 0-160 degree
    leg[0].servo[1].servoPin=19;    leg[0].servo[1].range[0]=550;   leg[0].servo[1].range[1]=2400;
    leg[0].servo[2].servoPin=18;    leg[0].servo[2].range[0]=550;   leg[0].servo[2].range[1]=2400;

    leg[1].servo[0].servoPin=9;     leg[1].servo[0].range[0]=2400;  leg[1].servo[0].range[1]=550;
    leg[1].servo[1].servoPin=10;    leg[1].servo[1].range[0]=2400;  leg[1].servo[1].range[1]=550;
    leg[1].servo[2].servoPin=11;    leg[1].servo[2].range[0]=2400;  leg[1].servo[2].range[1]=550;    //Between 0-180

    leg[2].servo[0].servoPin=28;    leg[2].servo[0].range[0]=2400;  leg[2].servo[0].range[1]=550;    //0-180
    leg[2].servo[1].servoPin=22;    leg[2].servo[1].range[0]=2400;  leg[2].servo[1].range[1]=550;    //0-160
    leg[2].servo[2].servoPin=21;    leg[2].servo[2].range[0]=2400;  leg[2].servo[2].range[1]=550;    //0-160

    leg[3].servo[0].servoPin=0;     leg[3].servo[0].range[0]=550;   leg[3].servo[0].range[1]=2400;   //0-160
    leg[3].servo[1].servoPin=1;     leg[3].servo[1].range[0]=550;   leg[3].servo[1].range[1]=2400;  //0-160
    leg[3].servo[2].servoPin=8;     leg[3].servo[2].range[0]=550;   leg[3].servo[2].range[1]=2400;  //0-160
    for(i=0; i<4 ; i++)
      for(j=0 ; j<3 ; j++)
        leg[i].servo[j].init();
}
void defaultPos() {                                                 //Default leg positions
  for(int i=0; i<4 ; i++){
    leg[i].toPos(60,60,60);
  }
}
void bodyMove(int posx, int posy, int posz) {                       //Body displacement 
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
