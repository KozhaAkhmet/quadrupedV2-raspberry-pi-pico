/*
* made by Kozha Akhmet Abdramanov
* Quadruped Robot V2 on Raspberry Pi Pico
*/

#include "pico/stdlib.h"
#include <iostream>
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <math.h>

#define PI 3.14

#define claw      83                                          //Defining Leg`s length in mm
#define connecter 55
#define initial   33    

void defineServo();
void walk();
void defaultPos() ;
void test();

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

struct Servo{                                                  //Defining Servo class
    public:
    int servoPin;
    int range[2];
    void init();
    void write(float pusle); 
};
void Servo::init(){
  gpio_set_function(servoPin, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(servoPin);

  pwm_set_clkdiv(slice_num, 64.f);
  pwm_set_wrap(slice_num, 39062.f);

  pwm_set_enabled(slice_num,true);
}
void Servo::write(float pulse){                               //Custom Servo control function
  pwm_set_gpio_level(servoPin, (pulse/20000.f)*39062.f);
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
class Leg {                                                        //Making Leg class and its ingredients
    public:
        Ang ang;
        Pos pos;
        Servo servo[3];
        void toPos(double posX, double posY, double posZ);
        void toAng(double al, double bet , double gam );
        void Step (double posX, double posY, double posZ);
};
void Leg::toPos(double posX, double posY, double posZ) {           //Inverse kinematic
    double al, bet, gam, L, L1 = sqrt(posX * posX + posY * posY);
    L = sqrt(posZ * posZ + (L1 - initial) * (L1 - initial));
    al = (180 * (acos((claw * claw - connecter * connecter - L * L) / (-2 * connecter * L)))) / PI +
         (180 * (acos(posZ / L)) / PI);
    gam = (((180 * (atan(posX / posY))) / PI + 45));
    bet = (180 * acos((L * L - claw * claw - connecter * connecter) / (-2 * claw * connecter))) / PI;
    ang.al = al;
    ang.gam = gam;
    ang.bet = bet;
    pos.x = posX;
    pos.y = posY;
    pos.z = posZ;
    servo[0].write(map(gam,0,180,  servo[0].range[0],  servo[0].range[1]));
    servo[1].write(map(al,0,180,   servo[1].range[0],  servo[1].range[1]));
    servo[2].write(map(bet,0,180,  servo[2].range[0],  servo[2].range[1]));
}
void Leg::toAng(double al, double bet , double gam ){    
  servo[0].write(map(gam,0,180,  servo[0].range[0],  servo[0].range[1]));
  servo[1].write(map(al,0,180,   servo[1].range[0],  servo[1].range[1]));
  servo[2].write(map(bet,0,180,  servo[2].range[0],  servo[2].range[1]));
}
void Leg::Step(double posX, double posY, double posZ){
  /*double R=sqrt((posX-pos.x)*(posX-pos.x) + (posY-pos.y)*(posY-pos.y) + (posZ-pos.z)*(posZ-pos.z))/2;
  double tmpx=pos.x,tmpy=pos.y,tmpz=pos.z, sinus;
 0/* while(!(pos.x + R*cos(millis()/500) == posX)){
  sinus= sin(millis()/500) > 0 ? sin(millis()/500) : 0 ;
  toPos(tmpx + R - R*cos(millis()/500), tmpy, tmpz + R*sinus);
  }*/
}
Leg leg[4];
int main(){
    
    defineServo();
    defaultPos();
    while (true)
    {
        walk();
        //test();
    }
}
/*void defineServo(){

    leg[0].servo[0].servoPin=22;    leg[0].servo[0].range[0]=400;   leg[0].servo[0].range[1]=2400; 
    leg[0].servo[1].servoPin=21;    leg[0].servo[1].range[0]=400;   leg[0].servo[1].range[1]=2400;
    leg[0].servo[2].servoPin=20;    leg[0].servo[2].range[0]=400;   leg[0].servo[2].range[1]=2400;

    leg[1].servo[0].servoPin=9;     leg[1].servo[0].range[0]=2400;  leg[1].servo[0].range[1]=400;
    leg[1].servo[1].servoPin=10;    leg[1].servo[1].range[0]=2400;  leg[1].servo[1].range[1]=400;
    leg[1].servo[2].servoPin=11;    leg[1].servo[2].range[0]=2400;  leg[1].servo[2].range[1]=400;

    leg[2].servo[0].servoPin=28;    leg[2].servo[0].range[0]=2400;  leg[2].servo[0].range[1]=400;
    leg[2].servo[1].servoPin=27;    leg[2].servo[1].range[0]=2400;  leg[2].servo[1].range[1]=400;
    leg[2].servo[2].servoPin=26;    leg[2].servo[2].range[0]=2400;  leg[2].servo[2].range[1]=400;

    leg[3].servo[0].servoPin=3;     leg[3].servo[0].range[0]=400;   leg[3].servo[0].range[1]=2400;
    leg[3].servo[1].servoPin=5;     leg[3].servo[1].range[0]=400;   leg[3].servo[1].range[1]=2400;
    leg[3].servo[2].servoPin=8;     leg[3].servo[2].range[0]=400;   leg[3].servo[2].range[1]=2400;
}*/
void defineServo(){
  int i,j;
    leg[0].servo[0].servoPin=20;    leg[0].servo[0].range[0]=550;  leg[0].servo[0].range[1]=2400;    //Between 0-160 degree
    leg[0].servo[1].servoPin=19;    leg[0].servo[1].range[0]=550;   leg[0].servo[1].range[1]=2400;
    leg[0].servo[2].servoPin=18;    leg[0].servo[2].range[0]=550;   leg[0].servo[2].range[1]=2400;

    leg[1].servo[0].servoPin=9;     leg[1].servo[0].range[0]=2400;  leg[1].servo[0].range[1]=550;
    leg[1].servo[1].servoPin=10;    leg[1].servo[1].range[0]=2400;  leg[1].servo[1].range[1]=550;
    leg[1].servo[2].servoPin=11;    leg[1].servo[2].range[0]=2400;  leg[1].servo[2].range[1]=550;    //Between 0-180

    leg[2].servo[0].servoPin=28;    leg[2].servo[0].range[0]=2400;  leg[2].servo[0].range[1]=550;    //0-180
    leg[2].servo[1].servoPin=22;    leg[2].servo[1].range[0]=2400;  leg[2].servo[1].range[1]=550;    //0-170
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
void walk() {                                                       //Manuel Walking gait
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
  for(j=40 ; j<60 ; j++){
    leg[0].toPos(40,j,40);
    leg[1].toPos(40,-j,40);
    leg[2].toPos(40,-j,40);
    leg[3].toPos(40,j,40);
    sleep_ms(100);
  }
  for(j=80 ; j>60 ; j--){
    leg[0].toPos(40,j,40);
    leg[1].toPos(40,-j,40);
    leg[2].toPos(40,-j,40);
    leg[3].toPos(40,j,40);
    sleep_ms(100);
  }
}
