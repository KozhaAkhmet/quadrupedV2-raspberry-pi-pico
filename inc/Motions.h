#ifndef QUADRUPEDV2_MOTIONS_H
#define QUADRUPEDV2_MOTIONS_H

#include "LegClass.h"
#include "pico/time.h"
#include "cmath"

Leg leg[4];

//todo write descriptions to all motions

void defineServo();
void walk();
void defaultPos() ;
void test();
void bodyCircularMotion();
void walkCycle();
void rotationCycle( bool dir );
void angleToLegs();

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
        leg[i].toPos(Vector(60,60,60));
    }
}

void bodyMove(Vector pos) {                       //Body displacement
    leg[0].toPos(Vector( + pos.x, 40 - pos.y, pos.z));
    leg[1].toPos(Vector( - pos.x, 40 - pos.y, pos.z));
    leg[2].toPos(Vector( + pos.x, 40 + pos.y, pos.z));
    leg[3].toPos(Vector( - pos.x, 40 + pos.y, pos.z));
}
/*
void walk() {                                                       //Manuel Walking gait (On Procsess..)
    int vel =500;
    leg[3].toPos(Vector(40, 20, 40));
    sleep_ms(vel);
    leg[3].toPos(Vector(40, 20, 60));
    sleep_ms(vel);
    leg[1].toPos(Vector(40, 120, 0));
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
*/
void test(){
    int i,j;
    for( i=0 ; i<4 ; i++)
        for(j=40 ; j<80 ; j++){
            leg[i].toPos(Vector(40,40,j));
            // sleep_ms(50);
        }
    for( i=0 ; i<4 ; i++)
        for(j=80 ; j>40 ; j--){
            leg[i].toPos(Vector(40,40,j));
            //sleep_ms(50);
        }
    for( i=0 ; i<4 ; i++)
        for(j=40 ; j<80 ; j++){
            leg[i].toPos(Vector(40,40,j));
            //sleep_ms(50);
        }
    for(j=80 ; j>40 ; j--){
        leg[0].toPos(Vector(40,40,j));
        leg[1].toPos(Vector(40,40,j));
        leg[2].toPos(Vector(40,40,j));
        leg[3].toPos(Vector(40,40,j));
        // sleep_ms(50);
    }
    for(float j=0 ; j < (2*PI) ; j=j+0.1){
        bodyMove(Vector(20*sinf(j),0,60));
        //sleep_ms(50);
    }
}

void bodyCircularMotion(){
    for(double j=0 ; j < (2*PI) ; j=j+0.1){
        bodyMove(Vector(20*sin(j),20*cos(j),60));
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

        sleep_ms(15);
    }
}


/**
 * Rotates in place. If dir 1 turns right else left.
 * @param dir
 */
void rotationInPlace( bool dir ){
    double angle = 0;

    if( dir == 1 )
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

#endif //QUADRUPEDV2_MOTIONS_H
