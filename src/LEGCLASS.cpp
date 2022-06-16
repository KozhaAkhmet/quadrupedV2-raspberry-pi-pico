#include "LEGCLASS.h"
#include <cmath>
#include "pico/time.h"

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Leg::toPos(float posX, float posY, float posZ) {           //Inverse kinematic (Needs Upgrade)
    float al, bet, gam, L, L1;
    L1 = (float) sqrt(posX * posX + posY * posY);
    L = (float) sqrt(posZ * posZ + (L1 - j3) * (L1 - j3));

    al = (float) (180 * (acos((j1 * j1 - j2 * j2 - L * L) / (-2 * j2 * L)))) / PI + (180 * (acos(posZ / L)) / PI);
    gam = (float) (((180 * (atan(posX / posY))) / PI + 45));
    bet = (float) (180 * acos((L * L - j1 * j1 - j2 * j2) / (-2 * j1 * j2))) / PI;

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
void Leg::toAng(float al, float bet , float gam ){
    servo[0].write(map(gam,0,180,  servo[0].range[0],  servo[0].range[1]));
    servo[1].write(map(al,0,180,   servo[1].range[0],  servo[1].range[1]));
    servo[2].write(map(bet,0,180,  servo[2].range[0],  servo[2].range[1]));
}
void Leg::stepCycle(float dis, float omega, float freq){              //Function for stepCycles 
    //get_absolute_time()
    float R = dis/2, tmpx = 60, tmpy = 60, tmpz = 60;
    float sinus= sin(freq) < 0 ? sin(freq) : 0 ;

    float x = - R*cos(freq);
    float y = 0;
    float z = tmpz + (R*1)*sinus;  //Making a half circle on z axis

    omega = (omega * PI)/180;       //Converting to radian

    toPos( tmpx + x*cos(omega) - y*sin(omega),  tmpy + x*sin(omega) + y*cos(omega), z);
}
void Leg::slide(float posX, float posY ,float posZ){
    for( float flag = 0; flag <= 2*PI ; flag = flag + PI/4){
        toPos(40,40,60);
        float R=sqrt((posX-lastPos.x)*(posX-lastPos.x) + (posY-lastPos.y)*(posY-lastPos.y) + (posZ-lastPos.z)*(posZ-lastPos.z))/2;
        float tmpx = lastPos.x , tmpy =  posY, tmpz = lastPos.z, x,y,z;
        //absolute_time_t time = get_absolute_time();
        //while(!(lastPos.x + R*cos(millis()/500) == posX)){
        for(float j = PI ; j > -PI ; j = j - 0.5){
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

