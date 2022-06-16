#include "LEGCLASS.h"
#include <cmath>
#include "pico/time.h"

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Leg::toPos(float posX, float posY, float posZ) {           //Inverse kinematic (Needs Upgrade)
    float al, bet, gam, L, L1;
    L1 = sqrtf(posX * posX + posY * posY);
    L =  sqrtf(posZ * posZ + (L1 - J3) * (L1 - J3));

    al =  ((180 * (acosf((J1 * J1 - J2 * J2 - L * L) / (-2 * J2 * L)))) / PI + (180 * (acosf(posZ / L)) / PI));
    gam =  (((180 * (atanf(posX / posY))) / PI + 45));
    bet =  ((180 * acosf((L * L - J1 * J1 - J2 * J2) / (-2 * J1 * J2))) / PI);

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
    float sinus = sinf(freq) < 0 ? sinf(freq) : 0 ;

    float x = - R * cosf(freq);
    float y = 0;
    float z = tmpz + (R*1)*sinus;  //Making a half circle on z axis

    omega = (omega * PI)/180;       //Converting to radian

    toPos( tmpx + x*cosf(omega) - y*sinf(omega),  tmpy + x*sinf(omega) + y*cosf(omega), z);
}
void Leg::slide(float posX, float posY ,float posZ){
    for( float flag = 0; flag <= 2*PI ; flag = flag + PI/4){
        toPos(40,40,60);
        float R=sqrtf((posX-lastPos.x)*(posX-lastPos.x) + (posY-lastPos.y)*(posY-lastPos.y) + (posZ-lastPos.z)*(posZ-lastPos.z))/2;
        float tmpx = lastPos.x , tmpy =  posY, tmpz = lastPos.z, x,y,z;
        //absolute_time_t time = get_absolute_time();
        //while(!(lastPos.x + R*cos(millis()/500) == posX)){
        for(float j = PI ; j > -PI ; j = j - 0.5){
            //tmpx + R - R*cos(j)
            //get_absolute_time()
            //
            x = - R*cosf(j);
            y = 0;
            toPos( tmpx + x*cosf(flag) - y*sinf(flag),  tmpy + x*sinf(flag) + y*cosf(flag),   tmpz);
            sleep_ms(100);
        }
        sleep_ms(1000);
    }
}

