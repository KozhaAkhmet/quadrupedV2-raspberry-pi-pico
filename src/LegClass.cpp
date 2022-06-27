#include "LegClass.h"
#include <cmath>
#include "pico/time.h"
/**
 * Converts one range to another. Similar to Arduno map function.
 * @param x Variable which range is converting
 * @param in_min Variable min bound.
 * @param in_max  Variable max bound.
 * @param out_min Targeted min bound.
 * @param out_max Targeted max bound.
 * @return Returns variable in targeted bound.
 */
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Inverse kinematic (Work but requires Update)
 * @param pos
 */
void Leg::toPos(Vector pos) {
    float al, bet, gam, L, L1;
    L1 = sqrtf(pos.x * pos.x + pos.y * pos.y);
    L =  sqrtf(pos.z * pos.z + (L1 - J3) * (L1 - J3));

    al =  (float) ((180 * (acosf((J1 * J1 - J2 * J2 - L * L) / (-2 * J2 * L)))) / PI   +   (180 * (acosf(pos.z / L)) / PI));
    gam = (float) (((180 * (atanf(pos.x / pos.y))) / PI + 45));
    bet = (float) ((180 * acosf((L * L - J1 * J1 - J2 * J2) / (-2 * J1 * J2))) / PI);

    lastAng.al = al;
    lastAng.gam = gam;
    lastAng.bet = bet;

    lastPos.x = pos.x;
    lastPos.y = pos.y;
    lastPos.z = pos.z;

    servo[0].write(map(gam,0,180,  servo[0].range[0],  servo[0].range[1]));
    servo[1].write(map(al,0,180,   servo[1].range[0],  servo[1].range[1]));
    servo[2].write(map(bet,0,180,  servo[2].range[0],  servo[2].range[1]));
}

/**
 * Forward Kinematics (Work). Applies anges to the Servos.
 * @param ang
 */
void Leg::toAng(Angle ang){
    servo[0].write(map(ang.gam,0,180,  servo[0].range[0],  servo[0].range[1]));
    servo[1].write(map(ang.al,0,180,   servo[1].range[0],  servo[1].range[1]));
    servo[2].write(map(ang.bet,0,180,  servo[2].range[0],  servo[2].range[1]));
}
/**
 * Cycle Function which do circular motion on defined(tmp) position.   (Work)
 * @param dis Step distance in mm.
 * @param dir Direction if the Cycle in angles.
 * @param freq Cycle update rate.
 */
void Leg::stepCycle(float dis, float dir, float freq){
    //get_absolute_time()
    Vector tmp(60);
    float R = dis/2;
    float sinus = sinf(freq) < 0 ? sinf(freq) : 0 ;

    float x = - R * cosf(freq);
    float y = 0;
    float z = tmp.z + (R*1)*sinus;  //Making a half circle on z axis

    dir = (float) (dir * PI)/180;       //Converting to radian

    toPos( Vector(x + x*cosf(dir) - y*sinf(dir),  tmp.y + x*sinf(dir) + y*cosf(dir), z));
}

/**
 * Does slide to input position. (Work but requires update)
 * @param pos
 */
void Leg::slide(Vector pos){
    for( float flag = 0; flag <= 2*PI ; flag = (float) flag + PI/4){
        toPos(Vector(40,60,60));
        float R=sqrtf((pos.x-lastPos.x)*(pos.x-lastPos.x) + (pos.y-lastPos.y)*(pos.y-lastPos.y) + (pos.z-lastPos.z)*(pos.z-lastPos.z))/2;
        float tmpx = lastPos.x , tmpy =  pos.y, tmpz = lastPos.z, x,y,z;
        //absolute_time_t time = get_absolute_time();
        //while(!(lastPos.x + R*cos(millis()/500) == pos.x)){
        for(float j = PI ; j > -PI ; j = (float) j - 0.5){
            //tmpx + R - R*cos(j)
            //get_absolute_time()
            //
            x = - R*cosf(j);
            y = 0;
            toPos( Vector(tmpx + x*cosf(flag) - y*sinf(flag),  tmpy + x*sinf(flag) + y*cosf(flag),   tmpz));
            sleep_ms(100);
        }
        sleep_ms(1000);
    }
}

