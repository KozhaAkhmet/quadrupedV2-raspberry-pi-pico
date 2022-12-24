#include "LegClass.h"

#include <cmath>
#include "pico/time.h"
#include "Mapping.h"
#include "Eigen"

/**
 * Inverse kinematic (Work but requires Update)
 * @param Eigen::Vector3f pos
 */
void Leg::toPos(Eigen::Vector3f pos) {
    double al, bet, gam;
    float L, L1;
    L1 = sqrtf(pos(0) * pos(0) + pos(1) * pos(1));
    L =  sqrtf(pos(2) * pos(2) + (L1 - J3) * (L1 - J3));

    al =   ((180 * (acosf((J1 * J1 - J2 * J2 - L * L) / (-2 * J2 * L)))) / PI   +   (180 * (acosf(pos(2) / L)) / PI));
    gam =  (((180 * (atanf(pos(0) / pos(1)))) / PI + 45));
    bet =  ((180 * acosf((L * L - J1 * J1 - J2 * J2) / (-2 * J1 * J2))) / PI);

    //lastAng(0) = al;
    //lastAng(1) = gam;
    //lastAng(2) = bet;
//
//    lastPos.x = pos(1);
//    lastPos(2) = pos(2);
//    lastPos.z = pos(3);
    //printf("%f   %f    %f   %f    %f\n", al, gam, bet, L1, L);

    servo[0].write(map<float>((float )gam,0,180,  servo[0].range[0],  servo[0].range[1]));
    servo[1].write(map<float>((float )al,0,180,   servo[1].range[0],  servo[1].range[1]));
    servo[2].write(map<float>((float )bet,0,180,  servo[2].range[0],  servo[2].range[1]));

}

/**
 * Forward Kinematics (Work). Applies anges to the Servos.
 * @param ang
 */
void Leg::toAng(Eigen::Vector3f ang){
    servo[0].write(map<float>(ang(0),0,180,  servo[0].range[0],  servo[0].range[1]));
    servo[1].write(map<float>(ang(1),0,180,   servo[1].range[0],  servo[1].range[1]));
    servo[2].write(map<float>(ang(2),0,180,  servo[2].range[0],  servo[2].range[1]));
}
/**
 * Cycle Function which do circular motion on defined(tmp) position.   (Work)
 * @param dis Step distance in mm.
 * @param dir Direction of the cycle.
 * @param angle Current angle of the cycle.
 */
bool Leg::stepCycle(float direction, float stepLength, float angle){
    // Checking if angle is reached ground or simply done its step by doing 180 degree step.
    if(angle >= PI)
        return true;

    //direction is in radian for better calculation
    //tmp is initial reference coordinate or simply translation of function

    Eigen::Vector3f translation(60,60,60);

    float radius = stepLength/2;

    Eigen::Vector3f functionVec( -radius * cosf(-angle),
                                 0,
                                 radius * sinf(-angle)
                                );

    //uncomment if you use direction as angle. This converts direction to radian.
    //direction = (direction * PI)/180;

    Eigen::Matrix3f rotationMatrix;
    rotationMatrix<< cosf(direction), sinf(direction),
                     0,
                    -sinf(direction), cosf(direction
                     );

    toPos(  translation + rotationMatrix * functionVec);
    return false;
}

/**
 * Does slide to input position. (Work but requires update)
 * @param Eigen::Vector3f targetPos
 */
void Leg::slide(Eigen::Vector3f targetPos){
    for( float flag = 0; flag <= 2*PI ; flag = (float) flag + PI/4){
        toPos(Eigen::Vector3f (40,60,60));
        float R=sqrtf((targetPos(0)-lastPos(1))*(targetPos(0)-lastPos(1)) + (targetPos(1)-lastPos(2))*(targetPos(1)-lastPos(2)) + (targetPos(2)-lastPos(3))*(targetPos(2)-lastPos(3)))/2;
        float tmpx = lastPos(0) , tmpy =  targetPos(1), tmpz = lastPos(2), x,y,z;
        //absolute_time_t time = get_absolute_time();
        //while(!(lastPos(1) + R*cos(millis()/500) == pos(1))){
        for(float j = PI ; j > -PI ; j = (float) j - 0.5){
            //tmpx + R - R*cos(j)
            //get_absolute_time()
            //
            x = - R*cosf(j);
            y = 0;
            toPos( Eigen::Vector3f (tmpx + x*cosf(flag) - y*sinf(flag),  tmpy + x*sinf(flag) + y*cosf(flag),   tmpz));
            sleep_ms(100);
        }
        sleep_ms(1000);
    }
}
// Old stepCycle with eigen library
/*
bool Leg::stepCycle(float stepLength, float direction, float freq){
    //direction is in radian for better calculation
    //tmp is initial reference coordinate
    Vector Eigen::Vector3f tmp(60,60,60);

    float radius = stepLength/2;
    float sinus = sinf(freq) < 0 ? sinf(freq) : 0;

    float x = -radius * cosf(freq);
    float y = 0;
    float z = tmp.z + (radius*i) * sinus;

    //uncomment if you use direction as angle. This converts direction to radian.
    //direction = (direction * PI)/180;

    toPos(Eigen:: Vector3f(x + x*cosf(direction) - y*sinf(direction),
                           tmp(1) + x*sinf(direction) + y*cosf(direction),
                           z
                           ));
    return true;
}
*/




