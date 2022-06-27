#ifndef QUADRUPEDV2_VECTOR_H
#define QUADRUPEDV2_VECTOR_H

class Vector{
public:
    float x;
    float y;
    float z;

    Vector()= default;
    explicit Vector(float input){
        x = input;
        y = input;
        z = input;
    }
    Vector(float x , float y ,float z){
        this->x = x;
        this->y = y;
        this->z = z;
    }
};
class Angle{
public:
    float al;
    float bet;
    float gam;

    Angle()= default;;
    explicit Angle(float input){
        al = input;
        bet = input;
        gam = input;
    }
    Angle(float al , float bet ,float gam){
        this->al = al;
        this->bet = bet;
        this->gam = gam;
    }
};
#endif //QUADRUPEDV2_VECTOR_H
