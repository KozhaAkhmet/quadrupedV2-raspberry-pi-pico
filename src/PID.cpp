#include "PID.h"
void PID::setP(double Ke, double lowerBound, double upperBound) {
    this->Ke = Ke;
    pBounds[0] = lowerBound;
    pBounds[1] = upperBound;
}
void PID::setI(double Ki, double lowerBound, double upperBound) {
    this->Ki = Ki;
    iBounds[0] = lowerBound;
    iBounds[1] = upperBound;
}
void PID::setD(double Kd, double lowerBound, double upperBound) {
    this->Kd = Kd;
    dBounds[0] = lowerBound;
    dBounds[1] = upperBound;
}

void PID::calculatePID(double value, double der) {
    error = ( - value ) ;
    present =  setBounds(Ke * error , pBounds[0], pBounds[1]);

    integral += error;
    past = setBounds(Ki * integral , iBounds[0], iBounds[1]);

    derivative = der;
    future = setBounds( Kd * derivative , dBounds[0], dBounds[1]);
}
void PID::calculatePID(double value, double targetValue, double der) {
    error = ( targetValue - value ) ;
    present =  setBounds(Ke * error , pBounds[0], pBounds[1]);

    integral += error;
    past = setBounds(Ki * integral , iBounds[0], iBounds[1]);

    derivative = der;
    future = setBounds( Kd * derivative , dBounds[0], dBounds[1]);
}

double PID::getP(){
    return present;
}
double PID::getI(){
    return past;
}
double PID::getD(){
    return future;
}

double PID::getPID(){
    return past + present + future;
}
double PID::getPD(){
    return future + present;
}

