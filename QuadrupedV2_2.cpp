#include "pico/stdlib.h"
#include <iostream>
#include "hardware/pwm.h"
#include "hardware/clocks.h"

int wrap =  39062;

class Servo{
    public:
    int servoPin;
    void write(float pusle); 
};
void Servo::write(float pulse){
    gpio_set_function(servoPin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(servoPin);

    pwm_set_enabled(slice_num,true);

    pwm_set_clkdiv(slice_num, 64.f);
    pwm_set_wrap(slice_num, 39062.f);

    //pwm_init(slice_num, &config, true);

    pwm_set_gpio_level(servoPin, (pulse/20000.f)*39062.f);
}
/*float clockDiv = 64;
float wrap = 39062;

void setMillis(int servoPin, float millis)
{
    pwm_set_gpio_level(servoPin, (millis/20000.f)*wrap);
}

void setServo(int servoPin, float startMillis)
{
    gpio_set_function(servoPin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(servoPin);

    pwm_config config = pwm_get_default_config();
    
    uint64_t clockspeed = clock_get_hz(5);
    clockDiv = 64;
    wrap = 39062;

    while (clockspeed/clockDiv/50 > 65535 && clockDiv < 256) clockDiv += 64; 
    wrap = clockspeed/clockDiv/50;

    pwm_config_set_clkdiv(&config, clockDiv);
    pwm_config_set_wrap(&config, wrap);

    pwm_init(slice_num, &config, true);

    pwm_set_gpio_level(servoPin, (startMillis/20000.f)*wrap);
}*/
bool direction = true;
int currentMillis = 400;
Servo servo; 

int main()
{
    servo.servoPin=3;
    while (true)
    {
        currentMillis += (direction)?5:-5;
        if (currentMillis >= 2400) direction = false;
        if (currentMillis <= 400) direction = true;
        servo.write(currentMillis);
        sleep_ms(10);
    }
}
