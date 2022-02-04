#include "pico/stdlib.h"
#include "hardware/pwm.h"


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
