#include "MyServo/MYSERVO.h"
#include "hardware/pwm.h"

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
