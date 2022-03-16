# Quadruped Robot on Raspberry Pi Pico
## *This page is on maintains...*
#
# **Hardware**
This project have [older version with Arduino](https://github.com/KozhaAkhmet/Quadruped_On_Arduino). I made made this Robot for research and make fun).

### Preview :
[![asd](Pictures/Screenshot%20from%202022-03-12%2020-25-41.png)](https://www.youtube.com/watch?v=42VeJgC9H7w)


**Raspberry Pi Pico:**

I was started with Arduino but it began to get not enoght. Then I started to search new much faster hardware then i find Raspberry Pi Pico. You can check the differences in [this site](https://robu.in/raspberry-pi-pico-vs-arduino-which-to-choose/) i will mention only main parts. At first Raspberry have 133 MHz, 264kB of SRAM and it have 16 GPIO Pins which gives HUGH advantage in cumputaion and connection. But in coding Arduino is really user friendly you will say that "Raspberry also have Thorn which nearly same to it" Yes it is, if we use Python. Unfortunetaly we use C++ to be faster where compiling and debugging require work with CMake.

**nRF24L01:**

This module is using *Radio Waves*. I prefer this because i already had [*Handcraft Radio Controller*](https://github.com/KozhaAkhmet/RC-Controller). I was thinking about WIFI or Bluetooth but it will be took much more time to learn and connecting. Long story short I continued where I started.

**Protoboard:**

I had the different size Protoboards. I was thinking about PCB but i choose to take 20x14 Protoboard and weld with hands than learn to make PCB and to order it. That choise gives me a chance to use trial and error method and it fits perfectly with other parts. In addition at that time i did not know about pinouts of Raspberry and other connection stuffs. 

**SG90 Servo Motor**

I really wanted to use new Mechanical Body and much precised servos but i needed to build according my budged. I used the body of my old project. 

**MPU6050:**

Gyro sensor is main part of motion sensors. Im using it first time.

**In Conclusion I use :**
| Name                   | Count       |  Used For As  |
| -----------            | ----------- | ------------  |
| Raspberry Pi Pico      | 1           | Brain         |
| nRF24L01               | 1           | Communication |
| Protoboard 16x20       | 1           | Protoboard    |
| MPU6050                | 1           | Gyrosensor    |
| SG90 Servo Motor       | 12          | Servo Motor   |
___

## **Connections**:
---
**Raspberry Pi Pico:** I place Raspberry at the cented on Protoboard. It gives a space 3 pins at sides where we place nRF and Servo

**nRF24L01:** I connect nRF to 6,7,4 pins which RX,CS,SCK and 16,17 is CSn,CE. Use a SPI0 channel.

**Servos:** I group each servo to it`s leg then connect it to that 3 pin spaces.

**MPU6050:** 




![Pico pinout](Pictures/raspberry-pi-pico-pinout-featured-image.jpg)

# **Software**
Libraries:
```C
    #include "pico/stdlib.h"
    #include <iostream>
    #include "hardware/pwm.h"
    #include "pico/binary_info.h"
    #include "hardware/clocks.h"
    #include "hardware/i2c.h"
    #include <math.h>
```

Data Structure

```c
    class Leg {       //Creating Leg class and its ingredients
    public:
        Ang lastAng;
        Pos lastPos;
        Servo servo[3];
        void toPos(double posX, double posY, double posZ);
        void toAng(double al, double bet , double gam );
        void step (double posX, double posY, double posZ);
        void slide(double posX, double posY ,double posZ);
    };
```



References:
- 
  
You can also check up [Pico C++ Documentation](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf) where you can find many information about PWM commands and connecting MPU6050.