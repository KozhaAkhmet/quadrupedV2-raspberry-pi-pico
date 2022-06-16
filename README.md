# Quadruped Robot on Raspberry Pi Pico
## *This page is on maintains...*
#
# **Hardware**
This project have [the older version with Arduino](https://github.com/KozhaAkhmet/Quadruped_On_Arduino). I was maded that Robot for research and make fun).

### Preview :
[![asd](Pictures/Screenshot%20from%202022-03-12%2020-25-41.png)](https://www.youtube.com/watch?v=42VeJgC9H7w)


**Raspberry Pi Pico:**

I had been started with Arduino but at that time it began to get slower. Then I started to search for new, much faster hardware.Then I find Raspberry Pi Pico Which is much faster than arduino.I will mention only main parts,but you can check the differences in [this site](https://robu.in/raspberry-pi-pico-vs-arduino-which-to-choose/) . Arduino IDE and It`s libraries are very user friendly also same thing we can say to Thonny IDE, which is used for the pico but It only works with Micropython. Rapsberry pi Pico is also can programmed in C++ but it requires a work.

**nRF24L01:**

This module is used for *Radio Communications*. I prefer this because I already had [*Handcraft Radio Controller*](https://github.com/KozhaAkhmet/RC-Controller). 

**Protoboard:**

At the beginnig, I prefer to use a proto board. I`ll make the PCB in the future.

**SG90 Servo Motor**

I used the body of my old project. It already finished and do not require a work.

**MPU6050:**

I use it as Gyro sensor.

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

**Raspberry Pi Pico:** I place Raspberry at the cented on Protoboard. It gives a 3 pin space at sides where we place nRF and Servo.

**nRF24L01:** I connect nRF to 6,7,4 pins which RX,CS,SCK and 16,17 is CSn,CE. Use a SPI0 channel.

**Servos:** I group each servo to it`s leg then connect it to that 3 pin spaces.

**MPU6050:** This is going to be at bottom of the proto board.

![Connection Diagram](Pictures/Schematic_Quadruped%20On%20Pico_2022-04-13.png)

