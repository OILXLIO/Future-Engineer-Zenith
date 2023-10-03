# The Documentation for Future Engineer 2023
### [Zenith] 


![](https://lh3.googleusercontent.com/u/0/drive-viewer/AJc5JmS-gvzix8rqHiP9ptq7tHeZygsObiNOmIOgPZ77TDPFEsBTKvNW-LatH-ngLn_0nhACZ-FHElf1pMwIivX24kCyNMjfTw=w1920-h929)


<p align="center">
  <img src="https://ybrobot.club/image/YB%20Robot%20logo.png" width="200"/>
</p>
<p align="center">
<b>By Yothinburana School Robot Club</b>
</p>

![Team](https://github.com/OlyPongpapat/Future-Engineer--BOO/assets/122149490/4e4d9215-4092-4590-ab22-bc3f343689a1)


<br><br><br><br>

## Robot Running (Youtube Links): 
Round 1 - https://youtu.be/AAdyjSv6BT0
Last Round - 

<br>


### This file contains:

- Designing process
- Programing explain

<br><br><br><br>

# **Part 1 : Robot Designing** ✨

<br>

### **The Robot** 🤖

![124229_0](https://github.com/OlyPongpapat/Future-Engineer-Zenith/assets/146799155/9823c8da-0cab-416f-8422-2352ca476deb)

<br>

This robot is made almost entierly out of lego. So, our designing process is quite hard

<br>

## **Choosing parts of our robot:**

<br>

- **Driving Motor** ( Power Functions L-Motor )

![4728-src](https://github.com/OlyPongpapat/Future-Engineer--BOO/assets/122149490/0f56b5d3-27e3-4ab8-bcaa-c41aa6cab4f0)


This motor is small yet power full. and it has the perfect size for our robot.
The motor come with a dedicated port for lego. So, we modified to make it able to connect to the board.

<br>

- **Ultrasonic Sensor** ( Gravity: URM09 Ultrasonic Distance ) to measure the distance between the robot and the walls

![image](https://camo.githubusercontent.com/16c1386fb545f593a4291ff12f99c2c1107c30e7dbdb3e14bd4e977f374d7ae9/68747470733a2f2f6466696d672e6466726f626f742e636f6d2f73746f72652f646174612f53454e303330372f53454e303330372e6a70673f696d61676556696577322f312f772f3536342f682f333736)

DFRobot URM09 is an ultrasonic sensor specifically designed for fast ranging and obstacle avoidance application.
This Ultrasonic can measure distance between 2-500 in centimeter prcisely which is very important in this mission.

<br>

- **Servo** ( GEEKSERVO-360 ) one for **steering** our robot and another for rotating **ultrasonic sensor**

![8da8dc3c-0330-47e2-adb3-314d5796e34e](https://github.com/OlyPongpapat/Future-Engineer-Zenith/assets/146799155/13a08c56-4dc1-482b-bff6-812d6c08bf86)


- **Color Sensor** - We use **Virus-III** that Designed by Sopon and **ZX-03** that Design by INEX to detect the color of the field.

![image](https://media.discordapp.net/attachments/998531635219796011/1143546792269586555/IMG_4607.jpg?width=286&height=405)  ![th](https://github.com/OlyPongpapat/Future-Engineer-Zenith/assets/146799155/059d9221-6a78-4121-a1ab-30730911ec04)


- **Compass or Gyro** - We use **GY-25** compass to control the direction of our robot.

<img src="https://inwfile.com/s-o/q6moq1.jpg"/>

- **Camera** - We use **Pixy 2.1** for our robot because of its very easy to use and its has free libraly and application to configure it.
<img src="https://www.zagrosrobotics.com/images/pixy2_3.jpg"  width="400"/>

- **Microcontroller Board** - **Arduino UNO**, the Processor unit for our robot

<img src="https://www.warf.com/imagesitem/original/6534_3736.jpg" width="400"/>

- **Shield L298P Motor Driver with GPIO**, enables arduino UNO to control  motors

<img src="https://static.cytron.io/image/cache/catalog/products/SHIELD-L298P-BUZZ/Shield-L298P-BUZZ%20(1)-800x800.jpg"  width="400"/>
  
**Wire connections for our robot**
```c
//  Motor B
int const ENB = 11;
int const INB = 13;

//  Buzzer
int const BUZZER = 4;

//  Servos
int const STEER_SRV = 9;
int const ULTRA_SRV = 8;

//  Ultrasonic Sensor
int const ULTRA_PIN = 2;

//  Light Sensors
int const RED_SEN = 0;
int const BLUE_SEN = 1;

//  Button
int const BUTTON = 3;

//  Pixy Camera
int const PIXY_SDA = 4
int const PIXY_SCL = 5

//  Gryo
// Connect to TX RX
```

![](https://lh3.googleusercontent.com/fife/AAbDypBMpb49LbKQ5uKGLfBDmphj8Q8arFVbUlOLNF8VVuXZUzkWvl3eadt5nIXetApAGM7xuwYlJYdm-KfruJEWDW08ieDruFHPaADXrP7M5XFcIfTFgCtY1fjK248FAwB_7itTqeUqzz8pINb41MQSfPzSzkifoEEMkDSyi1I-xJGw0YoBDbfG9zFeddrwtirY0IhSuuieEyrTlboi465iIsCqC__VBJN0-RV1PZkTUowbddHRlyaFzzxbg7O-Jov3FQW7jEgIFs0O2N1yQPx4CO8iDaT0TOcZ1_F7_eemipeXO4S4E6m0oJfD1PPAM21phtAkt1eKt8yExGOCzG3ldnAfriJ87OFrO_T0MO6mjr4OHHut8QX6QlA9YFfd1lUWe9POO6wRtRw-sgNifQiTRhEBlmrgXWLpPbZ0hRhcZYgA7tPZLdf0NT4UTS3-fbQSNGOIUq_ymm8Gx5ijanCEv4of1LiZajwnGiM-FbypDGq1SEkXtfCRteScp8EkKMZRXxhc3aLF2WPfy5UFPwGRDMAdBydtPxS6vD59GdavwcT2wM0_KaMOVXi_bt4DuZG5uf1BWUjaZsJMuJi8ep2sVsNUydBfVH7GA_FGagEzsXjmVWXOPzN4mLbx8haLA8274SpNiXhLMwLUam78VUcSefPEwddz6qgcSfW9c38OUQM03DozOGG3lE8nlZVFuu34VbXgZ8JWC1oPFVYxpYdMX1uQuENEolRZLeBf9sLs4i6UjiRAc-Btg6yNIvgf6CE30h-MIxXZP_CzLSxrODLrGQ3JfhYhH6h9DbrDEqD1pzlYd6HwN8QLn2BRTaJ3SNkk-Tw5X4YlSjZ-BdXg5h5t8j5lQq5Bku3wTZZ7ko15zrIhn_KO2Khng5n6a3ZsXnzQ6rLfF4_KyWNc7id1gsK8KbAWzRpw2AFXCg7J0mUvOmT2wbMUgJLRstg56BeaUlR1xZEQj-dMW8avdAiY0MJnnNpAz7QgXOLF-1UgCnmjKFx5rBme2_4KOm02dMK3lPYj1iV_REC5tAvsLiW4PKRkK6pgRKA_UWmDZiGZTorhgJcVrHkbaeyVv7oI31QaOPY4ULHsYju2wQRvFyWVeWqs5aMI0TRtCV72LUWD1kFNwv89BkGQWG-RRzy5oaxuqUjjrEll-fXJ9713vTQudQbFeSJgInNSrL6URnIIbswHI6UHWtb6ngPBqpmc_I5vtKsSB0Nz8LublM7z5L1st0ipVBGiEdzOLmtNY2P5TjpzVuQp1BeVMTFz9hoh4Mw7k2N9WQ6MN5EnKXEihFJDayxoqXcQYlAfnT-nycc0X8_VMKgcp_BbtZDKjNSZLS-3P9kW6iJQZ-svLY-Kgk1fj6WakgY1gD_dJBEikCEsC6Xg3xRZ0m75TbM-XgiBCYJun125R1O8SiooYNDEDzBQkEclHLUtWslDrk6fyHvGTah7XUi6VBy-ZvxaCCGKSGZ5y7_hiDPAFMNbYElfjDyPHmzphBHIXzpD17WWDNppPShQ0lq32baZH7wJpz7RYXy8GUUJY3-pWqFe1nv2C6-V-6pPQYzPuNusvADoTesBf1jDZtpq0cXsnKuOVTBUcc7a3pai9Sk5pePf14ab34IXVvzUipNU9P8xd653gCYYeh9qnW9DJgX8KiSAKvAuFn17jcAXdeDG=w1920-h929)  


# **Part 2 : Programming**
 
Please noted that this program is specially write for Arduino Uno. If you want to recreated it. You may need to change the method of writing the program by your self.

<br>

### **Required Programs**

- **[Arduino IDE 2.1.1]** - For Programming your robot
- **[Pixymon V2]** - For displaying and setting up your Pixy

<br>

### **Required Libraries**

To be able to use the sensors, many libraries need to be downloaded and included in this project. The libraries that will need to be downloaded will be depend on the sensors you use. Most of these library are important for completing 
both round of the competition.

All the libraries used can be included in the program using the **Library Manager** in **Arduino IDE.** except for the Pixy library which you need to download separately from [this website.](https://pixycam.com/downloads-pixy2/)

```
#include  "Mapf.h"

#include  <Servo.h>

#include  <PID_v2.h>

#include  <Pixy2I2C.h>
```

## Setting up() 

Finally we got to the exciting part 
Please, download our Arduino program, and use it as a reference, so you can furture understand our method.

<br>

### Setup()

For the ```setup()```, we have to include these libraries and set the variable of the sensors in order for the robot to work as wanted.

```c
#include  "Mapf.h"
#include  <Servo.h>
#include  <PID_v2.h>
#include  <Pixy2I2C.h>
Pixy2I2C pixy;

PID_v2 compassPID(0.9, 0.001, 0.07, PID::Direct);

void setup() {
  compassPID.Start(0, 0, 0);
  compassPID.SetOutputLimits(-180, 180);
  compassPID.SetSampleTime(10);
  pinMode(BUZZER, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(STEER_SRV, OUTPUT);
  pinMode(ULTRA_SRV, OUTPUT);
  pinMode(ULTRA_PIN, INPUT);
  pinMode(RED_SEN, INPUT);
  pinMode(BLUE_SEN, INPUT);
  pinMode(BUTTON, INPUT);
  Serial.begin(115200);
  pixy.init();
	while (!Serial);

	servo1.attach(STEER_SRV, 600, 2400);
	servo2.attach(ULTRA_SRV, 600, 2400);
	steering_servo(0);
	ultra_servo(0, 'L');

	// check_leds();
	while (analogRead(BUTTON) > 500);
	zeroYaw();
	while (analogRead(BUTTON) <= 500);
}
```


<br><br>

## first Round Program explaination

The program can be simplified to the following parts:

<br>

###  Steering of the Robot - Turns the servo used for controlling the robot driving direction to a specified degree.

Variables that affect the steering incudes:

- The input degree [ The direction we want to robot to steer ]
- The direction of the robot measured by **compass sensor** [ The direction that the robot is now facing ]
- The distance between the wall and the robot measured by **the ultrasonic sensor** [ Preventing the robot from hitting the wall ]

```c
void loop() {
  motor(80);
  while (analogRead(BUTTON) > 500) {
    getIMU();  
    line_detection();
    steering_servo(1 * compassPID.Run(pvYaw + ((getDistance() - 25) * 1) * ((float(TURN == 'R') - 0.5) * 2)));
    ultra_servo(-pvYaw, TURN);
  }
  while (analogRead(BUTTON) <= 500)
    ;
  motor(0);
  while (analogRead(BUTTON) > 500)
    ;
  while (analogRead(BUTTON) <= 500)
    ;
}
```

<br>

###  The Speed of the Driving Motor

Variables that affect the speed of the motor includes:

- The direction of the robot measured by **compass sensor** [ If the robot is facing in the wrong direction, decrease the speed (to give it enough time to process) ]

```c
void motor_and_steer(int degree) {
  degree = max(min(degree, 45), -45);
  steering_servo(degree);
  // motor(35);
  motor(map(abs(degree), 0, 45, 40, 27));
}
```

<br>

###  🚗 Steer the robot to left or right when a line is detected - There are 2 lines on each corner of the mission field(one red, one blue). This program detects those lines and tell the robot to steer to the correct direction. 

Variables that affect the turning of the robot includes:
- The Reflected light values measured by the **VirusIII**

```c
void line_detection() {
  int blue_value = analogRead(BLUE_SEN);
  if (TURN == 'U') {
    int red_value = analogRead(RED_SEN);
    if (blue_value < 600 || red_value < 200) {
      int lowest_red_sen = red_value;
      long timer_line = millis();
      while (millis() - timer_line < 100) {
        int red_value = analogRead(RED_SEN);
        if (red_value < lowest_red_sen) {
          lowest_red_sen = red_value;
        }
      }
      if (lowest_red_sen > 144) {
        // Red
        TURN = 'R';
        compass_offset += 90;
        // beep();
      } else {
        // Blue
        TURN = 'L';
        compass_offset -= 90;
        // beep();
        // delay(100);
        // beep();
        // delay(100);
        // beep();
      }
      halt_detect_line_timer = millis();
    }
  } else {
    if (millis() - halt_detect_line_timer > 1000) {
      if (blue_value < 600) {
        if (TURN == 'L') {
          compass_offset -= 90;
        } else {
          compass_offset += 90;
        }
        halt_detect_line_timer = millis();
      }
    }
  }
}
```
<br>

**This program ends after turning at the corners of the field 12 times (3 rounds).**

<br><br>

## last Round Program explaination

The program can be simplified to the following parts:

<br>

### Steering of the Robot - Turns the geekservo used for steering the robot to a specified degree.

Variables that affect the steering incudes:

- The color and position of the color blocks detected by **Pixy 2.1** [ Adjusts the vector of the robot to avoid hitting the detected obstacle on the field ]
- The input degree [ The direction we want to robot to steer to ]
- The direction of the robot measured by **compass sensor** [ The direction that the robot is now facing ]
- The distance between the wall and the robot measured by **the gravity ultrasonic sensor** [ Preventing the robot from hitting the wall]


```c
void loop() {

  long countdown_stop = millis();
  while (analogRead(BUTTON) > 500) {

    getIMU();
    ultra_servo(-pvYaw, Blocks_TURN);
    line_detection();
    float distance_wall = getDistance();
    float steering_degree = 1 * compassPID.Run(pvYaw + ((distance_wall - 25 )) * ((float(Blocks_TURN == 'R') - 0.5) * 2));
    if (millis() - pixy_timer > 50) {
      avoidance_degree = calculate_avoidance();
      pixy_timer = millis();
    }
    // int final_degree = (found_block || (found_block_factor > 0) ? mapf(min(max(distance_wall, 10), 45), 10, 45, steering_degree, avoidance_degree) : steering_degree);
    int final_degree = map(max(found_block, found_block_factor), 1, 0, mapf(min(max(distance_wall, 10), 45), 10, 45, steering_degree, -3 * avoidance_degree), steering_degree);
    //steering_servo(steering_degree);
    // steering_servo(-avoidance_degree);
    if (millis() - countdown_stop > 3000) {
      // Stops everything
      motor(0);
      while (true)
        ;
    }
    if (lines_detect_num < 12) {
      countdown_stop = millis();
    }
    motor_and_steer(final_degree);
  }
  motor(0);
  while (analogRead(BUTTON) <= 500)
    ;
  while (analogRead(BUTTON) > 500)
    ;
  while (analogRead(BUTTON) <= 500)
  ;
}
```
<br>

The calculating of degree in which the robot needs to turn to avoid the color blocks, the program can be simplified as follows:

- Calculate the distace between the camera and the block using this following formula:
> distance = size_obj * focal_length / size_obj_on_sensor

<br>
After calculating the distace, we need to find the position of the block using **Trigonometry**.

```c
pixy.ccc.getBlocks();
...

float(atan2(1000 * 2 / pixy.ccc.blocks[nearest_index].m_height * tan(float(map(pixy.ccc.blocks[nearest_index].m_x, 0, 316, -30, 30) + Wrap(bearingPID - initial_deg, -180, 179)) / 180 * PI) - 20, 1000 * 2 / pixy.ccc.blocks[nearest_index].m_height - 10)) * 180 / PI
```

> atan2!? 
> It is the 2-argument arctangent. By definition, degree (radians) = atan2(y,x)

> Find out more about atan2: [atan2 - Wikipedia]

As shown in the program above, we can calculate the driving direction degree using the **atan2 function**. Setting the **y coordinate** by measuring the position of the color blocks in 3D space using **the Pixy camera** then **subtract or add 20 cm** to **offset the y coordinate**, so that the robot will not hit the detected obstacle. The same for the **x coordinate**, and then we get the final steering degree! 

<br>

###  Changing the Speed of the Driving Motor - transmission of data with L298n to drive the motor at a specified speed with a specified acceleration.

Variables that affect the speed of the motor includes:

- The direction of the robot measured by **compass sensor** [ If the robot is facing in the wrong direction, decelerate the speed ]

**Uses the same code previously mentioned**

<br>
### 🚗 Steer the robot to left or right when a line is detected - There are 2 lines on each corner of the mission field(one red, one blue). This program detects those lines and tell the robot to steer to the correct direction.  

Variables that affect the turning of the robot includes:
- The number of color blocks detected by **the pixy camera** [ To make sure the robot turns at the corner of the field after avoiding the obstacles on the field ]
- The Reflected light values measured by the **Virus IIISensors**


**Uses mostly the same code previously mentioned**

<br>



**This program ends after turning at the corners of the mission field 12 times (3 rotation).**

<br>


# **THE END**
For further questions please email pawit.book@gmail.com

   [Yothinburana School Robot Club]: <https://ybrobot.club/>
   [Original Flipped Digital Lab]: <https://ofdl.tw/en/>
   [EV3FW-V1.21C-OFDL.bin]: <https://github.com/a10036gt/EV3-BatteryInfo-Block/releases/download/v1.2/EV3FW-V1.21C-OFDL.bin>
   [OFDL-EV3_Blocks-Collections]: <https://github.com/a10036gt/OFDL-EV3_Blocks-Collections/releases/tag/2020.09.12>
   [Pixy Blocks and Examples]: <https://github.com/charmedlabs/pixy/raw/master/releases/lego/lego_pixy-1.1.4.zip>
   [EV3 Education Lab]: <https://e498eb58-16e9-491c-8ce4-828510ab7c41.filesusr.com/archives/1f66bb_4708cf7510f64585bd447c26a4110fc5.zip?dn=LME-EV3_Full-setup_1.4.5_en-US_WIN32.zip>
   [Pixymon V2]: <https://github.com/charmedlabs/pixy/raw/master/releases/pixymon_windows/pixymon_windows-2.0.9.exe>
   [Wikipedia - Bezier Curve]: <https://en.wikipedia.org/wiki/B%C3%A9zier_curve>
   [Arduino-pico Github]: <https://github.com/earlephilhower/arduino-pico>
   [Arduino IDE 2.1.1]: <https://www.arduino.cc/en/software>
   [atan2 - Wikipedia]: <https://en.wikipedia.org/wiki/Atan2>
