

#### Documentation written by Thomas Fraser & Alistair English

-----

# Installation 

To use our code source, clone the repo

```shell
mkdir FG&BCode
cd FG&BCode
git clone https://github.com/TomFraser/FG-B-2017.git
```

Install Platformio for arduino compiler

```shell
pip3 install platformio
```

Run and upload code

```shell
cd FG-B-2017
cd [] #Motor, Light or TSOP

#To Compile
pio run
#To Upload
pio run -t upload
```



Please remember that the code source is preset with our pinouts and configuration. For more information please visit the hardware docs.

# Buzzer

#### Contains (.h)

> The buzzer libraries primary job is to control the buzzer used for feedback on the robot’s condition during match or debugging. The buzzer library consists of one primary class and three primary functions; *playTone*, *errorTone* & *readyTone.* The *playTone* method takes in one input of type integer and plays a buzz sound for the time specified (in milliseconds) in the input. It does this by pulling a digital pin on the Teensy and connected to the buzzer to the high state which then plays a tone. After the delay the pin is pulled low stopping the sound. The *errorTone* method is sounded when the robot encounters an error. The method simply sounds the buzzer and will not stop until the program is stopped. The final method *readyTone* plays when the robot and completed its setup which includes checking the SPI communication between the three teensy controllers on each robot. The tone is simply used to notify the user when setup is complete and the robot is ready to be used.

To use the library, import it to your main c++ file

```c++
#include <buzzer.h>
```

To declare the buzzer object

```c++
Buzzer audio = Buzzer();
```

To use the buzzer call the method

```c++
audio.playTone(200);
```



# Compass

#### Contains (.h, .cpp)

> The compass library handles the task of controlling the mpu9250 9-axis compass, gyro and magnetometer chip. The compass communicates with the Teensy using the I2c protocol. The library consists of many public and private methods; *calibrate, read, update, getHeading, setTarget*. The *calibrate* method handles the task of calibrating the compass, specifically the gyroscope. It does this by taking multiple reading over a certain delay to find the drift of the gyroscope while the robot is standing still. This allows us to counter the natural drift that the gyro has. Each reading throughout the game will take or add the drift to ensure that all headings are accurate. The *read* method simply takes a reading from the compass over I2c and gets the Z component (from XYZ) of the gyroscope. It then returns the reading from the gyro. The *update* method takes the reading, counters the drift and mods the value from 0 to 360 degrees. This allows us to have an accurate heading in degrees. We store this heading and use it later. The *setTarget* method takes in one variable being of type double which allows the software to set the desired heading of the robot. This allows us to easily make the robot face towards the goal by changing the target heading that the robot is trying to achieve. The *getHeading* method adds 180 to the heading which means that instead of 0,360 being at the centre of the robot, 180 is. This helps because if 0,360 is at the centre front, the robot ticks between the two values and doesn’t correct as well. There are other ways to fix this but by adding 180 to the heading is the easiest and fastest option software wise.

To use the library, import it into your main c++ file

```c++
#include <Compass.h>
```

To declare the compass object

```c++
Compass compass = Compass();
```

To use the compass call these methods

```c++
compass.getUpdate();
double heading = compass.getFinalHeading();
```



# Config

#### Contains (.h)

> The different header files under the name config is where most variables that can be changed live along with constants and pin definitions. Each header file consists of mostly *\#define* constants that can be changed by the user to dictate how the robot plays. This is simply a much cleaner way of having a setting file.

## Config.h

Config contains configurable constants useful for modifying the behaviour of the robot.

## Defines.h

Defines contains set constants that shouldn't need to be changed.

## Pins.h

Pins contains the constant definitions of pins for the teensy micro-controller.

To use these libraries import them using

```c++
#include <Config.h>
#include <Defines.h>
#include <Pins.h>
```



# Direction Controller

#### Contains (.h, .cpp)

The direction controllers main purpose is to handle the final movement of the robot. The library itself receives sensor information and sorts it into a final direction of movement for the robot. The methods within the library are; *calcMotors, calcLight and move.* The *calcMotors* method had two separate definitions, one that calculates an exact angle to move and one that calculates a restricted angle based on input in the program. The job of *calcMotors* is to use trigonometry to find a final *pwm* value to write to the motor (set between -255 and 255). The equation used is as follows: $\cos\left( motorAngle + 90 \right)\left( \text{radians} \right) - angleToBall\left( \text{radians} \right)*255$. The second definition of *calcMotors* simply limits the output of the above equation to factors of 45, giving the robot 8 directions of orbit. The *calcMotors* method also takes in the rotation determined by the *Rotation Controller* library. The *calcLight* method simply takes an input from the Light Sensor library in the form of a direction and sets the angle to move to said angle. If the Light Sensors aren’t seeing anything of value, the method returns a struct with the values of a Boolean and integer in the format of: *{false, 0} or {true, \_\_angle\_\_}*. The *move* method simply checks the returned values from *calcLight* to determine which angle to follow, the one from the light sensors or the ball. It then calls the *calcMotors* method with the final angle as an input and adds rotation from the *Rotation Controller* library.

To use the library, import it into your main c++ file

```c++
#include <DirectionController.h>
```

To declare the direction controller object

```c++
DirectionController direction = DirectionController();
```

To use primary methods

```c++
direction.calcMotors(angle, rotation);
//or
direction.calcMotors(angle, rotation, orbitNum);
//or
direction.setPWM(value/*-255 - 255*/);
```



# FGB Common

#### Contains (.h)

The FGB Common header contains methods that are being globally used across multiple libraries and methods that don't belong to a class.

To use the library, import it into your main c++ file

```c++
#include <fgbcommon.h>
```



# Kicker

#### Contains (.h)

> The Kicker libraries primary purpose is to control the solenoid and light gate. The library consists of three methods; *kickerReady, checkLightGate and kickBall*. The *kickerReady* method checks to make sure that a delay between kicking the ball occurs. The capacitors that power the solenoid need time to charge back to capacity. During this time, it is advised that the kicker isn’t used. The *kicker\_delay* constant is used to determine how long of a gap exists between the solenoid kicking. The *checkLightGate* method simply checks whether the light gate used for detecting the ball has been broken. If the light gate has been broken the kicker is ready to kick. The *kickBall* method simply triggers the solenoid and kicks the ball.



To use the library, import it into your main c++ file

```c++
#include <Kicker.h>
```

To call the constructor and declare the kicker object

```c++
Kicker kicker = Kicker();
```

To use the kicker, the lightgate must first be triggered, then the code checks if the delay between kicks it long enough (this is to allow the caps for the solenoid to charge). If both criteria are met, the solenoid will kick.

```c++
kicker.kickerReady();
kicker.checkLightGate();
kicker.kick();
```



# Light

#### Contains (.h, .cpp)

> The Light libraries purpose is to read, collect and analyse data from the 20 lightsensors on the bottom of the robot. It does this by preforming an analogRead on all 20 pins connected to the sensors, it then calculates a line using quadrants of triggered sensors. With this information, the robot then calculates a line average from which it moves away from. The robot also stores values for the line location allowing it to be pushed over the line and still make a recovery back into the play area.

The Light library controls all 20 lightsensors on the botom of the robot and keeps us from going out over the white line. The library reads the 20 lightsensors situated in a circular pattern giving us full control of the robots position relative to the line.

To use the Light library, import it into your main c++ file

```c++
#include <Kicker.h> //Do I really need to keep doing this for each libaray or do you get the jist?
```

To delcare the Light object

```c++
Light lightsensors = Light();
```

To be completely honest, the other programmer not writing this (Alistair) wrote the lightsensor code and the programmer writing this (Tom) has no damn idea how it works.



# Motor

#### Contains (.h, .cpp)

> The motor libaray has the purpose of splitting a single input value into all the requirements to control the motors via the LMD18200T motor controllers. The *set* method takes in a pwm value between -255 to 255 and then writes that value as a speed to the motor. The motor controller have 4 primary inputs that the microcontroller must handle. They are direction, pwm, brake and reversed. The direction pin simply controls whether the motor spins forwards or backwards, this binary value is determined by the sign infront of the pwm value given to the motor. The pwm is absolute value between 0-255. The brake pin is set low during use of the motor as to not damage it. The reversed pin simply sets the default direction of the motor from the get go, this allows you to make all the motors spin in one direction when given only power. 

The motor library controls our 4 Maxon DCX Motors using the LMD18200T motor control chip. The library writes Pulse Width Modulation values to the motor controllers giving us movement.

To use the Motor ibaray, import it into your main c++ file

```c++
#include <Motor.h>
```

To declare the Motor object

```c++
Motor motorA = Motor(pwm_pin, brk_pin, dir_pin, isRev);
Motor motorB = Motor(pwm_pin, brk_pin, dir_pin, isRev);
Motor motorC = Motor(pwm_pin, brk_pin, dir_pin, isRev);
Motor motorD = Motor(pwm_pin, brk_pin, dir_pin, isRev);
```

To use the motors

```c++
motorA.set(pwm_value);
//or
motorA.brake();
```



# Read TSOPs

#### Contains (.h, .cpp)

> The Read TSOPs libaray reads, identifies and calculates the most accurate location of the ball. The read tsops libaray also controls the 'orbit' of the robot. The orbit is the robots projected path to get behind the ball to prevent the ball from reaching our own goal. The *read* method in Read TSOPs is where most of the critical jobs are preformed. Inside the read function the tsops are read a total of 512 times to get an accurate location followed by filtering and trig calculations.

The Read TSOPs library reads and controls our TSOP infared sensors. The TSOPs are the robots primary way of ball detection.

To use the Read TSOPs library, import it into your main c++ file

```c++
#include <ReadTSOPS.h>
```

To declare the TSOP object

```c++
ReadTSOPS tsops;
```

To use the TSOPS

```c++
tsops.read();
//or
tsops.moveTangent();
```



# Rotation Controller

#### Contains (.h, .cpp)

> The rotation controller doesnt work.

The Rotation Controller library handles all rotational (non directional) movement. The rotation controller takes both pixy and compass input into account and acts accordingly.

To use the Rotation Controller libaray, import it into your main c++ file

```c++
#include <RotationController.h>
```

To declare the Rotation Controller object

```c++
RotationController rotation = RotationController();
```

Unfortunately the Rotation Controller contains code that is very specific to our robots (being the compass MPU9250 and Pixy Camera) so I wont provide any sample use.

---

# Primary C++ File Source

#### Master

> The Master controls the motors, solenoid, pixy and compass (maybe SRF10s?).

```c++
#include <Config.h>
#include <DirectionController.h>
#include <t3spi.h>
#include <Kicker.h>
#include <Buzzer.h>
#include <fgbcommon.h>
#include <PixyI2C.h>
#include <Motor.h>
#include <Pins.h>

Motor motorD = Motor(MOTOR_A_PWM, MOTOR_A_DIR, MOTOR_A_BRK, MOTOR_A_REV); //Really Motor D
Motor motorC = Motor(MOTOR_B_PWM, MOTOR_B_DIR, MOTOR_B_BRK, MOTOR_B_REV); //Really Motor A
Motor motorB = Motor(MOTOR_C_PWM, MOTOR_C_DIR, MOTOR_C_BRK, MOTOR_C_REV);
Motor motorA = Motor(MOTOR_D_PWM, MOTOR_D_DIR, MOTOR_D_BRK, MOTOR_D_REV);

//C, A, B, D

volatile uint16_t dataOut[DATA_LENGTH] = {};
volatile uint16_t dataIn[DATA_LENGTH] = {};

// MotorController MOTOR = MotorController();
T3SPI MASTER_TEENSY;
// Kicker kicker = Kicker();
DirectionController direction = DirectionController();
// Buzzer buzzer = Buzzer();

double tsopAng = 0.00;

int counter = 0;

void setup(){
    Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_29_30, I2C_PULLUP_EXT, 400000);
    Wire1.setDefaultTimeout(200000); // 200ms

    pinMode(A12, INPUT);
    MASTER_TEENSY.begin_MASTER(ALT_SCK, MOSI, MISO, CS0, CS_ActiveHIGH); //Begin Master
    MASTER_TEENSY.setCTAR(CTAR_0, 16, SPI_MODE0, LSB_FIRST, SPI_CLOCK_DIV8); //Set Packet Cache

    MASTER_TEENSY.enableCS(CS4, CS_ActiveHIGH); //Set CS for TSOPs
    //MASTER_TEENSY.enableCS(CS0, CS_ActiveHIGH); //Need to set CS for LIGHTs
    direction.init();
}

void loop(){
    //Motor Transmission
    MASTER_TEENSY.txrx16(dataOut, dataIn, DATA_LENGTH, CTAR_0, CS4); //Pull data from TSOPs
    double angle = dataIn[0];
    Serial.println(angle);
    delay(50);
    //Light Transmission
    // MASTER_TEENSY.txrx16(dataOut, dataIn, DATA_LENGTH, CTAR_0, CS4); //Pull data from LIGHTs
    // double lightAngle = dataIn[0];
    // Serial.println(lightAngle);

    direction.calcMotors(angle, 0.00, 0.00);
}

void blinkLED(){
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    delay(1000);
}
```

#### TSOP

> The TSOP slave controls and calculates tsop and ball locations.

```c++
#include <ReadTSOPS.h>
#include <Config.h>
#include <t3spi.h>
#include <Arduino.h>
#include <Config.h>
#include <Pins.h>

ReadTSOPS tsops;

volatile uint16_t dataIn[DATA_LENGTH] = {};
volatile uint16_t dataOut[DATA_LENGTH] = {};

T3SPI TSOP;

void setup(){
    TSOP.begin_SLAVE(ALT_SCK, MOSI, MISO, CS0);
    TSOP.setCTAR_SLAVE(16, SPI_MODE0);
    NVIC_ENABLE_IRQ(IRQ_SPI0);
}

void loop(){
    dataOut[0] = tsops.moveTangent();
}

void spi0_isr(){
    TSOP.rxtx16(dataIn, dataOut, DATA_LENGTH);
}
```

#### Light

> The Light slave calculates lightsensors and robot / field positions.

```c++
#include <t3spi.h>
#include <Config.h>
#include <Arduino.h>
#include <Light.h>

Light Light;

T3SPI LIGHT;

volatile uint16_t dataIn[DATA_LENGTH] = {};
volatile uint16_t dataOut[DATA_LENGTH] = {};

double lightAngle;

void setup(){
    Light.init();
    Serial.begin(9600);
    LIGHT.begin_SLAVE(ALT_SCK, MOSI, MISO, CS0);
    LIGHT.setCTAR_SLAVE(16, SPI_MODE0);

    NVIC_ENABLE_IRQ(IRQ_SPI0);
}

void loop(){
    Light.readLight();
    Serial.println(Light.getAngle());
    lightAngle = Light.getAngle();
    dataOut[0] = 100;
}

void spi0_isr(){
    LIGHT.rxtx16(dataIn, dataOut, DATA_LENGTH);
}
```

---



