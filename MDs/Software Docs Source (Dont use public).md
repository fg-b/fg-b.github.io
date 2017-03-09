#### Documentation written by Thomas Fraser & Alistair English

------

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

------

# Primary C++ File Source

#### Master

```c++
#include <Config.h>
#include <DirectionController.h>
#include <t3spi.h>
#include <Kicker.h>
#include <Buzzer.h>
#include <fgbcommon.h>
#include <Motor.h>
#include <Pins.h>

Motor motorD = Motor(MOTOR_A_PWM, MOTOR_A_DIR, MOTOR_A_BRK, MOTOR_A_REV); //Really Motor D
Motor motorA = Motor(MOTOR_B_PWM, MOTOR_B_DIR, MOTOR_B_BRK, MOTOR_B_REV); //Really Motor A
Motor motorB = Motor(MOTOR_C_PWM, MOTOR_C_DIR, MOTOR_C_BRK, MOTOR_C_REV);
Motor motorC = Motor(MOTOR_D_PWM, MOTOR_D_DIR, MOTOR_D_BRK, MOTOR_D_REV);

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
    pinMode(A12, INPUT);
    MASTER_TEENSY.begin_MASTER(ALT_SCK, MOSI, MISO, CS1, CS_ActiveLOW);
    MASTER_TEENSY.setCTAR(CTAR_0, 16, SPI_MODE0, LSB_FIRST, SPI_CLOCK_DIV8);
}

void loop(){
    MASTER_TEENSY.txrx16(dataOut, dataIn, DATA_LENGTH, CTAR_0, CS0);
    double angle = dataIn[0];
    delay(50);
    direction.calcMotors(angle, 0.00);

    // kicker.kickerReady(); //Kicker
    // kicker.checkLightGate();
    // kicker.kickBall();
}
//
// void testDirection(){
//     direction.setPWM(255);
//     delay(2000);
//     direction.setPWM(-255);
//     delay(2000);
// }
//
void blinkLED(){
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    delay(1000);
}
//
// void incrementSpeed(){
//     direction.setPWM(counter);
//     delay(100);
//     counter++;
//     if(counter >= 255){
//         counter = 0;
//     }
// }
```

#### TSOP

```c++
#include <ReadTSOPS.h>
#include <Config.h>
#include <t3spi.h>
#include <Arduino.h>
#include <Config.h>
#include <Pins.h>

ReadTSOPS tsops;
//
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

```c++
//#include <t3spi.h>
//#include <Light.h>

//T3SPI LightSPI;
//Light Light;

//int lightValues[19] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void setup(){
    Serial.begin(9600);
    pinMode(A7, INPUT);
    //um what the fuck is this lol
    //LightSPI.begin_SLAVE(ALT_SCK, MOSI, MISO, CS1); //Might be wrong CS pin.
    //LightSPI.setCTAR_SLAVE(16, SPI_MODE0);

    //Serial.print(Light.getAngle());
}

void loop(){
    //Light.getVals(lightValues);
    //for(int i=0; i < 19; i++){
      //Serial.print(lightValues[i]);
      //Serial.print(" ");
    //}
    //Serial.println();
    Serial.println(analogRead(A7));
    delay(100);
}
```



------

# Source

If you are reading this, it means I trust you with our source code before its opensourced. If you are going to take any concepts used here, please email us at [18135@bbc.qld.edu.au](18135@bbc.qld.edu.au).



### Buzzer



#### .h

```c++
#ifndef Buzzer_h
#define Buzzer_h

#include <Arduino.h>
#include <Config.h>
#include <Pins.h>
#include <Defines.h>

//
// The Buzzer Class
//
class Buzzer{

public:
    void playTone(int miliS);
    void errorTone();
    void readyTone();
private:

};

// Plays a tone for a amount of time
void Buzzer::playTone(int miliS){
    digitalWrite(BUZZER_PIN, HIGH);
    delay(miliS);
    digitalWrite(BUZZER_PIN, LOW);
}

// Plays a tone to signal an error until the Teensy loses power
void Buzzer::errorTone(){
    digitalWrite(BUZZER_PIN, HIGH);
}

// Plays two tones for 1.5 seconds to signal when the robot is ready to run
void Buzzer::readyTone(){
    playTone(500);
    delay(500);
    playTone(500);
}

#endif
```



### Compass



#### .h

```c++
#ifndef Compass_h
#define Compass_h

#include <Config.h>
#include <Pins.h>
#include <Defines.h>

#include <Wire.h>
#include <Vector3D.h>
#include <math.h>


class Compass {

public:
    Compass();
    double calibrate();
    Vector3D read();
    void update();
    double getHeading();
    void setTarget(double target_);

private:
    long previousTime;
    double calibration;
    double heading;
    double target;

    double doubleMod(double value, double maxVal);

    void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);
    void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);

    double convertRawGyro(int16_t raw) {
        // Since we are using 500 degrees/seconds range
        // -500 maps to a raw value of -32768
        // +500 maps to a raw value of 32767

        return (raw * 500.0) / 32768.0;
    }

};

#endif
```

#### .cpp

```c++
#include <Compass.h>

//========Constructor========
Compass::Compass(){
    Wire.begin();
    I2CwriteByte(IMU_ADDRESS, 29, 0x06);
    I2CwriteByte(IMU_ADDRESS, 26, 0x06);
    I2CwriteByte(IMU_ADDRESS, 27, GYRO_FULL_SCALE_500_DPS);
    I2CwriteByte(IMU_ADDRESS, 28, ACC_FULL_SCALE_2_G);
    I2CwriteByte(IMU_ADDRESS, 0x37, 0x02);
    I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16);

    setTarget(0);
}



//========Init Calibrate========
double Compass::calibrate(){
    read();

    delay(COMPASS_CALIBRATION_TIME);

    double reading = 0;
    for(int i=0; i<COMPASS_CALIBRATION_NUMBER; i++){
        reading += (double) read().z;
    }
    calibration = reading/COMPASS_CALIBRATION_NUMBER;

    return calibration;
}

//=====Read / Update======
Vector3D Compass::read(){
    uint8_t buffer[14];
    I2Cread(IMU_ADDRESS, 0x3B, 14, buffer);
    int16_t gx = -(buffer[8] << 8 | buffer[1]);
    int16_t gy = -(buffer[10] << 8 | buffer[11]);
    int16_t gz = buffer[12] << 8 | buffer[13];
    Vector3D returnVector = {convertRawGyro(gx), convertRawGyro(gy), convertRawGyro(gz)};
    return returnVector;
}

void Compass::update() {
    double reading = (double) read().z;

	long currentTime = micros();
    heading += (((double)(currentTime - previousTime) / 1000000.0) * (reading - calibration));
	heading = doubleMod(heading, 360.0);

	previousTime = currentTime;
}

//=======Set Target=======
void Compass::setTarget(double target_){
    target = -target_;
}

//=======Get Heading======
double Compass::getHeading(){
    double curr = (heading - 180) + target;
    return curr;
}


//======Utility Functions======
double Compass::doubleMod(double value, double maxVal){
    return fmod((value + maxVal), maxVal);
}

void Compass::I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{

  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void Compass::I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}
```



### Config



#### Config.h

```c++
#ifndef Config_h
#define Config_h

//Tsops
#define TSOP_NUM 12
#define READ_THRESHOLD 40
#define TSOP_NUM 12
#define MAX_READS 512
#define TSOP_FORWARD_LOWER 29
#define TSOP_FORWARD_UPPER 331
#define TSOP_ORBIT_ANGLE 90
#define TSOP_MIN_THRESHOLD 30

#define TSOP_K1 12
#define TSOP_K2 2
#define TSOP_K3 1

#define MED_STRENGTH 100
#define HIGH_STRENGTH 150

//Compass
#define COMPASS_CALIBRATION_TIME 2000
#define COMPASS_CALIBRATION_NUMBER 50

//SPI
#define DATA_LENGTH 1

//Lightsensors
#define LIGHTSENSOR_NUM 19
#define LIGHTSENSOR_THRESHOLD 30
#define DETECTED_NUMBER_LIGHT 3

//Pixy

//Kicker
#define KICKER_DELAY 2000L
#define LIGHTGATE_THRESHOLD 100

#endif
```

#### Defines.h

```c++
#ifndef Defines_h
#define Defines_h

//=========IMU (compass) Definitions=========
#define IMU_ADDRESS 0x68
#define MAG_ADDRESS 0x0C

#define GYRO_FULL_SCALE_250_DPS 0x00
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_2_G 0x00
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18

//Pixy
#define PIXY_CENTRE_X 160
#define PIXY_CENTRE_Y 100

//=================Other====================
#define angToRad 0.01745329251
#define radToAng 57.2957795131

//================Lightsensors==============
#define maxNumClusters 20
//CARTESIAN CO-ORDINATES
#define light1x 1.0
#define light1y 0.0

#define light2x 0.9510565162951535
#define light2y 0.3090169943749474

#define light3x 0.8090169943749475
#define light3y 0.5877852522924731

#define light4x 0.587785252292473
#define light4y 0.8090169943749475

#define light5x 0.30901699437494745
#define light5y 0.9510565162951535

#define light6x -0.30901699437494756
#define light6y 0.9510565162951535

#define light7x -0.587785252292473
#define light7y 0.8090169943749475

#define light8x -0.8090169943749473
#define light8y 0.5877852522924732

#define light9x -0.9510565162951535
#define light9y 0.3090169943749475

#define light10x -1.0
#define light10y 0.0

#define light11x -0.9510565162951536
#define light11y -0.3090169943749473

#define light12x -0.8090169943749472
#define light12y -0.5877852522924734

#define light13x -0.5877852522924732
#define light13y -0.8090169943749473

#define light14x -0.30901699437494756
#define light14y -0.9510565162951535

#define light15x 0.0
#define light15y -1.0

#define light16x 0.30901699437494723
#define light16y -0.9510565162951536

#define light17x 0.5877852522924737
#define light17y -0.809016994374947

#define light18x 0.8090169943749473
#define light18y -0.5877852522924734

#define light19x 0.9510565162951535
#define light19y -0.3090169943749476

#endif
```

#### Pins.h

```c++
#ifndef Pins_h
#define Pins_h

//========Tsop Pins========
#define POWER_PIN_1 0
#define POWER_PIN_2 20

//========Light Pins=======
//NOTE: SENSOR 1 IS THE RIGHT HAND ONE AND THEN THEY GO ANTICLOCKWISE AROUND THE CIRCLE
//(excluding the top most one - ask Al if ur unsure)
#define LIGHT_1 A4
#define LIGHT_2 A3
#define LIGHT_3 A2
#define LIGHT_4 A1
#define LIGHT_5 A0
#define LIGHT_6 A18
#define LIGHT_7 A17
#define LIGHT_8 A16
#define LIGHT_9 A15
#define LIGHT_10 A14
#define LIGHT_11 A13
#define LIGHT_12 A12
#define LIGHT_13 A11
#define LIGHT_14 A10
#define LIGHT_15 A9
#define LIGHT_16 A8
#define LIGHT_17 A7
#define LIGHT_18 A6
#define LIGHT_19 A5

//============Kicker pins==========
#define LIGHTGATE_PIN 0
#define KICKER_PIN 0

//============Buzzer pins==========
#define BUZZER_PIN 0

//Motor Pins
#define MOTOR_A_PWM 3
#define MOTOR_B_PWM 4
#define MOTOR_C_PWM 5
#define MOTOR_D_PWM 21
//
#define MOTOR_A_DIR 6
#define MOTOR_B_DIR 7
#define MOTOR_C_DIR 8
#define MOTOR_D_DIR 22
//
#define MOTOR_A_BRK 0
#define MOTOR_B_BRK 1
#define MOTOR_C_BRK 2
#define MOTOR_D_BRK 20
//
#define MOTOR_A_REV 1
#define MOTOR_B_REV 0
#define MOTOR_C_REV 1
#define MOTOR_D_REV 0

//============LED Indicators==========
#define LED_IND_1 28
#define LED_IND_2 27
#define LED_IND_3 26
#define LED_IND_4 25

#endif
```



### Direction Controller



#### .h

```c++
#ifndef DirectionController_h
#define DirectionController_h

#include <Config.h>
#include <Pins.h>
#include <Defines.h>
#include <Arduino.h>
#include <Motor.h>
// #include <Light.h>
#include <ReadTSOPS.h>
#include <RotationController.h>

struct lightStruct {
    bool seeing;
    double angle;
};

class DirectionController{

public:
    DirectionController();

    //Calculate motor angles
    void calcMotors(double angle, double rotation);

    //Calculate motor angles
    void calcMotors(double angle, double rotation, int dirNum);

    //Calculate lightsensors
    lightStruct calcLight();

    //Combines lightsensors and tsop direction
    void move(double angle);

    void setPWM(int pwm);

    Motor motorD = Motor(MOTOR_A_PWM, MOTOR_A_DIR, MOTOR_A_BRK, MOTOR_A_REV); //Really Motor D
    Motor motorA = Motor(MOTOR_B_PWM, MOTOR_B_DIR, MOTOR_B_BRK, MOTOR_B_REV); //Really Motor A
    Motor motorB = Motor(MOTOR_C_PWM, MOTOR_C_DIR, MOTOR_C_BRK, MOTOR_C_REV);
    Motor motorC = Motor(MOTOR_D_PWM, MOTOR_D_DIR, MOTOR_D_BRK, MOTOR_D_REV);

private:
    int angleArray[4] = {60, 135, 225, 300};

    double direction;

    // Light light = Light();

    double lightAngle;
    lightStruct values;

    // RotationController rotationController = RotationController();
};


#endif
```

#### .cpp

```c++
#include <DirectionController.h>

DirectionController::DirectionController(){
    // light.init();
}

void DirectionController::calcMotors(double angle, double rotation){
    //Solve the whole going forward while no seeing ball thing
    if(angle != 65506.00){
        motorA.set((cos(((angleArray[0] + 90) * angToRad) - (angle * angToRad))) * 100); //Probs should do this motor stuff in the main application? I guess we can do it here tho. Might be less clear to observers
        motorB.set((cos(((angleArray[1] + 90) * angToRad) - (angle * angToRad))) * 100);
        motorC.set((cos(((angleArray[2] + 90) * angToRad) - (angle * angToRad))) * 100);
        motorD.set((cos(((angleArray[3] + 90) * angToRad) - (angle * angToRad))) * 100);
    }else{
        motorA.set(0);
        motorB.set(0);
        motorC.set(0);
        motorD.set(0);
    }
}

void DirectionController::calcMotors(double angle, double rotation, int dirNum){
    int newAngle = (abs(round(angle/(360*1/dirNum))))*(360*1/dirNum);

    if(newAngle != -1){
        direction = newAngle < 180 ? (newAngle + 90) : (newAngle - 90);
    }

    motorA.set(((cos(((angleArray[0] + 90) * angToRad) - (direction * angToRad))) * 255) + rotation);
    motorB.set(((cos(((angleArray[1] + 90) * angToRad) - (direction * angToRad))) * 255) + rotation);
    motorC.set(((cos(((angleArray[2] + 90) * angToRad) - (direction * angToRad))) * 255) + rotation);
    motorD.set(((cos(((angleArray[3] + 90) * angToRad) - (direction * angToRad))) * 255) + rotation);
}

lightStruct DirectionController::calcLight(){
    values = {false, 0};
    //light.readLight();
    //light.averageAngles();
    //lightAngle = light.getAngle();
    if(lightAngle >= 0){
        values = {true, lightAngle};
        return values;
    }
    else{
        return values;
    }
}

void DirectionController::move(double tsopAngle){
    if(calcLight().seeing == false){
        calcMotors(tsopAngle, /*rotationController.rotate()*/0.00);
    }
    else{
        calcMotors(calcLight().angle/*Takes in angle to move away from line as oppose to following ball*/, /*rotationController.rotate()*/0.00);
    }
}

void DirectionController::setPWM(int pwm){
    motorA.set(pwm);
    motorB.set(pwm);
    motorC.set(pwm);
    motorD.set(pwm);
}
```



### FGB Common



#### .h

```c++
#include <Arduino.h>

#ifndef fgbcommon_h
#define fgbcommon_h

#define SHIFTARRAY(arr, input){                                 \
    for(int q = sizeof(arr)/sizeof(arr[0]) - 1; q >= 0; q--){   \
        *(arr + q + 1) = *(arr + q);                            \
    }                                                           \
    *(arr + 0) = input;                                         \
}                                                               \



#endif
```



### Kicker



#### .h

```c++
#ifndef Kicker_h
#define Kicker_h

#include <Arduino.h>
#include <Config.h>
#include <Pins.h>

enum kickerStatus {
    unknown,
    notReady,
    waitingForLightGate,
    waitingForCharge,
    ready
};

class Kicker{
public:
    Kicker();
    //Check if the kicker is ready to kick
    void kickerReady();
    //Check if the light gate is seeing the ball
    void checkLightGate();
    //Kick the ball
    void kickBall();
private:
    kickerStatus status;
    long currentMSec, lastKick;

};

#endif
```

#### .cpp

```c++
#include <Kicker.h>

Kicker::Kicker(){
    pinMode(KICKER_PIN, OUTPUT);
    digitalWrite(KICKER_PIN, LOW);
    pinMode(LIGHTGATE_PIN, INPUT);
    status = kickerStatus::unknown;
}

void Kicker::kickerReady(){
    long currentMSec = micros();
    if((currentMSec - lastKick) >= KICKER_DELAY){
        status = kickerStatus::waitingForLightGate;
    }
    else{
        status = kickerStatus::waitingForCharge;
    }
}

void Kicker::checkLightGate(){
    if(analogRead(LIGHTGATE_PIN) <= LIGHTGATE_THRESHOLD && status == kickerStatus::waitingForLightGate){
        status = kickerStatus::ready;
    }
}

void Kicker::kickBall(){
    if(status == kickerStatus::ready){
        digitalWrite(KICKER_PIN, HIGH);
        delay(50); //To Change in the future to a loop system. This will slow the robot down.
        digitalWrite(KICKER_PIN, LOW);
        lastKick = micros();
        status = kickerStatus::notReady;
    }
    else{
        status = kickerStatus::notReady;
    }
}
```



### Light

#### .h

```c++

```

#### .cpp

```c++

```



### Motor



#### .h

```c++
#ifndef Motor_h
#define Motor_h

// #include <Config.h>
// #include <Pins.h>
// #include <Defines.h>
#include <Arduino.h>

class Motor{

public:
    Motor(int pwm, int dir, int brk, int rev);

    //Set a pwm value to the motor
    void set(int pwm);
    //Stop the motor with no coast
    void brake();

private:
    int r_brk;
    int r_pwm;
    int r_dir;
    int reversed;
};

#endif
```

#### .cpp

```c++
#include <Motor.h>

Motor::Motor(int pwm, int dir, int brk, int rev){
    pinMode(pwm, OUTPUT);
    pinMode(dir, OUTPUT);
    pinMode(brk, OUTPUT);
    reversed = rev;
    r_pwm = pwm;
    r_dir = dir;
    r_brk = brk;
    analogWriteFrequency(pwm, 19000);
}
void Motor::set(int pwm){
    digitalWrite(r_dir, pwm > 0 ? !reversed : reversed);
    analogWrite(r_pwm, pwm > 0 ? pwm : -pwm);
    digitalWrite(r_brk, LOW);
}
void Motor::brake(){
    digitalWrite(r_dir, LOW);
    analogWrite(r_pwm, 0);
    digitalWrite(r_brk, HIGH);
}
```



### Read TSOPs



#### .h

```c++
#ifndef ReadTSOPS_h
#define ReadTSOPS_h

#include <Config.h>
#include <Pins.h>
#include <Defines.h>
#include <Arduino.h>
#include <math.h>

class ReadTSOPS{

public:
    ReadTSOPS();

    //Read the Tsops
    void read();

    //Reset the Tsops by dropping power
    void reset();

    //Stop reading tsops, error has occured
    void stop();

    //Calculate a tangent to move on to orbit around the ball
    int moveTangent();

    //Find the strength for the ball and determine how to orbit
    double findStrength();

    int bestSensor;
    double angleToBall;

private:
    int sensors[12] = {1,2,3,4,5,6,7,8,9,15,16,17};
    int values[12];
    int index;
    int value_index;
    int mod(int x, int m);
    double correctOrbit(double angleIn);

};

#endif
```

#### .cpp

```c++
#include <ReadTSOPS.h>
#include <Arduino.h>
// #define TSOP_NUM 12
// #define MAX_READS 128
//
// #define POWER_PIN_1 0
// #define POWER_PIN_2 0
// #define READ_THRESHOLD 40

ReadTSOPS::ReadTSOPS(){
    for(int i = 0; i < TSOP_NUM; i++){
        pinMode(sensors[i], INPUT);
    }
    pinMode(POWER_PIN_1, OUTPUT);
    pinMode(POWER_PIN_2, OUTPUT);

    bestSensor = 0;
    index = 0;
}

void ReadTSOPS::read(){
    bestSensor = 0;
    value_index = 0;
    index = -1;
    digitalWrite(POWER_PIN_1, HIGH);
    digitalWrite(POWER_PIN_2, HIGH);
    for(int j = 0; j < MAX_READS; j++){
        for(int i = 0; i < TSOP_NUM; i++){
            values[i] += (digitalRead(sensors[i]) == HIGH ? 0 : 1);
        }
    }
    digitalWrite(POWER_PIN_1, LOW);
    digitalWrite(POWER_PIN_2, LOW);
    delayMicroseconds(3000); //We can remove this if we dont need it later.
    for(int i = 0; i < TSOP_NUM; i++){
        // Filtering
        if(values[i] < TSOP_MIN_THRESHOLD){
            values[i] = 0;
        }
        if(values[i] > value_index){
            index = i; //1-12 as oppose to 0-11
            value_index = values[i];
        }
        values[i] = 0;
    }
    bestSensor = index;
}

void ReadTSOPS::reset(){
    digitalWrite(POWER_PIN_1, LOW);
    digitalWrite(POWER_PIN_2, LOW);
    delay(2);
    digitalWrite(POWER_PIN_1, HIGH);
    digitalWrite(POWER_PIN_2, HIGH);
}

void ReadTSOPS::stop(){
    digitalWrite(POWER_PIN_1, LOW);
    digitalWrite(POWER_PIN_2, LOW);
}

int ReadTSOPS::moveTangent(){ //Hmmmmm This shouldnt be done here, it should be done later down the course.
    read();
    //Begin weighting
    angleToBall = index * 30.00;

    return (int)correctOrbit(angleToBall);
    return (int)angleToBall;

    // if(angleToBall < 180.00 && angleToBall != 0){
    //     return angleToBall + 90.00;
    // }
    // if(angleToBall > 180.00 && angleToBall != 0){
    //     return angleToBall - 90.00;
    // }
    // return 0;
}

double ReadTSOPS::findStrength(){
     bestSensor = 0;
    value_index = 0;
    index = 0;
    for(int i = 0; i < TSOP_NUM; i++){
        values[i] = 0;
    }
    digitalWrite(POWER_PIN_1, HIGH);
    digitalWrite(POWER_PIN_2, HIGH);
    for(int j = 0; j < MAX_READS; j++){
        for(int i = 0; i < TSOP_NUM; i++){
            values[i] += (digitalRead(sensors[i]) == HIGH ? 0 : 1);
        }
    }
    digitalWrite(POWER_PIN_1, LOW);
    digitalWrite(POWER_PIN_2, LOW);
    delayMicroseconds(3000); //We can remove this if we dont need it later.
    for(int i = 0; i < TSOP_NUM; i++){
        // Filtering
        if(values[i] < 30){
            values[i] = 0;
        }
        if(values[i] > value_index){
            index = i + 1; //1-12 as oppose to 0-11
            value_index = values[i];
        }
        // values[i] = 0;
    }
    bestSensor = index;
    double tempStr = TSOP_K1 * values[bestSensor] + TSOP_K2 * (values[mod(bestSensor-1, TSOP_NUM)] + values[mod(bestSensor+1, TSOP_NUM)]) + TSOP_K3 * (values[mod(bestSensor-2, TSOP_NUM)] + values[mod(bestSensor+2, TSOP_NUM)]);
    return tempStr/16;
}

int ReadTSOPS::mod(int x,int m){
    int r = x % m;
    return r < 0 ? r + m : r;
}

double ReadTSOPS::correctOrbit(double angleIn){
    if(angleIn <= TSOP_FORWARD_LOWER || angleIn >= TSOP_FORWARD_UPPER){
        return angleIn;
    }else{
        return angleIn < 180 ? (angleIn + TSOP_ORBIT_ANGLE) : (angleIn - TSOP_ORBIT_ANGLE);
    }
}
```



### Rotation Controller



#### .h

```c++
#ifndef RotationController_h
#define RotationController_h

#include <Config.h>
#include <Pins.h>
#include <Defines.h>
#include <Arduino.h>
#include <PixyI2C.h>
#include <Compass.h>

class RotationController{

public:
    RotationController(){};

    //Gets the pixy information
    bool getPixy();

    //Calulates the pixy and which direction to go
    void calcPixy();

    //Calculates the rotation needed to face the centre of the goal
    void calcRotation();

    //Uses compass library to find the compass roataion required
    void getCompass();

    //The final rotation to add to the motors
    double rotate();

private:
    PixyI2C pixy;
    Compass compass = Compass();
    int rotationToAdd;
    int finalRotation;

    int blockHeight, blockWidth, blockX, blockY;

    double compassHeading;
    double newTarget;

};

#endif
```

#### .cpp

```c++
#include <RotationController.h>

bool RotationController::getPixy(){
    if(pixy.getBlocks()){ //seing the block
        blockHeight = pixy.blocks[0].height;
        blockWidth = pixy.blocks[0].width;
        blockX = pixy.blocks[0].x;
        blockY = pixy.blocks[0].y;
        return true;
    }
    return false;
}

void RotationController::getCompass(){
    compass.update();
    compassHeading = compass.getHeading();
}


void RotationController::calcPixy(){
    if(getPixy()){
        if(blockX >= PIXY_CENTRE_X){
            rotationToAdd = blockX - PIXY_CENTRE_X;
        }
        else if(blockX <= PIXY_CENTRE_X){
            rotationToAdd = PIXY_CENTRE_X - blockX;
        }
    }
    else{
        rotationToAdd = 0;
    }
}

void RotationController::calcRotation(){
    newTarget = compassHeading > 180 ? compassHeading - rotationToAdd : compassHeading + rotationToAdd;
    compass.setTarget(newTarget);
}

double RotationController::rotate(){
    compass.update();
    finalRotation = compass.getHeading();
}
```



---







