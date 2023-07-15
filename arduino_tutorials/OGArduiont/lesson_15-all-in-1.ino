#include <Arduino.h>
#include "../../.pio/libdeps/uno/Adafruit MPU6050/Adafruit_MPU6050.h"
#include "../../.pio/libdeps/uno/TimerEvent/src/TimerEvent.h"
#include "../../.pio/libdeps/uno/DHT sensor library/DHT.h"
#include "../../.pio/libdeps/uno/Adafruit Unified Sensor/Adafruit_Sensor.h"
#include "../../.pio/libdeps/uno/Adafruit HMC5883 Unified/Adafruit_HMC5883_U.h"
#include "../../.pio/libdeps/uno/LiquidCrystal_I2C/LiquidCrystal_I2C.h"
#include "../../.pio/libdeps/uno/TM16xx LEDs and Buttons/src/TM16xx.h"
#include "../../.pio/libdeps/uno/TM16xx LEDs and Buttons/src/TM1640.h"
#include "../../.pio/libdeps/uno/TM16xx LEDs and Buttons/src/TM16xxMatrix.h"
#include "../../.pio/libdeps/uno/Servo/src/Servo.h"
#include "../../.pio/libdeps/uno/IRremoteTank/IRremoteTank.h"

unsigned char start01[] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};
unsigned char front[] = {0x00,0x00,0x00,0x00,0x00,0x24,0x12,0x09,0x12,0x24,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char back[] = {0x00,0x00,0x00,0x00,0x00,0x24,0x48,0x90,0x48,0x24,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char left[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x28,0x10,0x44,0x28,0x10,0x44,0x28,0x10,0x00};
unsigned char right[] = {0x00,0x10,0x28,0x44,0x10,0x28,0x44,0x10,0x28,0x44,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char STOP01[] = {0x2E,0x2A,0x3A,0x00,0x02,0x3E,0x02,0x00,0x3E,0x22,0x3E,0x00,0x3E,0x0A,0x0E,0x00};
unsigned char clear[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
#define SCL_Pin A5 //Set clock pin to A5
#define SDA_Pin A4 //Set data pin to A4
#define ML_Ctrl 13 //define direction control pin of left motor
#define ML_PWM 11 //define PWM control pin of left motor
#define MR_Ctrl 12 //define direction control pin of right motor
#define MR_PWM 3 //define PWM control pin of right motor
#define Trig 5 //ultrasonic trig Pin
#define Echo 4 //ultrasonic echo Pin
int distance;
int random2;
int a;
int a1;
int a2;
#define servoPin 9 //servo Pin
int pulsewidth;
#define light_L_Pin A1
#define light_R_Pin A2
int left_light;
int right_light;
char bluetooth_val; //save the value of Bluetooth reception
int flag; //flag variable, it is used to entry and exist function

float checkdistance() {
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);
    float distance = pulseIn(Echo, HIGH) / 58.00;
    delay(10);
    return distance;
}
//The function to control servo the function controlling servo
void procedure(int myangle) {
    for (int i = 0; i <= 50; i = i + (1)) {
        pulsewidth = myangle * 11 + 500;
        digitalWrite(servoPin,HIGH);
        delayMicroseconds(pulsewidth);
        digitalWrite(servoPin,LOW);
        delay((20 - pulsewidth / 1000));
    }
}

/***************Dot Matrix *****************/
// this function is used for dot matrix display
void matrix_display(unsigned char matrix_value[])
{
    IIC_start();
    IIC_send(0xc0); //Choose address
    for(int i = 0;i < 16;i++) //pattern data has 16 bits
    {
        IIC_send(matrix_value[i]); //convey the pattern data
    }
    IIC_end(); //end the transmission of pattern data
    IIC_start();


    IIC_send(0x8A); //display control, set pulse width to 4/16
    IIC_end();
}
//The condition starting to transmit data
void IIC_start()
{
    digitalWrite(SCL_Pin,HIGH);
    delayMicroseconds(3);
    digitalWrite(SDA_Pin,HIGH);
    delayMicroseconds(3);
    digitalWrite(SDA_Pin,LOW);
    delayMicroseconds(3);
}
//convey data
void IIC_send(unsigned char send_data)
{
    for(char i = 0;i < 8;i++) //each byte has 8 bits
    {
        digitalWrite(SCL_Pin,LOW);
        delayMicroseconds(3);
        if(send_data & 0x01)
        {
            digitalWrite(SDA_Pin,HIGH);
        }
        else
        {
            digitalWrite(SDA_Pin,LOW);
        }
        delayMicroseconds(3);
        digitalWrite(SCL_Pin,HIGH);
        delayMicroseconds(3);
        send_data = send_data >> 1;
    }
}
//The sign that data transmission ends
void IIC_end()
{
    digitalWrite(SCL_Pin,LOW);
    delayMicroseconds(3);
    digitalWrite(SDA_Pin,LOW);
    delayMicroseconds(3);
    digitalWrite(SCL_Pin,HIGH);
    delayMicroseconds(3);
    digitalWrite(SDA_Pin,HIGH);
    delayMicroseconds(3);
}
/*************the function to run motor*************/
void Car_front()
{
    digitalWrite(MR_Ctrl,LOW);
    analogWrite(MR_PWM,200);
    digitalWrite(ML_Ctrl,LOW);
    analogWrite(ML_PWM,200);
}
void Car_back()
{
    digitalWrite(MR_Ctrl,HIGH);
    analogWrite(MR_PWM,200);
    digitalWrite(ML_Ctrl,HIGH);
    analogWrite(ML_PWM,200);
}
void Car_left()
{
    digitalWrite(MR_Ctrl,LOW);
    analogWrite(MR_PWM,255);
    digitalWrite(ML_Ctrl,HIGH);
    analogWrite(ML_PWM,255);
}
void Car_right()
{
    digitalWrite(MR_Ctrl,HIGH);
    analogWrite(MR_PWM,255);
    digitalWrite(ML_Ctrl,LOW);
    analogWrite(ML_PWM,255);
}
void Car_Stop()
{
    digitalWrite(MR_Ctrl,LOW);
    analogWrite(MR_PWM,0);
    digitalWrite(ML_Ctrl,LOW);
    analogWrite(ML_PWM,0);
}
void Car_T_left()
{
    digitalWrite(MR_Ctrl,LOW);
    analogWrite(MR_PWM,255);
    digitalWrite(ML_Ctrl,LOW);
    analogWrite(ML_PWM,180);
}
void Car_T_right()
{
    digitalWrite(MR_Ctrl,LOW);
    analogWrite(MR_PWM,180);
    digitalWrite(ML_Ctrl,LOW);
    analogWrite(ML_PWM,255);
}


/*****************Obstacle Avoidance Function**************/
void avoid()
{
    flag = 0; //the design that enter obstacle avoidance function
    while (flag == 0)
    {
        random2 = random(1, 100);
        a = checkdistance();
        if (a < 20) //when the front distance detected is less than 20cm
        {
            Car_Stop(); //robot stops
            delay(200); //delay in 200ms
            procedure(160); //Ultrasonic platform turns left

            for (int j = 1; j <= 10; j = j + (1)) {
                a1 = checkdistance();
            }
            delay(200);
            procedure(20); //Ultrasonic platform turns right
            for (int k = 1; k <= 10; k = k + (1)) {
                a2 = checkdistance();
            }
            if (a1 < 50 || a2 < 50)
            {
                if (a1 > a2) //left distance is greater than right
                {
                    procedure(90);
                    Car_left(); //robot turns left
                    delay(500); //turn left 500ms
                    Car_front(); //go forward
                }
                else
                {
                    procedure(90);
                    Car_right(); //robot turns right
                    delay(500);
                    Car_front(); //go forward
                }
            }
            else
            {
                if ((long) (random2) % (long) (2) == 0)
                        {
                        procedure(90);
                        Car_left(); //robot turns left
                        delay(500);
                        Car_front(); //go forward
                        }
                else
                {
                    procedure(90);
                    Car_right(); //robot turns right
                    delay(500);
                    Car_front(); ///go forward
                }
            }
        }
        else //If the front distance is greater than or equal to 20cm, robot car will go front
        {
            Car_front(); //go forward
        }
        if (Serial.available())
        {
            bluetooth_val = Serial.read();
            if (bluetooth_val == 'S') //receive S
            {
                flag = 1; //when assign 1 to flag, end loop
            }
        }
    }
}
/*******************Follow****************/
void follow() {
    flag = 0;
    while (flag == 0) {
        distance = checkdistance();
        if (distance >= 20 && distance <= 60) //the range to go front
        {
            Car_front();
        }
        else if (distance > 10 && distance < 20) //the range to stop
        {
            Car_Stop();
        }
        else if (distance <= 10) // the range to go back
        {
            Car_back();
        }
        else //other situations, stop
        {
            Car_Stop();
        }
        if (Serial.available())
        {
            bluetooth_val = Serial.read();
            if (bluetooth_val == 'S')
            {
                flag = 1; //end loop
            }
        }
    }
}


/****************Light Follow******************/
void light_track() {
    flag = 0;
    while (flag == 0) {
        left_light = analogRead(light_L_Pin);
        right_light = analogRead(light_R_Pin);
        if (left_light > 650 && right_light > 650)
        {
            Car_front();
        }
        else if (left_light > 650 && right_light <= 650) //the value
            detected by photo resistor, turn left
        {
            Car_left();
        }
        else if (left_light <= 650 && right_light > 650) //the value
            detected by photo resistor, turn right
        {
            Car_right();
        }
        else //other situations, stop
        {
            Car_Stop();
        }
        if (Serial.available())
        {
            bluetooth_val = Serial.read();
            if (bluetooth_val == 'S') {
                flag = 1;
            }
        }
    }
}

void setup(){
    Serial.begin(9600);
    pinMode(Trig, OUTPUT);
    pinMode(Echo, INPUT);
    pinMode(ML_Ctrl, OUTPUT);
    pinMode(ML_PWM, OUTPUT);
    pinMode(MR_Ctrl, OUTPUT);
    pinMode(MR_PWM, OUTPUT);

    pinMode(servoPin, OUTPUT);
    procedure(90); //set servo to 90° pinMode(SCL_Pin,OUTPUT);
    pinMode(SDA_Pin,OUTPUT);
    matrix_display(clear); //Clear the display
    matrix_display(start01); //display start pattern
    pinMode(light_L_Pin, INPUT);
    pinMode(light_R_Pin, INPUT);
}
void loop(){
    if (Serial.available())
    {
        bluetooth_val = Serial.read();
        Serial.println(bluetooth_val);
    }
    switch (bluetooth_val)
    {
        case 'F': //Forward instruction
            Car_front();
            matrix_display(front); //display forward pattern
            break;


        case 'B': //Back instruction
            Car_back();
            matrix_display(back); // display back pattern
            break;
        case 'L': //left-turning instruction
            Car_left();
            matrix_display(left); //show left-turning pattern
            break;
        case 'R': //right-turning instruction
            Car_right();
            matrix_display(right); //show right-turning pattern
            break;
        case 'S': //stop instruction
            Car_Stop();
            matrix_display(STOP01); //display stop pattern
            break;
        case 'Y':
            matrix_display(start01); //show start pattern
            follow();
            break;
        case 'U':
            matrix_display(start01); //show start pattern
            avoid();


            break;
        case 'X':
            matrix_display(start01); //show start pattern
            light_track();
            break;
    }
}