// Include the libraries: mklink /J arduino_libraries "C:\Program Files (x86)\Arduino\libraries"
#include <Arduino.h>
#include "../.pio/libdeps/uno/Adafruit MPU6050/Adafruit_MPU6050.h"
#include "../.pio/libdeps/uno/TimerEvent/src/TimerEvent.h"
#include "../.pio/libdeps/uno/DHT sensor library/DHT.h"
#include "../.pio/libdeps/uno/Adafruit Unified Sensor/Adafruit_Sensor.h"
#include "../.pio/libdeps/uno/LiquidCrystal_I2C/LiquidCrystal_I2C.h"
#include "../.pio/libdeps/uno/TM16xx LEDs and Buttons/src/TM16xx.h"
#include "../.pio/libdeps/uno/TM16xx LEDs and Buttons/src/TM1640.h"
#include "../.pio/libdeps/uno/TM16xx LEDs and Buttons/src/TM16xxMatrix.h"
#include "../.pio/libdeps/uno/Servo/src/Servo.h"
#include "../.pio/libdeps/uno/IRremoteTank/IRremoteTank.h"

byte Heart[8] = { 0b00000, 0b01010, 0b11111, 0b11111, 0b01110, 0b00100, 0b00000, 0b00000};
byte Bell[8] = { 0b00100, 0b01110, 0b01110, 0b01110, 0b11111, 0b00000, 0b00100, 0b00000};
byte Alien[8] = { 0b11111, 0b10101, 0b11111, 0b11111, 0b01110, 0b01010, 0b11011, 0b00000};
byte Check[8] = { 0b00000, 0b00001, 0b00011, 0b10110, 0b11100, 0b01000, 0b00000, 0b00000};
byte Speaker[8] = { 0b00001, 0b00011, 0b01111, 0b01111, 0b01111, 0b00011, 0b00001, 0b00000};
byte Sound[8] = { 0b00001, 0b00011, 0b00101, 0b01001, 0b01001, 0b01011, 0b11011, 0b11000};
byte Skull[8] = { 0b00000, 0b01110, 0b10101, 0b11011, 0b01110, 0b01110, 0b00000, 0b00000};
byte Lock[8] = { 0b01110, 0b10001, 0b10001, 0b11111, 0b11011, 0b11011, 0b11111, 0b00000 };
//Array, used to store the data of the pattern, can be calculated by yourself or obtained from the modulus tool
unsigned char STOP01[] = {0x2E,0x2A,0x3A,0x00,0x02,0x3E,0x02,0x00,0x3E,0x22,0x3E,0x00,0x3E,0x0A,0x0E,0x00};
unsigned char hou[] = {0x00, 0x7f, 0x08, 0x08, 0x7f, 0x00, 0x3c, 0x42, 0x42, 0x3c, 0x00, 0x3e, 0x40, 0x40, 0x3e, 0x00};
unsigned char op[] = {0x00, 0x00, 0x3c, 0x42, 0x42, 0x3c, 0x00, 0x7e, 0x12, 0x12, 0x0c, 0x00, 0x00, 0x5e, 0x00, 0x00};
unsigned char met[] = {0xf8, 0x0c, 0xf8, 0x0c, 0xf8, 0x00, 0x78, 0xa8, 0xa8, 0xb8, 0x00, 0x08, 0x08, 0xf8, 0x08, 0x08};
unsigned char pesto[] = {0xfe, 0x12, 0x12, 0x7c, 0xb0, 0xb0, 0x80, 0xb8, 0xa8, 0xe8, 0x08, 0xf8, 0x08, 0xe8, 0x90, 0xe0};
unsigned char bleh[] = {0x00,0x11,0x0a,0x04,0x8a,0x51,0x40,0x40,0x40,0x40,0x51,0x8a,0x04,0x0a,0x11,0x00};
unsigned char clear[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
const uint8_t pesto2[] = {0x11, 0x0a, 0x04, 0x0a, 0x11, 0xc0, 0x40, 0x40, 0x40, 0x40, 0xc0, 0x11, 0x0a, 0x04, 0x0a, 0x11};


LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x3F for a 16 chars and 2 line display

#define light_L_Pin A0 ///define the pin of left photo resistor sensor
#define light_R_Pin A1 ///define the pin of right photo resistor sensor
#define IR_Pin A2
#define SCL_Pin  A5  //Set clock pin to A5
#define SDA_Pin  A4  //Set data pin to A4

#define Rem_OK 0xFF02FD
#define Rem_U 0xFF629D
#define Rem_D 0xFFA857
#define Rem_L 0xFF22DD
#define Rem_R 0xFFC23D

#define Rem_1 0xFF6897
#define Rem_2 0xFF9867
#define Rem_3 0xFFB04F
#define Rem_4 0xFF30CF
#define Rem_5 0xFF18E7
#define Rem_6 0xFF7A85
#define Rem_7 0xFF10EF
#define Rem_8 0xFF38C7
#define Rem_9 0xFF5AA5
#define Rem_0 0xFF4AB5
#define Rem_x 0xFF42BD
#define Rem_y 0xFF52AD


#define Trig 1  //ultrasonic trig Pin
#define Echo 0  //ultrasonic echo Pin
#define Led 2

#define matrixData 4  //servo Pin
#define matrixClock 5  //servo Pin
#define DHTPIN 6

#define servoPinXY 9  //servo Pin
#define servoPinZ 10  //servo Pin
#define ML_PWM 11   //define PWM control pin of left motor
#define MR_Ctrl 12  //define the direction control pin of right motor
#define ML_Ctrl 13  //define the direction control pin of left motor
#define MR_PWM 3   //define PWM control pin of right motor

int a=0;
IRrecv irrecv(IR_Pin);
decode_results results;
unsigned long ir_rec, duration, cm;

// Initialize DHT sensor for normal 16mhz Arduino:

TM1640 module(matrixData, matrixClock);
TM16xxMatrix matrix(&module, 16, 8);    // TM16xx object, columns, rows
DHT dht = DHT(DHTPIN, DHT11);

Adafruit_MPU6050 mpu;

const unsigned int timerOnePeriod = 1000;
const unsigned int timerTwoPeriod = 1250;
const unsigned int timerThreePeriod = 1666;
const unsigned int timerFourPeriod = 2200;
const unsigned int timerFivePeriod = 2500;
const unsigned int timerSixPeriod = 2700;
const unsigned int timerDHTPeriod = 1440;
const unsigned int timerGyroPeriod = 1000;

TimerEvent timerOne;
TimerEvent timerTwo;
TimerEvent timerThree;
TimerEvent timerFour;
TimerEvent timerFive;
TimerEvent timerSix;
TimerEvent timerDHT;
TimerEvent timerGyro;

long random2;
float distancef;
float distanceR;
float distanceL;


int sensorValueR ;        // value read from the pot
int sensorValueL ;        // value read from the pot
int outputValueR ;        // value output to the PWM (analog out)
int outputValueL ;        // value output to the PWM (analog out)
int calcValue ;      // inverse input
int pulsewidth;

char bluetooth_val; ///save the value of Bluetooth reception
int flag; ///flag variable, it is used to entry and exist function




/************the function to run motor**************/
void Car_front()
{
    analogWrite(Led, calcValue);
    digitalWrite(MR_Ctrl,HIGH);
    analogWrite(MR_PWM,255);
    digitalWrite(ML_Ctrl,HIGH);
    analogWrite(ML_PWM,255);
}
void Car_back()
{
    digitalWrite(MR_Ctrl,LOW);
    analogWrite(MR_PWM,200);
    digitalWrite(ML_Ctrl,LOW);
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
    digitalWrite(MR_Ctrl,HIGH);
    analogWrite(MR_PWM,50);
    digitalWrite(ML_Ctrl,HIGH);
    analogWrite(ML_PWM,30);
}
void Car_T_right()
{
    digitalWrite(MR_Ctrl,HIGH);
    analogWrite(MR_PWM,30);
    digitalWrite(ML_Ctrl,HIGH);
    analogWrite(ML_PWM,50);
}
/*

// This function will be called every timerOnePeriod
void timerOneFunc(){
    module.sendChar(pesto, 16);
}
void timerTwoFunc(){
    module.sendChar(reinterpret_cast<byte>(hou));
}
void timerThreeFunc(){
    module.sendChar(op,16);
}
void timerFourFunc(){
    module.sendChar(met,16);
}
void timerFiveFunc(){
    module.sendChar(pesto,16);
}
void timerSixFunc(){
    module.sendChar(bleh,16);
}
*/

void timerGyroFunc(){
    // Get new sensor events with the readings
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    lcd.setCursor(2,0);   //Set cursor to character 2 on line 0
    lcd.print("Hello Pesto!");
    lcd.setCursor(3, 1);
    lcd.write(0);
    lcd.setCursor(5, 1);
    lcd.print("Love u");
    lcd.setCursor(12, 1);
    lcd.write(0);

/*    lcd.setCursor(0,0); // Sets the location at which subsequent text written to the LCD will be displayed
    lcd.print("A:"); // m/s2
    lcd.print(a.acceleration.x);
    lcd.print(" ");
    lcd.print(a.acceleration.y);
    lcd.print(" ");
    lcd.print(a.acceleration.z);
    lcd.setCursor(0,1);
    lcd.print("R:");
    lcd.print(g.gyro.x);
    lcd.print(" ");
    lcd.print(g.gyro.y);
    lcd.print(" ");
    lcd.print(g.gyro.z);*/
}
void timerDHTFunc(){
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)

    // Read the humidity in %:
    float h = dht.readHumidity();
    // Read the temperature as Celsius:
    float t = dht.readTemperature();

    // Check if any reads failed and exit early (to try again):
    if (isnan(h) || isnan(t) ) {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }
    Serial.print("Humidity: ");
    Serial.println(h);
    Serial.print("Temperature: ");
    Serial.println(t);
    lcd.setCursor(0,0); // Sets the location at which subsequent text written to the LCD will be displayed
    lcd.print("T:"); // Prints string "Temp." on the LCD
    lcd.print(t); // Prints the temperature value from the sensor
    lcd.print(" H:");
    lcd.print(h);
    lcd.print("%");
}

//The function to control servo
void procedure(int pin,int myangle) {
    for (int i = 0; i <= 50; i = i + (1)) {
        pulsewidth = myangle * 11 + 500;
        digitalWrite(pin, HIGH);
        delayMicroseconds(pulsewidth);
        digitalWrite(pin, LOW);
        delay((20 - pulsewidth / 1000));
    }
}

//The function to control ultrasonic sensor
float checkdistance() {
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);
    float checkDistance = pulseIn(Echo, HIGH) / 58.00;  //58.20, that is, 2*29.1=58.2
    delay(10);
    return checkDistance;
}
//****************************************************************
void dance() {
    flag = 0; ///the design that enter obstacle avoidance function
    while (flag == 0) {
        for(int i=0; i<16; i++)
        {
            for(int j=0; j<8; j++)
            {
                procedure(servoPinXY, i*10);
                procedure(servoPinZ, j*10);
                matrix.setPixel(i,j, true);
                delay(10);
                matrix.setPixel(i,j, false);
            }
        }

        // One pixel, row by row
        for(int i=0; i<8; i++)
        {
            for(int j=0; j<16; j++)
            {
                procedure(servoPinXY, -i*10);
                procedure(servoPinZ, -j*10);
                matrix.setPixel(j,i, true);
                delay(10);
                matrix.setPixel(j,i, false);
            }
        }
        for (int myangle = 0; myangle <= 180; myangle += 1) { // goes from 0 degrees to 180 degrees
            // in steps of 1 degree
            procedure(servoPinXY, myangle);              // tell servo to go to position in variable 'myangle'
            delay(15);                   //control the rotation speed of servo
            // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
            // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
            digitalWrite(Trig, LOW);
            delayMicroseconds(2);
            digitalWrite(Trig, HIGH);
            delayMicroseconds(10);
            digitalWrite(Trig, LOW);
            // Read the signal from the sensor: a HIGH pulse whose
            // duration is the time (in microseconds) from the sending
            // of the ping to the reception of its echo off of an object.
            duration = pulseIn(Echo, HIGH);
            // Convert the time into a distance
            cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
            Serial.print(cm);
            Serial.print("cm");
            Serial.println();
            delay(5);
            if (cm>=2 && cm<=10)
                digitalWrite(10, HIGH);
            delay(100);
            digitalWrite(10, LOW);
            delay(100);
        }
        for (int myangle = 100; myangle >= 0; myangle -= 1) { // goes from 180 degrees to 0 degrees
            procedure(servoPinXY, myangle);              // tell servo to go to position in variable 'myangle'
            delay(10);
        }
        if (irrecv.decode(&results)) //receive the IR remote value
        {
            ir_rec=results.value;
            if (ir_rec == Rem_5)   //Robot car stops
            {
                Car_Stop();
                // matrix_display(STOP01);  //show stop image
            }
        }

    }
}

/*****************Obstacle Avoidance Function**************/
void avoid()
{
    flag = 0; ///the design that enter obstacle avoidance function
    while (flag == 0)
    {
        random2 = random(1, 100);
        a = checkdistance(); ///assign the front distance detected by ultrasonic sensor to variable a
        if (a < 20) ///when the front distance detected is less than 20cm
        {
            analogWrite (Led, 0);
            Car_Stop(); /// robot stops
            procedure(servoPinZ,45);
            delay(200); ///delay in 200ms
            procedure(servoPinZ,0);
            delay(200); ///delay in 200ms
            procedure(servoPinXY,160); ///Ultrasonic platform turns left
            for (int j = 1; j <= 10; j = j + (1)) { ///for statement, the data will be more accurate if ultrasonic sensor detect a few times.
                distanceL = checkdistance(); ///assign the left distance detected  by ultrasonic sensor to variable a1
            }
            delay(200);
            procedure(servoPinXY,20); ///Ultrasonic platform turns right
            for (int k = 1; k <= 10; k = k + (1)) {
                distanceR = checkdistance(); ///assign the right distance detected by ultrasonic sensor to variable a2
            }
            if (distanceL < 50 || distanceR < 50)
                ///robot will turn to the longer distance side when left or right distance is less than 50cm.if the left or right
                /// distance is less than 50cm, the robot will turn to the greater distance
            {
                if (distanceL > distanceR) ///left distance is greater than right
                {
                    procedure(servoPinXY,90); ///Ultrasonic platform turns back to right ahead ultrasonic platform turns front
                    Car_left(); ///robot turns left
                    delay(500); ///turn left 500ms
                    Car_front(); ///go forward
                }
                else
                {
                    procedure(servoPinXY,90);
                    Car_right(); ///robot turns right
                    delay(500);
                    Car_front(); ///go forward
                }
            }
            else ///both distance on two side is greater than or equal to 50cm, turn randomly
            {
                if ((long) (random2) % (long) (2) == 0) ///when the random number is even
                {
                    procedure(servoPinXY,90);
                    Car_left(); ///robot turns left
                    delay(500);
                    Car_front(); ///go forward
                }
                else
                {
                    procedure(servoPinXY,90);
                    Car_right(); ///robot turns right
                    delay(500);
                    Car_front(); ///go forward
                } } }
        else ///If the front distance is greater than or equal to 20cm, robot car will go front
        {
            Car_front(); ///go forward
        }
        if (irrecv.decode(&results)) //receive the IR remote value
        {
            ir_rec=results.value;
            if (ir_rec == Rem_5)   //Robot car stops
            {
                Car_Stop();
                // matrix_display(STOP01);  //show stop image
            }
        }
    }
}




void setup(){
    Serial.begin(115200);

    pinMode(servoPinXY, OUTPUT);
    procedure(servoPinXY, 90); //set servo to 90°
    pinMode(servoPinZ, OUTPUT);
    procedure(servoPinZ, 0); //set servo to 90°
    irrecv.enableIRIn(); // Initialize the IR receiver

    pinMode(Trig, OUTPUT);
    pinMode(Echo, INPUT);
    pinMode(ML_Ctrl, OUTPUT);
    pinMode(ML_PWM, OUTPUT);
    pinMode(MR_Ctrl, OUTPUT);
    pinMode(MR_PWM, OUTPUT);
    pinMode(Led, OUTPUT);

    pinMode(SCL_Pin,OUTPUT);
    pinMode(SDA_Pin,OUTPUT);
    digitalWrite(SCL_Pin,LOW);
    digitalWrite(SDA_Pin,LOW);

    dht.begin();
    lcd.init();
    lcd.clear();
    lcd.backlight();      // Make sure backlight is on
    // create a new characters
    lcd.createChar(0, Heart);
    lcd.createChar(1, Bell);
    lcd.createChar(2, Alien);
    lcd.createChar(3, Check);
    lcd.createChar(4, Speaker);
    lcd.createChar(5, Sound);
    lcd.createChar(6, Skull);
    lcd.createChar(7, Lock);

    // Print a message on both lines of the LCD.
    lcd.setCursor(2,0);   //Set cursor to character 2 on line 0
    lcd.print("Hello Pesto!");
    lcd.setCursor(3, 1);
    lcd.write(0);
    lcd.setCursor(5, 1);
    lcd.print("Love u");
    lcd.setCursor(12, 1);
    lcd.write(0);

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // 2, 4, 8, 16G
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);      // 250, 500, 1000, 2000 deg
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);    // 5, 10, 21, 44, 94, 164, 260hz

    module.clearDisplay();
    // One pixel, column by column
    for(int i=0; i<16; i++)
    {
        for(int j=0; j<8; j++)
        {
            matrix.setPixel(i,j, true);
            delay(10);
            matrix.setPixel(i,j, false);
        }
    }

    // One pixel, row by row
    for(int i=0; i<8; i++)
    {
        for(int j=0; j<16; j++)
        {
            matrix.setPixel(j,i, true);
            delay(10);
            matrix.setPixel(j,i, false);
        }
    }


/*    timerOne.set(timerOnePeriod, timerOneFunc);
    timerTwo.set(timerTwoPeriod, timerTwoFunc);
    timerThree.set(timerThreePeriod, timerThreeFunc);
    timerFour.set(timerFourPeriod, timerFourFunc);
    timerFive.set(timerFivePeriod, timerFiveFunc);
    timerSix.set(timerSixPeriod, timerSixFunc);*/
    timerDHT.set(timerDHTPeriod, timerDHTFunc);
    timerGyro.set(timerGyroPeriod, timerGyroFunc);

}


void loop(){

    timerOne.update();
    timerTwo.update();
    timerThree.update();
    timerFour.update();
    timerFive.update();
    timerSix.update();
    timerDHT.update();
    timerGyro.update();

    random2 = random(1, 100);
    sensorValueR = analogRead(light_R_Pin);
    sensorValueL = analogRead(light_L_Pin);
    outputValueR = map(sensorValueR, 0, 1023, 0, 255);
    outputValueL = map(sensorValueL, 0, 1023, 0, 255);
    calcValue = 255 - ((outputValueR + outputValueL) * 1.5);
    calcValue = (calcValue < 0) ? 0 : calcValue;
    distancef = checkdistance();  //assign the front distance detected by ultrasonic sensor to variable a

    if (Serial.available())
    {
        bluetooth_val = Serial.read();
        Serial.println(bluetooth_val);
    }


    if (irrecv.decode(&results)) //receive the IR remote value
    {
        ir_rec=results.value;
        String type="UNKNOWN";
        String typelist[14]={"UNKNOWN", "NEC", "SONY", "RC5", "RC6", "DISH", "SHARP", "PANASONIC", "JVC", "SANYO", "MITSUBISHI", "SAMSUNG", "LG", "WHYNTER"};
        if(results.decode_type>=1&&results.decode_type<=13){
            type=typelist[results.decode_type];
        }
        Serial.print("IR TYPE:"+type+"  ");
        Serial.println(ir_rec,HEX);
        irrecv.resume();
    }

    if (ir_rec == Rem_2) //Go forward
    {
        procedure(servoPinXY, 90);  //Servo rotates to 90°
        procedure(servoPinZ, 0);
        Car_front();
//        matrix_display(&hou);  //Display front image
    }
    if (ir_rec == Rem_8)  //Robot car goes back
    {
        procedure(servoPinXY, 90);  //Servo rotates to 90°
        procedure(servoPinZ, 0);
        Car_back();
//        matrix_display(op);  //Go back
    }
    if (ir_rec == Rem_1)   //Robot car turns left
    {
        procedure(servoPinXY, 160);  //Servo rotates to 90°
        procedure(servoPinZ, 30);
        Car_T_left();
//        matrix_display(met);  //Display left-turning image
    }
    if (ir_rec == Rem_3)   //Robot car turns right
    {
        procedure(servoPinXY, 20);  //Servo rotates to 90°
        procedure(servoPinZ, 30);
        Car_T_right();
//        matrix_display(pesto);  //Display right-turning image
    }
    if (ir_rec == Rem_5)   //Robot car stops
    {
        Car_Stop();
       // matrix_display(STOP01);  //show stop image
    }
    if (ir_rec == Rem_4)   //robot car rotates anticlockwise
    {        procedure(servoPinXY, 160);  //Servo rotates to 90°
        procedure(servoPinZ, 30);
        Car_left();
//        matrix_display(bleh);  //show anticlockwise rotation picture
    }
    if (ir_rec == Rem_6)  //robot car rotates clockwise
    {        procedure(servoPinXY, 20);  //Servo rotates to 90°
        procedure(servoPinZ, 30);
        Car_right();
//        matrix_display(bleh);  //show clockwise rotation picture
    }
    if (ir_rec == Rem_x)  //robot car rotates clockwise
    {
        avoid();

    }
    if (ir_rec == Rem_y)  //robot car rotates clockwise
    {
        dance();

    }


}


