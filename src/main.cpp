// Include the libraries: mklink /J arduino_libraries "C:\Program Files (x86)\Arduino\libraries"
#include <Arduino.h>
#include "../.pio/libdeps/uno/Adafruit MPU6050/Adafruit_MPU6050.h"
#include "../.pio/libdeps/uno/Adafruit Unified Sensor/Adafruit_Sensor.h"
#include "../.pio/libdeps/uno/Adafruit HMC5883 Unified/Adafruit_HMC5883_U.h"
#include "../.pio/libdeps/uno/LiquidCrystal_I2C/LiquidCrystal_I2C.h"
#include "../.pio/libdeps/uno/TM16xx LEDs and Buttons/src/TM16xx.h"
#include "../.pio/libdeps/uno/TM16xx LEDs and Buttons/src/TM1640.h"
#include "../.pio/libdeps/uno/TM16xx LEDs and Buttons/src/TM16xxMatrix.h"
#include "../.pio/libdeps/uno/Servo/src/Servo.h"
#include "../.pio/libdeps/uno/RobotIRremote/src/RobotIRremote.h"
#include "../.pio/libdeps/uno/TimerEvent/src/TimerEvent.h"

/***************** Declare all the functions *****************/
void Car_front();
void Car_back();
void Car_left();
void Car_right();
void Car_Stop();
void Car_T_left();
void Car_T_right();

void gyroFunc();
void compass();
float checkdistance();
void dance();
void avoid();
void light_track();
void IIC_start();
void IIC_send(unsigned char send_data);
void IIC_end();
void matrix_display(unsigned char matrix_value[]);
void pestoMatrix();
void timerOneFunc();

/***************** Make DotMatric Images *****************/
// Array, used to store the data of the pattern
unsigned char STOP01[] = {0x2E,0x2A,0x3A,0x00,0x02,0x3E,0x02,0x00,0x3E,0x22,0x3E,0x00,0x3E,0x0A,0x0E,0x00};
unsigned char hou[] =    {0x00,0x7f,0x08,0x08,0x7f,0x00,0x3c,0x42,0x42,0x3c,0x00,0x3e,0x40,0x40,0x3e,0x00};
unsigned char op[] =     {0x00,0x00,0x3c,0x42,0x42,0x3c,0x00,0x7e,0x12,0x12,0x0c,0x00,0x00,0x5e,0x00,0x00};
unsigned char met[] =    {0xf8,0x0c,0xf8,0x0c,0xf8,0x00,0x78,0xa8,0xa8,0xb8,0x00,0x08,0x08,0xf8,0x08,0x08};
unsigned char pesto[] =  {0xfe,0x12,0x12,0x7c,0xb0,0xb0,0x80,0xb8,0xa8,0xe8,0x08,0xf8,0x08,0xe8,0x90,0xe0};
unsigned char bleh[] =   {0x00,0x11,0x0a,0x04,0x8a,0x51,0x40,0x40,0x40,0x40,0x51,0x8a,0x04,0x0a,0x11,0x00};
unsigned char clear[] =  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

/***************** Set timer period for function  *****************/
const unsigned int timerOnePeriod = 1000;
TimerEvent timerOne;

/***************** Setup LCD & Make icon images *****************/
byte Heart[8] = { 0b00000, 0b01010, 0b11111, 0b11111, 0b01110, 0b00100, 0b00000, 0b00000};
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x3F for a 16 chars and 2 line display

#define light_L_Pin A0 // define the pin of left photo resistor sensor
#define light_R_Pin A1 // define the pin of right photo resistor sensor
#define IR_Pin      A2

#define Rem_OK 0xFF02FD // Set al remote buttons
#define Rem_U  0xFF629D
#define Rem_D  0xFFA857
#define Rem_L  0xFF22DD
#define Rem_R  0xFFC23D

#define Rem_1  0xFF6897
#define Rem_2  0xFF9867
#define Rem_3  0xFFB04F
#define Rem_4  0xFF30CF
#define Rem_5  0xFF18E7
#define Rem_6  0xFF7A85
#define Rem_7  0xFF10EF
#define Rem_8  0xFF38C7
#define Rem_9  0xFF5AA5
#define Rem_0  0xFF4AB5
#define Rem_x  0xFF42BD
#define Rem_y  0xFF52AD

#define Trig 6  // ultrasonic trig Pin
#define Echo 7  // ultrasonic echo Pin
#define Led  2

#define matrixData  4  // Set data  pin to 4
#define matrixClock 5  // Set clock pin to 5
#define SCL_Pin  matrixClock  //Set clock pin to A5
#define SDA_Pin  matrixData  //Set data pin to A4

#define servoPinXY 0  // servo Pin horizontal
#define servoPinZ  1  // servo Pin vertical

#define ML_PWM     11  // define PWM control pin of left motor
#define MR_Ctrl    12  // define the direction control pin of right motor
#define MR_PWM     3   // define PWM control pin of right motor
#define ML_Ctrl    13  // define the direction control pin of left motor

IRrecv IRrecv(IR_Pin); // Set the remote
decode_results results;
long ir_rec, previousIR; // set remote vars
int previousXY, previousZ;

TM1640 module(matrixData, matrixClock); // Set the 16 x 8 dot matrix
TM16xxMatrix matrix(&module, 16, 8);    // TM16xx object, columns, rows
int screen = 0;

Adafruit_MPU6050 mpu; // Set the gyroscope

long random2;     //set random for choice making
float distanceF, distanceR, distanceL; // set var for distance mesure 

int sensorValueR ;        // value read from the R light sensor
int sensorValueL ;        // value read from the L light sensor
int outputValueR ;        // value output to the R PWM (analog out)
int outputValueL ;        // value output to the L PWM (analog out)
int calcValue ;           // inverse input

Servo servoXY; // set horizontal servo
Servo servoZ;  // set vertical servo

int posXY = 90;  // set horizontal servo position
int speedXY = 3;

int posZ = 90;   // set vertical servo position
int speedZ = 3;

int flag; // flag variable, it is used to entry and exist function

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(1337);


/************ the function to run motor **************/
void Car_front()
{
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
    analogWrite(MR_PWM,255);
    digitalWrite(ML_Ctrl,HIGH);
    analogWrite(ML_PWM,180);
}
void Car_T_right()
{
    digitalWrite(MR_Ctrl,HIGH);
    analogWrite(MR_PWM,180);
    digitalWrite(ML_Ctrl,HIGH);
    analogWrite(ML_PWM,255);
}
/************ the gyroscope **************/
void gyroFunc(){
    // Get new sensor events with the readings
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    lcd.clear();
    lcd.setCursor(0,0); // Sets the location at which subsequent text written to the LCD will be displayed
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
    lcd.print(g.gyro.z);
}
/************ the Compass **************/
void compass(){
    /* Get a new sensor event */
    sensors_event_t event;
    mag.getEvent(&event);
    float heading = atan2(event.magnetic.y, event.magnetic.x);
    float declinationAngle = -90;
    heading += declinationAngle;

    if(heading < 0)
        heading += 2*PI;

    if(heading > 2*PI)
        heading -= 2*PI;

    float headingDegrees = heading * 180/M_PI;
    lcd.clear();
    // Print a message on both lines of the LCD.
    lcd.setCursor(0,0);   //Set cursor to character 2 on line 0
    lcd.print("Compass ");
    lcd.print(headingDegrees);
    lcd.setCursor(4,1);
    if (headingDegrees >= 0 && headingDegrees < 45){
        lcd.println("North");
    }
    if (headingDegrees >= 45 && headingDegrees < 135){
        lcd.println("East ");
    }
    if (headingDegrees >= 135 && headingDegrees < 225){
        lcd.println("South");
    }
    if (headingDegrees >= 225 && headingDegrees < 315){
        lcd.println("West ");
    }
    if (headingDegrees >= 315 && headingDegrees < 360){
        lcd.println("North");
    }
}


/************ control ultrasonic sensor **************/
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

/************ arbitrary sequence **************/
void dance() {
    flag = 0; ///the design that enter obstacle avoidance function
    while (flag == 0) {
        for(int i=0; i<16; i++) {
            for(int j=0; j<8; j++) {
                matrix.setPixel(i,j, true);
                delay(10);
                matrix.setPixel(i,j, false);
            }
        }
        for(int i=0; i<8; i++) {         // One pixel, row by row
            for(int j=0; j<16; j++) {
                matrix.setPixel(j,i, true);
                delay(10);
                matrix.setPixel(j,i, false);
            }
        }
        for (int myangle = 0; myangle <= 180; myangle += 1) { // goes from 0 degrees to 180 degrees
            servoXY.write(myangle);              // tell servo to go to position in variable 'myangle'
            delay(15);                   //control the rotation speed of servo
        }
        for (int myangle = 100; myangle >= 0; myangle -= 1) { // goes from 180 degrees to 0 degrees
            servoXY.write(myangle);              // tell servo to go to position in variable 'myangle'
            delay(10);
        }
        if (IRrecv.decode(&results)) {
            ir_rec = results.value;
            IRrecv.resume();
            if (ir_rec == Rem_OK) {
                flag = 1;
            }
        }
    }
}

/***************** Obstacle Avoidance Function **************/
void avoid()
{
    flag = 0; ///the design that enter obstacle avoidance function
    while (flag == 0) {
        random2 = random(1, 100);
        distanceF= checkdistance();
        if (distanceF < 25) {
            analogWrite (Led, 255);
            Car_Stop(); /// robot stops
            servoZ.write(115);
            delay(10); ///delay in 200ms
            servoZ.write(90);
            delay(10); ///delay in 200ms
            analogWrite (Led, 0);
            servoXY.write(160); /// look left
            for (int j = 1; j <= 10; j = j + (1)) { ///  the data will be more accurate if sensor detect a few times.
                distanceL = checkdistance();
            }
            delay(200);
            servoXY.write(20); /// look right
            for (int k = 1; k <= 10; k = k + (1)) {
                distanceR = checkdistance();
            }
            if (distanceL < 50 || distanceR < 50) {
                if (distanceL > distanceR) {
                    servoXY.write(90);
                    Car_left();
                    delay(500); ///turn left 500ms
                    Car_front();
                }
                else {
                    servoXY.write(90);
                    Car_right();
                    delay(500);
                    Car_front();
                }
            } else {  /// not (distanceL < 50 || distanceR < 50)
                if ((long) (random2) % (long) (2) == 0) ///when the random number is even
                {
                    servoXY.write(90);
                    Car_left(); ///robot turns left
                    delay(500);
                    Car_front(); ///go forward
                }
                else
                {
                    servoXY.write(90);
                    Car_right(); ///robot turns right
                    delay(500);
                    Car_front(); ///go forward
                } } }
        else /// if (distanceF < 25) { If the front distance is greater than or equal, robot car will go forward
        {
            Car_front();
        }
        if (IRrecv.decode(&results))
        {
            ir_rec = results.value;
            IRrecv.resume();
            if (ir_rec == Rem_OK) {
                flag = 1;
            }
        }
    }
}

/****************Light Follow******************/
void light_track() {
    flag = 0;
    while (flag == 0) {
        sensorValueL = analogRead(light_L_Pin);
        sensorValueR = analogRead(light_R_Pin);
        if (sensorValueL > 650 && sensorValueR > 650)
        {
            Car_front();
        }
        else if (sensorValueL > 650 && sensorValueR <= 650)
        {
            Car_left();
        }
        else if (sensorValueL <= 650 && sensorValueR > 650)
        {
            Car_right();
        }
        else //other situations, stop
        {
            Car_Stop();
        }
        if (IRrecv.decode(&results))
        {
            ir_rec = results.value;
            IRrecv.resume();
            if (ir_rec == Rem_OK) {
                flag = 1;
            }
        }
    }
}
/********************** the function for dot matrix display ***********************/
void matrix_display(unsigned char matrix_value[])
{
    IIC_start();  // use the function of the data transmission start condition
    IIC_send(0xc0);  //select address

    for(int i = 0;i < 16;i++) //pattern data has 16 bits
    {
        IIC_send(matrix_value[i]); //convey the pattern data
    }

    IIC_end();   //end the transmission of pattern data
    IIC_start();
    IIC_send(0x8A);  //display control, set pulse width to 4/16 s
    IIC_end();
}

//the condition to start conveying data
void IIC_start()
{
    digitalWrite(SCL_Pin,HIGH);
    delayMicroseconds(3);
    digitalWrite(SDA_Pin,HIGH);
    delayMicroseconds(3);
    digitalWrite(SDA_Pin,LOW);
    delayMicroseconds(3);
}
//Convey data
void IIC_send(unsigned char send_data)
{
    for(char i = 0;i < 8;i++)  //Each byte has 8 bits 8bit for every character
    {
        digitalWrite(SCL_Pin,LOW);  // pull down clock pin SCL_Pin to change the signal of SDA
        delayMicroseconds(3);
        if(send_data & 0x01)  //set high and low level of SDA_Pin according to 1 or 0 of every bit
        {
            digitalWrite(SDA_Pin,HIGH);
        }
        else
        {
            digitalWrite(SDA_Pin,LOW);
        }
        delayMicroseconds(3);
        digitalWrite(SCL_Pin,HIGH); //pull up the clock pin SCL_Pin to stop transmission
        delayMicroseconds(3);
        send_data = send_data >> 1;  // detect bit by bit, shift the data to the right by one
    }
}

//The sign of ending data transmission
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
/********************** END of the function for dot matrix display ***********************/

/************ Show matrix images **************/
void pestoMatrix() {

    switch (screen) {
        case 1: matrix_display(STOP01); break;
        case 2: matrix_display(hou);    break;
        case 3: matrix_display(op);     break;
        case 4: matrix_display(met);    break;
        case 5: matrix_display(pesto);  break;
        case 6: matrix_display(bleh);   break;
        default:matrix_display(bleh);
    }
    screen == 6 ? screen = 0 : screen += 1;
}

void timerOneFunc(){
    pestoMatrix();
}
/************ Setup (booting the arduino) **************/
void setup(){
    lcd.init();
    lcd.clear();
    lcd.backlight();      // Make sure backlight is on
    // create a new characters
    lcd.createChar(0, Heart);

    lcd.clear();
    lcd.setCursor(2,0);   //Set cursor to character 2 on line 0
    lcd.print("MPU6050 test!");
    delay(500);

    lcd.setCursor(0,1);
    // Try to initialize!
    if (!mpu.begin()) {
        lcd.println("MPU6050 not found");
        while (1) {
            delay(10);
        }
    }
    lcd.println("MPU6050 Found!");
    lcd.clear();

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    lcd.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange()) {
        case MPU6050_RANGE_2_G:
            lcd.println("+-2G");
            break;
        case MPU6050_RANGE_4_G:
            lcd.println("+-4G");
            break;
        case MPU6050_RANGE_8_G:
            lcd.println("+-8G");
            break;
        case MPU6050_RANGE_16_G:
            lcd.println("+-16G");
            break;
    }
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    lcd.print("Gyro range set to: ");
    switch (mpu.getGyroRange()) {
        case MPU6050_RANGE_250_DEG:
            lcd.println("+- 250 deg/s");
            break;
        case MPU6050_RANGE_500_DEG:
            lcd.println("+- 500 deg/s");
            break;
        case MPU6050_RANGE_1000_DEG:
            lcd.println("+- 1000 deg/s");
            break;
        case MPU6050_RANGE_2000_DEG:
            lcd.println("+- 2000 deg/s");
            break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    lcd.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth()) {
        case MPU6050_BAND_260_HZ:
            lcd.println("260 Hz");
            break;
        case MPU6050_BAND_184_HZ:
            lcd.println("184 Hz");
            break;
        case MPU6050_BAND_94_HZ:
            lcd.println("94 Hz");
            break;
        case MPU6050_BAND_44_HZ:
            lcd.println("44 Hz");
            break;
        case MPU6050_BAND_21_HZ:
            lcd.println("21 Hz");
            break;
        case MPU6050_BAND_10_HZ:
            lcd.println("10 Hz");
            break;
        case MPU6050_BAND_5_HZ:
            lcd.println("5 Hz");
            break;
    }

    delay(1000);
    lcd.clear();
    gyroFunc();
    delay(1000);
    lcd.clear();
    compass();
    delay(1000);

    servoXY.attach(servoPinXY);
    servoZ.attach(servoPinZ);
    servoXY.write(posXY);              // tell servo to go to position in variable 'pos'
    delay(15);
    servoZ.write(posZ);              // tell servo to go to position in variable 'pos'
    delay(15);

    IRrecv.enableIRIn(); // Initialize the IR receiver

    pinMode(Trig, OUTPUT);
    pinMode(Echo, INPUT);
    pinMode(ML_Ctrl, OUTPUT);
    pinMode(ML_PWM, OUTPUT);
    pinMode(MR_Ctrl, OUTPUT);
    pinMode(MR_PWM, OUTPUT);
    pinMode(Led, OUTPUT);

    pinMode(matrixClock,OUTPUT);
    pinMode(matrixData,OUTPUT);
    digitalWrite(matrixClock,LOW);
    digitalWrite(matrixData,LOW);
    matrix_display(clear);
    timerOne.set(timerOnePeriod, timerOneFunc);
    pestoMatrix();
    delay(1000);


    module.clearDisplay();
    for(int i=0; i<16; i++) // One pixel, column by column
    {
        for(int j=0; j<8; j++)
        {
            matrix.setPixel(i,j, true);
            delay(10);
            matrix.setPixel(i,j, false);
        }
    }

    for(int i=0; i<8; i++) // One pixel, row by row
    {
        for(int j=0; j<16; j++)
        {
            matrix.setPixel(j,i, true);
            delay(10);
            matrix.setPixel(j,i, false);
        }
    }
    // Print a message on both lines of the LCD.
    lcd.clear();
    lcd.setCursor(2,0);   //Set cursor to character 2 on line 0
    lcd.print("Hello Pesto!");
    lcd.setCursor(3, 1);
    lcd.write(0); // show custom caracter (heart)
    lcd.setCursor(5, 1);
    lcd.print("Love u");
    lcd.setCursor(12, 1);
    lcd.write(0);
}
/************ Main loop (running the arduino) **************/
void loop(){
    if (IRrecv.decode(&results)) { /// receive the IR remote value
        ir_rec = results.value;
        //lcd.println(results.value, HEX); ///Wrap word in 16 HEX to output and receive code
        IRrecv.resume();


        if (ir_rec == 0xFFFFFFFF) {
            ir_rec = previousIR;
        }
        switch (ir_rec) {
            case Rem_L:
                posXY = min(180, posXY + speedXY);
                break;
            case Rem_R:
                posXY = max(0, posXY - speedXY);
                break;
            case Rem_OK:
                posXY = 90;
                posZ = 90;
                break;
            case Rem_U:
                posZ = min(160, posZ + speedZ);
                break;
            case Rem_D:
                posZ = max(0, posZ - speedZ);
                break;
            case Rem_0:
                avoid();
                break;
            case Rem_1:
                Car_T_left();
                break;
            case Rem_2:
                Car_front();
                break;
            case Rem_3:
                Car_T_right();
                break;
            case Rem_4:
                Car_left();
                break;
            case Rem_5:
                Car_Stop();
                break;
            case Rem_6:
                Car_right();
                break;
            case Rem_7:
                compass();
                break;
            case Rem_8:
                Car_back();
                break;
            case Rem_9:
                gyroFunc();
                break;
            case Rem_x:
                dance();
                break;
            case Rem_y:
                light_track();
                break;
        }
        if (posXY != previousXY) {
            servoXY.write(posXY);
        }
        if (posZ != previousZ) {
            servoZ.write(posZ);
        }
        previousIR = ir_rec;
        previousXY = posXY;
        previousZ = posZ;
    }
    timerOne.update();
    random2 = random(1, 100);
    sensorValueR = analogRead(light_R_Pin);
    sensorValueL = analogRead(light_L_Pin);
    outputValueR = map(sensorValueR, 0, 1023, 0, 255);
    outputValueL = map(sensorValueL, 0, 1023, 0, 255);
    calcValue = 255 - ((outputValueR + outputValueL) * 1.5);
    calcValue = (calcValue < 0) ? 0 : calcValue;
    analogWrite(Led, calcValue);
    distanceF = checkdistance();  /// assign the front distance detected by ultrasonic sensor to variable a
    if (distanceF < 35) {
        analogWrite (Led, 255);
        pestoMatrix();
        delay(distanceF);
    } else {
        analogWrite (Led, 0);
    }
}


