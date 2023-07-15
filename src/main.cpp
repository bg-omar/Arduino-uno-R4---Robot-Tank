// Include the libraries: mklink /J arduino_libraries "C:\Program Files (x86)\Arduino\libraries"
#include <Arduino.h>
#include "../.pio/libdeps/uno/Adafruit MPU6050/Adafruit_MPU6050.h"
#include "../.pio/libdeps/uno/TimerEvent/src/TimerEvent.h"
#include "../.pio/libdeps/uno/DHT sensor library/DHT.h"
#include "../.pio/libdeps/uno/Adafruit Unified Sensor/Adafruit_Sensor.h"
#include "../.pio/libdeps/uno/Adafruit HMC5883 Unified/Adafruit_HMC5883_U.h"
#include "../.pio/libdeps/uno/LiquidCrystal_I2C/LiquidCrystal_I2C.h"
#include "../.pio/libdeps/uno/TM16xx LEDs and Buttons/src/TM16xx.h"
#include "../.pio/libdeps/uno/TM16xx LEDs and Buttons/src/TM1640.h"
#include "../.pio/libdeps/uno/TM16xx LEDs and Buttons/src/TM16xxMatrix.h"
#include "../.pio/libdeps/uno/Servo/src/Servo.h"
#include "../.pio/libdeps/uno/IRremoteTank/IRremoteTank.h"

byte Heart[8] = { 0b00000, 0b01010, 0b11111, 0b11111, 0b01110, 0b00100, 0b00000, 0b00000};

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


#define Trig 6  //ultrasonic trig Pin
#define Echo 7  //ultrasonic echo Pin
#define Led 2

#define matrixData 4  //servo Pin
#define matrixClock 5  //servo Pin
#define DHTPIN 8

#define servoPinXY 0  //servo Pin
#define servoPinZ 1  //servo Pin
#define ML_PWM 11   //define PWM control pin of left motor
#define MR_Ctrl 12  //define the direction control pin of right motor
#define ML_Ctrl 13  //define the direction control pin of left motor
#define MR_PWM 3   //define PWM control pin of right motor

IRrecv irrecv(IR_Pin);
decode_results results;
long ir_rec, duration, cm;

// Initialize DHT sensor for normal 16mhz Arduino:

TM1640 module(matrixData, matrixClock);
TM16xxMatrix matrix(&module, 16, 8);    // TM16xx object, columns, rows
DHT dht = DHT(DHTPIN, DHT11);

Adafruit_MPU6050 mpu;

long random2;
float distanceF;
float distanceR;
float distanceL;


int sensorValueR ;        // value read from the pot
int sensorValueL ;        // value read from the pot
int outputValueR ;        // value output to the PWM (analog out)
int outputValueL ;        // value output to the PWM (analog out)
int calcValue ;      // inverse input

int posXY = 0;
int posZ = 25;



int flag; ///flag variable, it is used to entry and exist function

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(1337);

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

void endall(){
    if (irrecv.decode(&results)) //receive the IR remote value
    {
        ir_rec=results.value;
        if (ir_rec == Rem_5)   //Robot car stops
        {
            Car_Stop();
        }
    }
}

//The function to control servo
void servoXY(int myangle) {
    for (int i = 0; i <= 50; i = i + (1)) {
        auto pulsewidth = myangle * 11 + 500;
        digitalWrite(servoPinXY,HIGH);
        delayMicroseconds(pulsewidth);
        digitalWrite(servoPinXY,LOW);
        delay((20 - pulsewidth / 1000));
    }
}

//The function to control servo
void servoZ(int myangle) {
    for (int i = 0; i <= 50; i = i + (1)) {
        auto pulsewidth = myangle * 11 + 500;
        digitalWrite(servoPinZ,HIGH);
        delayMicroseconds(pulsewidth);
        digitalWrite(servoPinZ,LOW);
        delay((20 - pulsewidth / 1000));
    }
}

void displaySensorDetails()
{
    sensor_t sensor;
    mag.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print  ("Sensor:       "); Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

void timerGyroFunc(){
    // Get new sensor events with the readings
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

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
    endall();
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
    endall();
}

void compass(){
    /* Get a new sensor event */
    sensors_event_t event;
    mag.getEvent(&event);

    /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
    Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

    // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(event.magnetic.y, event.magnetic.x);

    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    float declinationAngle = 0.035;
    heading += declinationAngle;

    // Correct for when signs are reversed.
    if(heading < 0)
        heading += 2*PI;

    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
        heading -= 2*PI;

    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180/M_PI;
    // Print a message on both lines of the LCD.
    lcd.setCursor(0,0);   //Set cursor to character 2 on line 0
    lcd.print("Compass ");
    lcd.print(headingDegrees);
    endall();
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
        for (int myangle = 0; myangle <= 180; myangle += 1) { // goes from 0 degrees to 180 degrees
            // in steps of 1 degree
            servoXY(myangle);              // tell servo to go to position in variable 'myangle'
            delay(15);                   //control the rotation speed of servo

        }
        for (int myangle = 100; myangle >= 0; myangle -= 1) { // goes from 180 degrees to 0 degrees
            servoXY(myangle);              // tell servo to go to position in variable 'myangle'
            delay(10);
        }
        endall();

    }
}

/*****************Obstacle Avoidance Function**************/
void avoid()
{
    flag = 0; ///the design that enter obstacle avoidance function
    while (flag == 0)
    {
        random2 = random(1, 100);
        distanceF= checkdistance(); ///assign the front distance detected by ultrasonic sensor to variable a
        if (distanceF < 20) ///when the front distance detected is less than 20cm
        {
            analogWrite (Led, 255);
            Car_Stop(); /// robot stops
            servoZ(45);
            delay(200); ///delay in 200ms
            servoZ(0);
            delay(200); ///delay in 200ms
            servoXY(160); ///Ultrasonic platform turns left
            for (int j = 1; j <= 10; j = j + (1)) { ///for statement, the data will be more accurate if ultrasonic sensor detect a few times.
                distanceL = checkdistance(); ///assign the left distance detected  by ultrasonic sensor to variable a1
            }
            delay(200);
            servoXY(20); ///Ultrasonic platform turns right
            for (int k = 1; k <= 10; k = k + (1)) {
                distanceR = checkdistance(); ///assign the right distance detected by ultrasonic sensor to variable a2
            }
            if (distanceL < 50 || distanceR < 50)
                ///robot will turn to the longer distance side when left or right distance is less than 50cm.if the left or right
                /// distance is less than 50cm, the robot will turn to the greater distance
            {
                if (distanceL > distanceR) ///left distance is greater than right
                {
                    servoXY(90); ///Ultrasonic platform turns back to right ahead ultrasonic platform turns front
                    Car_left(); ///robot turns left
                    delay(500); ///turn left 500ms
                    Car_front(); ///go forward
                }
                else
                {
                    servoXY(90);
                    Car_right(); ///robot turns right
                    delay(500);
                    Car_front(); ///go forward
                }
            }
            else ///both distance on two side is greater than or equal to 50cm, turn randomly
            {
                if ((long) (random2) % (long) (2) == 0) ///when the random number is even
                {
                    servoXY(90);
                    Car_left(); ///robot turns left
                    delay(500);
                    Car_front(); ///go forward
                }
                else
                {
                    servoXY(90);
                    Car_right(); ///robot turns right
                    delay(500);
                    Car_front(); ///go forward
                } } }
        else ///If the front distance is greater than or equal to 20cm, robot car will go front
        {
            Car_front(); ///go forward
        }
         endall();
    }
}

void setup(){
    Serial.begin(115200);
    pinMode(servoPinXY, OUTPUT);
    pinMode(servoPinZ, OUTPUT);
    servoXY(posXY);              // tell servo to go to position in variable 'pos'
    delay(15);
    servoZ(posZ);              // tell servo to go to position in variable 'pos'
    delay(15);

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

    Serial.println("HMC5883 Magnetometer Test"); Serial.println("");

    /* Initialise the sensor */
    if(!mag.begin())
    {
        /* There was a problem detecting the HMC5883 ... check your connections */
        Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
        while(1);
    }

    /* Display some basic information on this sensor */
    displaySensorDetails();


    dht.begin();
    lcd.init();
    lcd.clear();
    lcd.backlight();      // Make sure backlight is on
    // create a new characters
    lcd.createChar(0, Heart);

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


}


void loop(){
    random2 = random(1, 100);
    sensorValueR = analogRead(light_R_Pin);
    sensorValueL = analogRead(light_L_Pin);
    outputValueR = map(sensorValueR, 0, 1023, 0, 255);
    outputValueL = map(sensorValueL, 0, 1023, 0, 255);
    calcValue = 255 - ((outputValueR + outputValueL) * 1.5);
    calcValue = (calcValue < 0) ? 0 : calcValue;
    distanceF = checkdistance();  //assign the front distance detected by ultrasonic sensor to variable a
    if (distanceF < 20) {
        analogWrite (Led, 255);
    } else {
        analogWrite (Led, 0);
    }
    if (irrecv.decode(&results)) {
        ir_rec = results.value;
        irrecv.resume();

        if (ir_rec == Rem_2) {
            Car_front();
        }
        if (ir_rec == Rem_8) {
            Car_back();
        }
        if (ir_rec == Rem_1) {
            Car_T_left();
        }
        if (ir_rec == Rem_3) {
            Car_T_right();
        }
        if (ir_rec == Rem_5) {
            Car_Stop();
        }
        if (ir_rec == Rem_4) {
            Car_left();
        }
        if (ir_rec == Rem_6) {
            Car_right();
        }
        if (ir_rec == Rem_x) {
            avoid();
        }
        if (ir_rec == Rem_y) {
            dance();
        }
        if (ir_rec == Rem_OK) {
            servoXY(90);
            delay(30);
            servoZ(25);
            delay(30);
        }
        if (ir_rec == Rem_U) //Go forward
        {
            servoZ(0);
            delay(30);
        }
        if (ir_rec == Rem_D)  //Robot car goes back
        {
            servoZ(90);
            delay(30);
        }
        if (ir_rec == Rem_L)   //Robot car turns left
        {
            servoXY(160);
            delay(30);//Servo rotates to 90°
        }
        if (ir_rec == Rem_R)   //Robot car turns right
        {
            servoXY(20);
            delay(30);//Servo rotates to 90°
        }
        if (ir_rec == Rem_7) {
            timerDHTFunc();
        }
        if (ir_rec == Rem_9) {
            timerGyroFunc();
        }
        if (ir_rec == Rem_0) {
            compass();
        }
    }
}


