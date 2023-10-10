/*
UNO BLE -->     DC:54:75:C3:D9:ED   -
PC USB Dongel   00:1F:E2:C8:82:BA
ESP 1           66:CB:3E:E9:02:8A
ESP small cam   3C:E9:0E:88:65:16
PS4 Controller: A4:AE:11:E1:8B:B3 (SONYWA) GooglyEyes
PS5 Controller: 88:03:4C:B5:00:66
*/

/***************************************************************************************************************/
// section define
/***************************************************************************************************************/

#define USE_JOYSTICK 0
#define USE_ADAFRUIT 0
#define USE_U8G2 1
#define SMALL 0
#define DISPLAY_DEMO 1

#define USE_GYRO 1
#define USE_COMPASS 1
#define USE_BAROMETER 0

#define USE_IRREMOTE 0
#define USE_I2C_SCANNER 1
#define USE_PWM 1
#define USE_DOT 1

#define USE_ROBOT 1
#define USE_TIMERS 1
#define USE_TEXTFINDER 0

#define USE_MATRIX 0
#define READ_ESP32 0
#define USE_LCD 1
#define ARDUINO_ARCH_RENESAS_UNO

/***************************************************************************************************************/
// section include
/***************************************************************************************************************/

#include "index.h"
#include "secrets.h"

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <LiquidCrystal_I2C.h>
#include <cstring>
#include <cstdint>
#include <cmath>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_BME280.h>

#include <U8g2lib.h>

#if USE_TIMERS
    uint64_t timerButton;
    #include <TimerEvent.h>
#endif

#if USE_MATRIX
    #include "Arduino_LED_Matrix.h"
    ArduinoLEDMatrix matrix;
#endif

#if USE_ADAFRUIT
    #include <Adafruit_GFX.h>
    #if SMALL
        #include <Adafruit_SSD1306.h>
    #endif
    #include <Adafruit_SH110X.h>
    #define SCREEN_WIDTH 128 // OLED display width, in pixels
    #define SCREEN_HEIGHT 64 // OLED display height, in pixels

    #define PIXEL_BLACK 0   ///< Draw 'off' pixels
    #define PIXEL_WHITE 1   ///< Draw 'on' pixels
    #define PIXEL_INVERSE 2 ///< Invert pixels

    #define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
    #define SCREEN_ADDRESS 0x3C

    #define NUMFLAKES     4 // Number of snowflakes in the animation example

    #define LOGO_HEIGHT   16
    #define LOGO_WIDTH    16
#endif

#if USE_IRREMOTE
    #define IR_SEND_PIN        9
    #define RAW_BUFFER_LENGTH  750
    #define DECODE_NEC
    #include <IRremote.hpp>
    uint64_t ir_rec, previousIR; // set remote vars
#endif

#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define FREQUENCY             50
#define SEALEVELPRESSURE_HPA (1013.25)


#if USE_TEXTFINDER
    #include <TextFinder.h>
    TextFinder finder(Serial1);
#endif

/********************************************** Declare all the functions***************************************/
// section declaration
/***************************************************************************************************************/
void Car_front();
void Car_left();
void Car_right();
void Car_Stop();
void Car_Back();

#if USE_I2C_SCANNER
    void I2CScanner();
#endif

void compass();
int pulseWidth(int);
double checkDistance();



#if USE_ROBOT
	void dance();
	void avoid();
	void light_track();
#endif

#if USE_DOT
    void IIC_start();
    void IIC_send(unsigned char send_data);
    void IIC_end();
    void matrix_display(unsigned char matrix_value[]);
    void pestoMatrix();
#endif

#if USE_TIMERS
	void dotMatrixTimer();
	void sensorTimer();
    const int timerOnePeriod = 1000;
    const int timerTwoPeriod = 250;
    const int timerThreePeriod = 7000;
    const int timerMouthPeriod = 1250;

    boolean timerTwoActive = false;
    boolean timerTreeActive = false;
    unsigned long last_event = 0;
    TimerEvent timerOne;
    TimerEvent timerTwo;
    TimerEvent timerThree;
    TimerEvent timerMouth;
#endif

#if USE_PWM
	void RGBled(int r_val, int g_val, int b_val);
#endif

#if USE_GYRO
    void gyroCalibrate_sensor();
    void gyroDetectMovement();
    void gyroFunc();
    float ax, ay, az, gx, gy, gz, baseAx, baseAy, baseAz, baseGx, baseGy, baseGz, temperature;
    Adafruit_MPU6050 mpu; // Set the gyroscope
#endif

#if USE_COMPASS
    void compass();
    Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
#endif

/********************************************** Make DotMatric Images*******************************************/
// section DotMatrix Images
/***************************************************************************************************************/
#if USE_DOT
	// Array, used to store the data of the pattern
	unsigned char STOP01[] = {0x2E,0x2A,0x3A,0x00,0x02,0x3E,0x02,0x00,0x3E,0x22,0x3E,0x00,0x3E,0x0A,0x0E,0x00};
	unsigned char hou[] =    {0x00,0x7f,0x08,0x08,0x7f,0x00,0x3c,0x42,0x42,0x3c,0x00,0x3e,0x40,0x40,0x3e,0x00};
	unsigned char op[] =     {0x00,0x00,0x3c,0x42,0x42,0x3c,0x00,0x7e,0x12,0x12,0x0c,0x00,0x00,0x5e,0x00,0x00};
	unsigned char met[] =    {0xf8,0x0c,0xf8,0x0c,0xf8,0x00,0x78,0xa8,0xa8,0xb8,0x00,0x08,0x08,0xf8,0x08,0x08};
	unsigned char pesto[] =  {0xfe,0x12,0x12,0x7c,0xb0,0xb0,0x80,0xb8,0xa8,0xe8,0x08,0xf8,0x08,0xe8,0x90,0xe0};
	unsigned char bleh[] =   {0x00,0x11,0x0a,0x04,0x8a,0x51,0x40,0x40,0x40,0x40,0x51,0x8a,0x04,0x0a,0x11,0x00};

	unsigned char north[] =  {0x00,0x00,0x00,0x00,0x00,0x7e,0x04,0x08,0x10,0x20,0x7e,0x00,0x00,0x00,0x00,0x00};
	unsigned char east[]  =  {0x00,0x00,0x00,0x00,0x00,0xfe,0x92,0x92,0x92,0x92,0x82,0x00,0x00,0x00,0x00,0x00};
	unsigned char south[] =  {0x00,0x00,0x00,0x00,0x00,0x9e,0x92,0x92,0x92,0x92,0xf2,0x00,0x00,0x00,0x00,0x00};
	unsigned char west[]  =  {0x00,0x00,0x00,0x3c,0x40,0x40,0x40,0x78,0x78,0x40,0x40,0x40,0x3c,0x00,0x00,0x00};

	unsigned char front[] =  {0x00,0x00,0x00,0x00,0x00,0x24,0x12,0x09,0x12,0x24,0x00,0x00,0x00,0x00,0x00,0x00};
	unsigned char back[] =   {0x00,0x00,0x00,0x00,0x00,0x24,0x48,0x90,0x48,0x24,0x00,0x00,0x00,0x00,0x00,0x00};
	unsigned char left[] =   {0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x28,0x10,0x44,0x28,0x10,0x44,0x28,0x10,0x00};
	unsigned char right[] =  {0x00,0x10,0x28,0x44,0x10,0x28,0x44,0x10,0x28,0x44,0x00,0x00,0x00,0x00,0x00,0x00};

	unsigned char clear[] =  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
#endif

byte Heart[8] = { 0b00000, 0b01010, 0b11111, 0b11111, 0b01110, 0b00100, 0b00000, 0b00000};

#if USE_ADAFRUIT
	static const unsigned char PROGMEM logo_bmp[] =
		{0b00000000, 0b11000000,
		 0b00000001, 0b11000000,
		 0b00000001, 0b11000000,
		 0b00000011, 0b11100000,
		 0b11110011, 0b11100000,
		 0b11111110, 0b11111000,
		 0b01111110, 0b11111111,
		 0b00110011, 0b10011111,
		 0b00011111, 0b11111100,
		 0b00001101, 0b01110000,
		 0b00011011, 0b10100000,
		 0b00111111, 0b11100000,
		 0b00111111, 0b11110000,
		 0b01111100, 0b11110000,
		 0b01110000, 0b01110000,
		 0b00000000, 0b00110000 };
#endif

#if USE_MATRIX
    const uint32_t animation[][4] = {
            {
                    0x0,
                    0x200500,
                    0x20000000,
                    66
            },
            {
                    0x5,
                    0xa80500,
                    0x20000000,
                    66
            },
            {
                    0xd812,
                    0x40880500,
                    0x20000000,
                    66
            },
            {
                    0xd812,
                    0x40880500,
                    0x20000000,
                    66
            },
            {
                    0x5,
                    0xa80500,
                    0x20000000,
                    66
            },
            {
                    0x0,
                    0x200500,
                    0x20000000,
                    66
            },
            {
                    0x0,
                    0x200500,
                    0x20000000,
                    66
            }
    };
#endif

/********************************************** PIN Defines ****************************************************/
// section pin define
/***************************************************************************************************************/

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)


#define Rem_OK  0xBF407F
#define Rem_U   0xB946FF
#define Rem_D   0xEA15FF
#define Rem_L   0xBB44FF
#define Rem_R   0xBC43FF

#define Rem_1   0xE916FF
#define Rem_2   0xE619FE
#define Rem_3   0xF20DFE
#define Rem_4   0xF30CFF
#define Rem_5   0xE718FF
#define Rem_6   0xA15EFD
#define Rem_7   0xF708FF
#define Rem_8   0xE31CFF
#define Rem_9   0xA55AFF
#define Rem_0   0xAD52FF
#define Rem_x   0xBD42FF
#define Rem_y   0xB54ADF
#define IRepeat 0xFFFFFF

#define DPAD_U  1100
#define DPAD_R  1200
#define DPAD_D  1300
#define DPAD_L  1400
#define SQUARE  3100
#define xCROSS  3200
#define CIRCLE  3300
#define TRIANG  3400
#define OPTION  2900
#define xSHARE  2800
#define PSHOME  2500
#define CHARGE  3500
#define XAUDIO  3600
#define URIGHT  1500
#define DRIGHT  1600
#define D_LEFT  1700
#define UPLEFT  1800
#define L1      2100
#define L3      2300
#define R1      2200
#define R3      2400
#define TOUCHPD 2700
#define MIC     3700


#define RX_PIN       0
#define TX_PIN       1
#define LED_PIN      2
#define L_PWM       3   // define PWM control pin of right motor

#define DotDataPIN   4  // Set data  pin to 4
#define DotClockPIN  5  // Set clock pin to 5
#define Trig_PIN     6  // ultrasonic trig Pinz
#define Echo_PIN     7  // ultrasonic echo Pin

#define PIN_9        9
#define PIN_10      10
#define R_PWM      11  // define PWM control pin of left motor
#define L_ROT     12  // define the direction control pin of right motor
#define R_ROT     13  // define the direction control pin of left motor


#define IR_Pin      A2
#define MIC_PIN     A3

#if USE_JOYSTICK
    #define Joystick_X  A0
    #define Joystick_Y  A1
#else
    #define light_L_Pin A0
    #define light_R_Pin A1
#endif


#if USE_PWM
	#define PWM_0        0
	#define PWM_1        1
	#define PWM_2        2
	#define PWM_3        3
	#define PWM_4        4
	#define PWM_5        5
	#define PWM_6        6
	#define PWM_7        7
	#define PWM_8        8
	#define PWM_9        9
	#define PWM_10      10
	#define PWM_11      11
	#define PWM_12      12
	#define PWM_13      13
	#define PWM_14      14
	#define PWM_15      15
	#define PWM_16      16
#endif

/*************************************************** the Global Variables **************************************/
// section Global Variables
/***************************************************************************************************************/

#define THRESHOLD 5
#define top  0 // lcd screen top line
#define bot  1 // lcd screen bottom line



int previousXY, previousZ;
int screen = 0;
int displaySwitch = 1;

long random2, randomXY, randomZ;
double distanceF, distanceR, distanceL;

int R_velocity = 0;
int L_velocity = 0;

long baseSound;
int r,g,b;
int lightSensorL, lightSensorR, outputValueR, outputValueL;  // inverse input
double calcValue;

int posXY = 90;  // set horizontal servo position
int posZ = 45;   // set vertical servo position


int flag; // flag variable, it is used to entry and exist function

const unsigned int MAX_MESSAGE_LENGTH = 30;

int LStickX, LStickY, RStickX, RStickY, L2_TRIG, R2_TRIG;
int16_t lastY = 0;

/***************************************************** the Sensor Assigns **************************************/
// section Sensor Assigns
/***************************************************************************************************************/

#if USE_LCD
    LiquidCrystal_I2C lcd(0x27,16,2);
#endif

#if USE_ADAFRUIT
    #if SMALL
        Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
    #else
        Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
    #endif
#endif

#if USE_U8G2
    #if SMALL
        U8G2_SSD1306_128X64_NONAME_1_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
    #else
        U8G2_SH1106_128X64_NONAME_1_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
    #endif
#endif


#if USE_PWM
    uint8_t servonum = 0;
	Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#endif

#if USE_BAROMETER
	Adafruit_BME280 bme;
#endif

/************************************************** the I2CScanner *********************************************/
// section I2CScanner
/***************************************************************************************************************/
#if USE_I2C_SCANNER
	void I2CScanner() {
		byte error, address;
		int nDevices;

		display.println("I2C Scanning...");
        #if USE_ADAFRUIT
            display.display();
        #endif
		nDevices = 0;
		
		delay(200);
		for(address = 1; address < 127; address++ ) {
			Wire.beginTransmission(address);
			error = Wire.endTransmission();

			if (error == 0) {
				display.print(" 0x");
				if (address<16) {
					display.print("0");
				}
				display.println(address,HEX);
                #if USE_ADAFRUIT
                    display.display();
                #endif
				nDevices++;
				delay(200);
			}
			else if (error==4) {
				display.print("Unknow error at address 0x");
				if (address<16) {
					
					display.print("0");
				}
				display.println(address,HEX);
                #if USE_ADAFRUIT
                    display.display();
                #endif
            }
		}
		delay(20);
		display.print(nDevices);
		delay(20);
		display.println(" devices");
        #if USE_ADAFRUIT
            display.display();
        #endif
		delay(100);
		if (nDevices == 0) {
			display.println("No I2C devices found");
            #if USE_ADAFRUIT
                display.display();
            #endif
		}
	}
#endif
/************************************************** the gyroscope **********************************************/
// section gyroRead
/***************************************************************************************************************/
#if USE_GYRO
    void gyroRead(){
        sensors_event_t a, gyro, temp;
        mpu.getEvent(&a, &gyro, &temp);

        temperature = temp.temperature;
        ax = a.acceleration.x - baseAx;
        ay = a.acceleration.y - baseAy;
        az = a.acceleration.z - baseAz;
        gx = gyro.gyro.x - baseGx;
        gy = gyro.gyro.y - baseGy;
        gz = gyro.gyro.z - baseGz;
    }

    void gyroFunc(){
        gyroRead();
        (ax > 0) ? display.print("+"), display.print(ax) : display.print(ax);
        display.print(" ");
        (ay > 0) ? display.print("+"), display.print(ay) : display.print(ay);
        display.print(" ");
        (az > 0) ? display.print("+"), display.print(az) : display.print(az);
        display.print("   ");
        
        (gx > 0) ? display.print("+"), display.print(gx) : display.print(gx);
        display.print(" ");
        (gy > 0) ? display.print("+"), display.print(gy) : display.print(gy);
        display.print(" ");
        (gz > 0) ? display.print("+"), display.print(gz) : display.print(gz);
        display.print("   ");
    }

    void gyroDetectMovement() {
        #if USE_TIMERS
            gyroRead();
            if(( abs(ax) + abs(ay) + abs(az)) > THRESHOLD){
                timerTwoActive = true;
                timerTreeActive = true;
                timerButton = R1;
            }
            if(( abs(gx) + abs(gy) + abs(gz)) > THRESHOLD){
                timerTwoActive = true;
                timerTreeActive = true;
                timerButton = L1;
            }

        #endif
    }
    void gyroCalibrate_sensor() {
        float totX = 0;
        float totY = 0;
        float totZ = 0;
        float totgX = 0;
        float totgY = 0;
        float totgZ = 0;
        sensors_event_t a, gyro, temp;
        delay(10);
        for (size_t i = 0; i < 10; i++) {
            mpu.getEvent(&a, &gyro, &temp);
            delay(10);
            totX += a.acceleration.x;
            delay(10);
            totY += a.acceleration.y;
            delay(10);
            totZ += a.acceleration.z;
            delay(10);
            totgX += gyro.gyro.x;
            delay(10);
            totgY += gyro.gyro.y;
            delay(10);
            totgZ += gyro.gyro.z;
            delay(10);
        }
        baseAx = totX / 10;
        baseAy = totY / 10;
        baseAz = totZ / 10;
        baseGx = totgX / 10;
        baseGy = totgY / 10;
        baseGz = totgZ / 10;
    }

    void gyroSetup() {
        // Try to initialize!
        if (!mpu.begin()) {
            
            display.print("MPU6050 not found");
            delay(500);

        } else {
            display.print("MPU6050 Found!    ");
            delay(500);
            mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
            mpu.setGyroRange(MPU6050_RANGE_500_DEG);
            mpu.setFilterBandwidth(MPU6050_BAND_260_HZ); /// 5, 10, 21, 44, 94, 184, 260(off)
            gyroCalibrate_sensor();
            delay(500);
        }
    }
#endif
/*************************************************** the Compass ***********************************************/
// section Compass
/***************************************************************************************************************/
#if USE_COMPASS
    double readCompass(){
        display.print("Compass ");
        sensors_event_t event; /// Get a new sensor event */
        mag.getEvent(&event);

        double heading = atan2(event.magnetic.y, event.magnetic.x);
        double declinationAngle = 0.035;
        heading += declinationAngle;

        if(heading < 0) {
            heading += 2 * PI;
        }
        if(heading > 2*PI) {
            heading -= 2 * PI;
        }
        double headingDegrees = (heading * 180/M_PI) - 90;
        return (headingDegrees < 0) ? 360 + headingDegrees : headingDegrees;
    }

    void compass(){
        double headingDegrees = readCompass();
        display.print(headingDegrees);
        if (headingDegrees >= 0 && headingDegrees < 45){
            matrix_display(north);
            display.print("North       ");
        }
        if (headingDegrees >= 45 && headingDegrees < 135){
            matrix_display(east);
            display.print("East        ");
        }
        if (headingDegrees >= 135 && headingDegrees < 225){
            matrix_display(south);
            display.print("South       ");
        }
        if (headingDegrees >= 225 && headingDegrees < 315){
            matrix_display(west);
            display.print("West        ");
        }
        if (headingDegrees >= 315 && headingDegrees < 360){
            matrix_display(north);
            display.print("North       ");
        }
    }

    void compassSetup() {
        /* Initialise the sensor */
        if(!mag.begin()) {
            
            display.print("HMC5883 not found   ");
            delay(500);

        } else {
            
            display.print("HMC5883 Found!     ");
            delay(500);
        }
    }
#endif
/********************************************** control ultrasonic sensor***************************************/
// section BaroMeter
/***************************************************************************************************************/
#if USE_BAROMETER
    void baroSetup() {
        display.clearDisplay();
        /* Initialise the sensor */
        if (!bme.begin(0x76)) {
            
            display.print("BME280,not found!");
            delay(500);
        } else {
            
            display.print("BME280 Found!     ");
            delay(500);
        }
    }

    void baroMeter() {
        display.clearDisplay();
        
        display.print("Temp= ");
        display.print(bme.readTemperature());
        display.print("*C ");

        display.print("P= ");
        display.print(bme.readPressure() / 100.0F);
        display.print("hPa");

        
        display.print("Alt= ");
        display.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
        display.print("m ");

        display.print("H= ");
        display.print(bme.readHumidity());
        display.print("%");
        delay(500);
    }
#endif
/********************************************** control ultrasonic sensor***************************************/
// section UltraSonic
/***************************************************************************************************************/

double checkDistance() {
    digitalWrite(Trig_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig_PIN, LOW);
    double checkDistance = pulseIn(Echo_PIN, HIGH) / 58.00;  //58.20, that is, 2*29.1=58.2
    delay(10);
    return checkDistance;
}


void exitLoop() {
    if (Serial1.available()) {
        static char message[MAX_MESSAGE_LENGTH]; // Create char for serial1 message
        static unsigned int message_pos = 0;
        char inByte = Serial1.read();
        if (inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1)) { // Add the incoming byte to our message
            message[message_pos] = inByte;
            message_pos++;
        } else { // Full message received...
            //message[message_pos] = '\0'; // Add null character to string to end string
            int PS4input = atoi(message);
            if (PS4input == PSHOME)flag = 1;
        }
    }

    #if USE_IRREMOTE
        if (IrReceiver.decode()) {
            ir_rec = IrReceiver.decodedIRData.decodedRawData;
            IrReceiver.resume();
            if (ir_rec == Rem_OK) {
                flag = 1;
            }
        }
    #endif
}

/********************************************** arbitrary sequence *********************************************/
// section Dance
/***************************************************************************************************************/
#if USE_ROBOT
	void dance() {
		randomXY = random(1, 180);
		randomZ = random(1, 160);
		pwm.setPWM(PWM_0, 0, pulseWidth(randomXY));
		delay(500);
		pwm.setPWM(PWM_1, 0, pulseWidth(randomZ));
		delay(500);

		// One pixel, row by row
		randomXY = random(1, 180);
		randomZ = random(1, 160);
		pwm.setPWM(PWM_0, 0, pulseWidth(randomXY));
		delay(500);
		pwm.setPWM(PWM_1, 0, pulseWidth(randomZ));
		delay(500);


		randomXY = random(1, 180);
		randomZ = random(1, 160);
		pwm.setPWM(PWM_0, 0, pulseWidth(randomXY));
		delay(500);
		pwm.setPWM(PWM_1, 0, pulseWidth(randomZ));
		delay(500);
        exitLoop();
	}
#endif
/********************************************** Obstacle Avoidance Function*************************************/
// section Avoid
/***************************************************************************************************************/
#if USE_ROBOT
	void avoid() {
		flag = 0; ///the design that enter obstacle avoidance function
		while (flag == 0) {
            random2 = random(1, 100);
            distanceF = checkDistance();
            if (distanceF < 25) {
                analogWrite(LED_PIN, 255);
                Car_Stop(); /// robot stops
                pwm.setPWM(PWM_1, 0, pulseWidth(115));
                delay(10); ///delay in 200ms
                pwm.setPWM(PWM_1, 0, pulseWidth(90));
                delay(10); ///delay in 200ms
                analogWrite(LED_PIN, 0);
                pwm.setPWM(PWM_0, 0, pulseWidth(160)); /// look left
                for (int j = 1;
                     j <= 10; j = j + (1)) { ///  the data will be more accurate if sensor detect a few times.
                    distanceL = checkDistance();
                }
                delay(200);
                pwm.setPWM(PWM_0, 0, pulseWidth(20)); /// look right
                for (int k = 1; k <= 10; k = k + (1)) {
                    distanceR = checkDistance();
                }
                if (distanceL < 50 || distanceR < 50) {
                    if (distanceL > distanceR) {
                        pwm.setPWM(PWM_0, 0, pulseWidth(90));
                        Car_left();
                        delay(500); ///turn left 500ms
                        Car_front();
                    } else {
                        pwm.setPWM(PWM_0, 0, pulseWidth(90));
                        Car_right();
                        delay(500);
                        Car_front();
                    }
                } else {  /// not (distanceL < 50 || distanceR < 50)
                    if ((long) (random2) % (long) (2) == 0) ///when the random number is even
                    {
                        pwm.setPWM(PWM_0, 0, pulseWidth(90));
                        Car_left(); ///robot turns left
                        delay(500);
                        Car_front(); ///go forward
                    } else {
                        pwm.setPWM(PWM_0, 0, pulseWidth(90));
                        Car_right(); ///robot turns right
                        delay(500);
                        Car_front(); ///go forward
                    }
                }
            } else /// if (distanceF < 25) { If the front distance is greater than or equal, robot car will go forward
            {
                Car_front();
            }
            exitLoop();
        }
	}
#endif
/*************************************************** Light Follow **********************************************/
// section Follow Light
/***************************************************************************************************************/
#if USE_ROBOT
	void light_track() {
		flag = 0;
		while (flag == 0) {
			lightSensorR = analogRead(light_R_Pin);
			lightSensorL = analogRead(light_L_Pin);
			if (lightSensorR > 650 && lightSensorL > 650) {
				Car_front();
			}
			else if (lightSensorR > 650 && lightSensorL <= 650) {
				Car_left();
			}
			else if (lightSensorR <= 650 && lightSensorL > 650) {
				Car_right();
			}
			else {
				Car_Stop();
			}
            exitLoop();
		}
	}
#endif
/********************************************** the function for dot matrix display ****************************/
// section Pesto Matrix
/***************************************************************************************************************/
#if USE_DOT
    void matrix_display(unsigned char matrix_value[]) {
        IIC_start();  // use the function of the data transmission start condition
        IIC_send(0xc0);  //select address

        for(int i = 0;i < 16;i++) { //pattern data has 16 bits
            IIC_send(matrix_value[i]); //convey the pattern data
        }

        IIC_end();   //end the transmission of pattern data
        IIC_start();
        IIC_send(0x8A);  //display control, set pulse width to 4/16 s
        IIC_end();
    }

    //the condition to start conveying data
    void IIC_start() {
        digitalWrite(DotClockPIN,HIGH);
        delayMicroseconds(3);
        digitalWrite(DotDataPIN,HIGH);
        delayMicroseconds(3);
        digitalWrite(DotDataPIN,LOW);
        delayMicroseconds(3);
    }
    //Convey data
    void IIC_send(unsigned char send_data) {
        for(char i = 0;i < 8;i++){  //Each byte has 8 bits 8bit for every character
            digitalWrite(DotClockPIN,LOW);  // pull down clock pin DotClockPIN to change the signal of SDA
            delayMicroseconds(3);
            if(send_data & 0x01){  //set high and low level of DotDataPIN according to 1 or 0 of every bit
                digitalWrite(DotDataPIN,HIGH);
            } else {
                digitalWrite(DotDataPIN,LOW);
            }
            delayMicroseconds(3);
            digitalWrite(DotClockPIN,HIGH); //pull up the clock pin DotClockPIN to stop transmission
            delayMicroseconds(3);
            send_data = send_data >> 1;  // detect bit by bit, shift the data to the right by one
        }
    }

    //The sign of ending data transmission
    void IIC_end() {
        digitalWrite(DotClockPIN,LOW);
        delayMicroseconds(3);
        digitalWrite(DotDataPIN,LOW);
        delayMicroseconds(3);
        digitalWrite(DotClockPIN,HIGH);
        delayMicroseconds(3);
        digitalWrite(DotDataPIN,HIGH);
        delayMicroseconds(3);
    }

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
#endif
/********************************************** the function to run motor **************************************/
// section motor function
/***************************************************************************************************************/

void Car_front(){
    digitalWrite(L_ROT,HIGH);
    analogWrite(L_PWM,200);
    digitalWrite(R_ROT,HIGH);
    analogWrite(R_PWM,200);
    delay(10);
}

void Car_left(){
    digitalWrite(L_ROT,LOW);
    analogWrite(L_PWM,255);
    digitalWrite(R_ROT,HIGH);
    analogWrite(R_PWM,255);
    delay(10);
}
void Car_right(){
    digitalWrite(L_ROT,HIGH);
    analogWrite(L_PWM,255);
    digitalWrite(R_ROT,LOW);
    analogWrite(R_PWM,255);
    delay(10);
}
void Car_Stop(){
    digitalWrite(L_ROT,LOW);
    analogWrite(L_PWM,0);
    digitalWrite(R_ROT,LOW);
    analogWrite(R_PWM,0);
    delay(10);
}

void Car_Back(){
    digitalWrite(L_ROT,LOW);
    analogWrite(L_PWM,200);
    digitalWrite(R_ROT,LOW);
    analogWrite(R_PWM,200);
    delay(10);
}

void joystick(int PS4input) {
    if      (PS4input >= 4000 && PS4input <= 4255){ L2_TRIG = PS4input - 4000; }
    else if (PS4input >= 5000 && PS4input <= 5255){ R2_TRIG = PS4input - 5000; }
    else if (PS4input >= 6000 && PS4input <= 6255){ LStickX = PS4input - 6000; }
    else if (PS4input >= 7000 && PS4input <= 7255){ LStickY = PS4input - 7000; }
    else if (PS4input >= 8000 && PS4input <= 8255){ RStickX = PS4input - 8000; }
    else if (PS4input >= 9000 && PS4input <= 9255){ RStickY = PS4input - 9000; }

    if (L2_TRIG > 1) {
        digitalWrite(R_ROT, LOW);
        digitalWrite(L_ROT, LOW);
        analogWrite(R_PWM, L2_TRIG); // Send PWM signal to motor A
        analogWrite(L_PWM, L2_TRIG); // Send PWM signal to motor B
    }
    if (R2_TRIG > 1) {
        digitalWrite(R_ROT, HIGH);
        digitalWrite(L_ROT, HIGH);
        analogWrite(R_PWM, R2_TRIG); // Send PWM signal to motor A
        analogWrite(L_PWM, R2_TRIG); // Send PWM signal to motor B
    }
    //---------------------------------------------- RIGHT THUMBSTICK
    if (RStickY < 128) {
        int yMapped = map(LStickY, 128, 0, 45, 0);
        pwm.setPWM(PWM_1, 0, pulseWidth(yMapped));
    }
    else if (RStickY > 128) {
        int yMapped = map(LStickY, 128, 255, 45, 70);
        pwm.setPWM(PWM_1, 0, pulseWidth(yMapped));
    }
    if (RStickX < 128) {
        int xMapped = map(LStickX, 128, 0, 90, 160);
        pwm.setPWM(PWM_0, 0, pulseWidth(xMapped));
    } else  if (RStickX > 128) {
        int xMapped = map(LStickX, 128, 255, 90, 20);
        pwm.setPWM(PWM_0, 0, pulseWidth(xMapped));
    }

    //----------------------------------------------- LEFT THUMBSTICK
    if (LStickY < 128) {
        digitalWrite(R_ROT, LOW);
        digitalWrite(L_ROT, LOW);
        int yMapped = map(LStickY, 128, 0, 0, 255);
        L_velocity = L_velocity + yMapped;
        R_velocity = R_velocity + yMapped;
    }
    else if (LStickY > 128) {
        digitalWrite(R_ROT, HIGH);
        digitalWrite(L_ROT, HIGH);
        int yMapped = map(LStickY, 128, 0, 0, 255);
        L_velocity = L_velocity - yMapped;
        R_velocity = R_velocity - yMapped;
    } else { // If joystick stays in middle the motors are not moving
        R_velocity = 0;
        L_velocity = 0;
    }

    if (LStickX < 128) {  // X-axis used for left and right control
        int xMapped = map(LStickX, 128, 0, 0, 255);
        L_velocity = L_velocity - xMapped;
        R_velocity = R_velocity + xMapped;
        if (L_velocity < 0) {  L_velocity = 0;  }
        if (R_velocity > 255) { R_velocity = 255; }
    }
    if (LStickX > 128) {
        int xMapped = map(LStickX, 128, 255, 0, 255);
        L_velocity = L_velocity + xMapped;
        R_velocity = R_velocity - xMapped;
        if (L_velocity > 255) { L_velocity = 255; }
        if (R_velocity < 0) { R_velocity = 0;  }
    }
    if (R_velocity < 100) { R_velocity = 0; }
    if (L_velocity < 100) { L_velocity = 0; }
    analogWrite(R_PWM, R_velocity); // Send PWM signal to motor A
    analogWrite(L_PWM, L_velocity); // Send PWM signal to motor B

}



/********************************************** Light Sensors **************************************/
// section Light Sensors
/***************************************************************************************************************/

double lightSensor(){
#if USE_JOYSTICK
    calcValue = 0;
#else
    lightSensorL = analogRead(light_R_Pin);
    lightSensorR = analogRead(light_L_Pin);
    outputValueR = map(lightSensorL, 0, 1023, 0, 255);
    outputValueL = map(lightSensorR, 0, 1023, 0, 255);
    calcValue = 255 - (outputValueR + outputValueL)*.5;
#endif
    return (calcValue < 0) ? 0 : calcValue;
}


/************************************************ Display Adafruit  *************************************************/
// section Display Adafruit
/***************************************************************************************************************/

#if USE_ADAFRUIT

    void testdrawrect() {
        display.clearDisplay();
        for(int16_t i=0; i<display.height()/2; i+=2) {
            display.drawRect(i, i, display.width()-2*i, display.height()-2*i, PIXEL_WHITE);
            #if USE_ADAFRUIT
    display.display();
#endif // Update screen with each newly-drawn rectangle
            delay(1);
        }
    }

    void testfillrect() {
        display.clearDisplay();
        for(int16_t i=0; i<display.height()/2; i+=3) {
            // The INVERSE color is used so rectangles alternate white/black
            display.fillRect(i, i, display.width()-i*2, display.height()-i*2, PIXEL_INVERSE);
            #if USE_ADAFRUIT
    display.display();
#endif // Update screen with each newly-drawn rectangle
            delay(1);
        }
    }

    void testdrawcircle() {
        display.clearDisplay();
        for(int16_t i=0; i<max(display.width(),display.height())/2; i+=2) {
            display.drawCircle(display.width()/2, display.height()/2, i, PIXEL_WHITE);
            #if USE_ADAFRUIT
    display.display();
#endif
            delay(1);
        }
    }

    void testfillcircle() {
        display.clearDisplay();
        for(int16_t i=max(display.width(),display.height())/2; i>0; i-=3) {
            // The INVERSE color is used so circles alternate white/black
            display.fillCircle(display.width() / 2, display.height() / 2, i, PIXEL_INVERSE);
            #if USE_ADAFRUIT
    display.display();
#endif // Update screen with each newly-drawn circle
            delay(1);
        }
    }

    void testdrawroundrect() {
        display.clearDisplay();
        display.drawRoundRect(0, 0, display.width()-2, display.height()-2,
                              display.height()/8, PIXEL_WHITE);
        #if USE_ADAFRUIT
    display.display();
#endif
        delay(200);
    }

    void testfillroundrect() {
        display.clearDisplay();
        for(int16_t i=0; i<display.height()/2-2; i+=2) {
            // The INVERSE color is used so round-rects alternate white/black
            display.fillRoundRect(i, i, display.width()-2*i, display.height()-2*i,
                                  display.height()/4, PIXEL_INVERSE);
            #if USE_ADAFRUIT
    display.display();
#endif
            delay(1);
        }
    }

    void testdrawtriangle() {
        display.clearDisplay();
        for(int16_t i=0; i<max(display.width(),display.height())/2; i+=5) {
            display.drawTriangle(
                    display.width()/2  , display.height()/2-i,
                    display.width()/2-i, display.height()/2+i,
                    display.width()/2+i, display.height()/2+i, PIXEL_WHITE);
            #if USE_ADAFRUIT
    display.display();
#endif
            delay(1);
        }
    }

    void testfilltriangle() {
        display.clearDisplay();
        for(int16_t i=max(display.width(),display.height())/2; i>0; i-=5) {
            // The INVERSE color is used so triangles alternate white/black
            display.fillTriangle(
                    display.width()/2  , display.height()/2-i,
                    display.width()/2-i, display.height()/2+i,
                    display.width()/2+i, display.height()/2+i, PIXEL_INVERSE);
            #if USE_ADAFRUIT
    display.display();
#endif
            delay(1);
        }
    }

    void testdrawchar() {
        display.clearDisplay();

        display.setTextSize(1);      // Normal 1:1 pixel scale
        display.setTextColor(PIXEL_WHITE); // Draw white text
        display.setCursor(0, 0);     // Start at top-left corner
        display.cp437(true);         // Use full 256 char 'Code Page 437' font

        // Not all the characters will fit on the display. This is normal.
        // Library will draw what it can and the rest will be clipped.
        for(int16_t i=0; i<256; i++) {
            if(i == '\n') display.write(' ');
            else          display.write(i);
        }

        #if USE_ADAFRUIT
    display.display();
#endif
        delay(1);
    }

    void testdrawstyles() {
        display.clearDisplay();

        display.setTextSize(2);             // Normal 1:1 pixel scale
        display.setTextColor(PIXEL_WHITE);        // Draw white text
        display.setCursor(0,0);             // Start at top-left corner
        display.println(F("Hellow, Pesto!"));

        display.setTextColor(PIXEL_BLACK, PIXEL_WHITE); // Draw 'inverse' text
        display.println(1337);

        display.setTextSize(2);             // Draw 2X-scale text
        display.setTextColor(PIXEL_WHITE);
        display.print(F("0x")); display.println(0xDEADBEEF, HEX);

        #if USE_ADAFRUIT
    display.display();
#endif
    }

    void testscrolltext() {
        display.clearDisplay();

        display.setTextSize(2); // Draw 2X-scale text
        display.setTextColor(PIXEL_WHITE);
        display.setCursor(10, 0);
        display.println(F("PESTO?"));
        #if USE_ADAFRUIT
    display.display();
#endif      // Show initial text
        #if SMALL
            // Scroll in various directions, pausing in-between:
            display.startscrollright(0x00, 0x0F);
            display.stopscroll();
            delay(100);
            display.startscrollleft(0x00, 0x0F);
            display.stopscroll();
            delay(100);
            display.startscrolldiagright(0x00, 0x07);
            display.startscrolldiagleft(0x00, 0x07);
            display.stopscroll();
        #endif
    }

    void testdrawbitmap() {
        display.clearDisplay();

        display.drawBitmap(
                (display.width()  - LOGO_WIDTH ) / 2,
                (display.height() - LOGO_HEIGHT) / 2,
                logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
        #if USE_ADAFRUIT
    display.display();
#endif
        delay(100);
    }



#endif


void displayLoop(){
    #if USE_ADAFRUIT
        switch (displaySwitch) {
            case 1:  testdrawrect();       break;
            case 2:  testfillrect();       break;
            case 3:  testdrawcircle();     break;
            case 4:  testfillcircle();     break;
            case 5:  testdrawroundrect();  break;
            case 6:  testfillroundrect();  break;
            case 7:  testdrawtriangle();   break;
            case 8:  testfilltriangle();   break;
            case 9:  testdrawchar();       break;
            case 10: testdrawstyles();     break;
            case 11: testscrolltext();     break;
            case 12: testdrawbitmap();     break;
            default:  display.display();
        }
        displaySwitch == 12 ? displaySwitch = 1 : displaySwitch += 1;
    #endif
}

/***************************** IrReceiver **********************************/
// section Loop IrReceiver
/***************************************************************************/

#if USE_IRREMOTE
void irRemote() {
        if (IrReceiver.decode()) {  // Grab an IR code   At 115200 baud, printing takes 200 ms for NEC protocol and 70 ms for NEC repeat
            if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_WAS_OVERFLOW) {         // Check if the buffer overflowed
                display.clearDisplay();
                
                display.print(F("Try to increase the \"RAW_BUFFER_LENGTH\" value of " STR(RAW_BUFFER_LENGTH) " in " __FILE__));
                delay(100);
            } else {
                display.clearDisplay();
                if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
                    ir_rec = previousIR;
                    
                    display.print(F("?"));
                    delay(100);
                } else {
                    ir_rec = IrReceiver.decodedIRData.decodedRawData;
                }
                
                display.print(ir_rec, HEX);
            }
            IrReceiver.resume();                            // Prepare for the next value

            switch (ir_rec) {
                /****** Head movements    ******/
                case Rem_1: posXY = min(180, posXY + speedXY); break;
                case Rem_2: posZ = min(160, posZ + speedZ); break;
                case Rem_3: posXY = max(0, posXY - speedXY); break;
                case Rem_4: posXY = 90; posZ = 45; break;
                case Rem_5: posZ = max(0, posZ - speedZ); break;
                case Rem_6: posXY = 90; posZ = 15; break;

                    /****** Options & Sensors ******/
                case Rem_7:
                    display.clearDisplay();
                    timerTwoActive = !timerTwoActive;
                    timerTreeActive = false;
                    timerButton = Rem_7;
                    delay(100);
                    break;
                case Rem_8: dance(); break;
                case Rem_9:
                    display.clearDisplay();
                    timerTwoActive = !timerTwoActive;
                    timerTreeActive = false;
                    timerButton = Rem_9;
                    delay(100);
                    break;
                case Rem_x: avoid(); break;
                case Rem_y: light_track(); break;

                    /****** Engine - driving  ******/
                case Rem_OK:
                    Car_Stop(); break;
                case Rem_U:
                    Car_front(); break;
                case Rem_D:
                    Car_Back(); break;
                case Rem_L:
                    Car_left(); break;
                case Rem_R:
                    Car_right(); break;
                default:
                    break;
            }
            if (posXY != previousXY) {
                pwm.setPWM(PWM_0, 0, pulseWidth(posXY));
            }
            if (posZ != previousZ) {
                pwm.setPWM(PWM_1, 0, pulseWidth(posZ));
            }
            previousIR = ir_rec;
            previousXY = posXY;
            previousZ = posZ;
            Serial1.write(" sending PESTO!!!");
        }
    }
#endif

/************************************************ Display u8g2  *************************************************/
// section Display u8g2
/***************************************************************************************************************/

#if USE_U8G2

void u8g2_prepare() {
    display.setFont(u8g2_font_6x10_tf);
    display.setFontRefHeightExtendedText();
    display.setDrawColor(1);
    display.setFontPosTop();
    display.setFontDirection(0);
}

void u8g2_box_title(uint8_t a) {
    display.drawStr( 10+a*2, 5, "U8g2");
    display.drawStr( 10, 20, "GraphicsTest");

    display.drawFrame(0,0,display.getDisplayWidth(),display.getDisplayHeight() );
}

void u8g2_box_frame(uint8_t a) {
    display.drawStr( 0, 0, "drawBox");
    display.drawBox(5,10,20,10);
    display.drawBox(10+a,15,30,7);
    display.drawStr( 0, 30, "drawFrame");
    display.drawFrame(5,10+30,20,10);
    display.drawFrame(10+a,15+30,30,7);
}

void u8g2_disc_circle(uint8_t a) {
    display.drawStr( 0, 0, "drawDisc");
    display.drawDisc(10,18,9);
    display.drawDisc(24+a,16,7);
    display.drawStr( 0, 30, "drawCircle");
    display.drawCircle(10,18+30,9);
    display.drawCircle(24+a,16+30,7);
}

void u8g2_r_frame(uint8_t a) {
    display.drawStr( 0, 0, "drawRFrame/Box");
    display.drawRFrame(5, 10,40,30, a+1);
    display.drawRBox(50, 10,25,40, a+1);
}
void u8g2_string(uint8_t a) {
    display.setFontDirection(0);
    display.drawStr(30+a,31, " 0");
    display.setFontDirection(1);
    display.drawStr(30,31+a, " 90");
    display.setFontDirection(2);
    display.drawStr(30-a,31, " 180");
    display.setFontDirection(3);
    display.drawStr(30,31-a, " 270");
}

void u8g2_line(uint8_t a) {
    display.drawStr( 0, 0, "drawLine");
    display.drawLine(7+a, 10, 40, 55);
    display.drawLine(7+a*2, 10, 60, 55);
    display.drawLine(7+a*3, 10, 80, 55);
    display.drawLine(7+a*4, 10, 100, 55);
}

void u8g2_triangle(uint8_t a) {
    uint16_t offset = a;
    display.drawStr( 0, 0, "drawTriangle");
    display.drawTriangle(14,7, 45,30, 10,40);
    display.drawTriangle(14+offset,7-offset, 45+offset,30-offset, 57+offset,10-offset);
    display.drawTriangle(57+offset*2,10, 45+offset*2,30, 86+offset*2,53);
    display.drawTriangle(10+offset,40+offset, 45+offset,30+offset, 86+offset,53+offset);
}

void u8g2_ascii_1() {
    char s[2] = " ";
    uint8_t x, y;
    display.drawStr( 0, 0, "ASCII page 1");
    for( y = 0; y < 6; y++ ) {
        for( x = 0; x < 16; x++ ) {
            s[0] = y*16 + x + 32;
            display.drawStr(x*7, y*10+10, s);
        }
    }
}

void u8g2_ascii_2() {
    char s[2] = " ";
    uint8_t x, y;
    display.drawStr( 0, 0, "ASCII page 2");
    for( y = 0; y < 6; y++ ) {
        for( x = 0; x < 16; x++ ) {
            s[0] = y*16 + x + 160;
            display.drawStr(x*7, y*10+10, s);
        }
    }
}

void u8g2_extra_page(uint8_t a)
{
    display.drawStr( 0, 0, "Unicode");
    display.setFont(u8g2_font_unifont_t_symbols);
    display.setFontPosTop();
    display.drawUTF8(0, 24, "☀ ☁");
    switch(a) {
        case 0:
        case 1:
        case 2:
        case 3:
            display.drawUTF8(a*3, 36, "☂");
            break;
        case 4:
        case 5:
        case 6:
        case 7:
            display.drawUTF8(a*3, 36, "☔");
            break;
    }
}

void u8g2_xor(uint8_t a) {
    uint8_t i;
    display.drawStr( 0, 0, "XOR");
    display.setFontMode(1);
    display.setDrawColor(2);
    for( i = 0; i < 5; i++)
    {
        display.drawBox(10+i*16, 18 + (i&1)*4, 21,31);
    }
    display.drawStr( 5+a, 19, "XOR XOR XOR XOR");
    display.setDrawColor(0);
    display.drawStr( 5+a, 29, "CLR CLR CLR CLR");
    display.setDrawColor(1);
    display.drawStr( 5+a, 39, "SET SET SET SET");
    display.setFontMode(0);

}

#define cross_width 24
#define cross_height 24
static const unsigned char cross_bits[] U8X8_PROGMEM  = {
        0x00, 0x18, 0x00, 0x00, 0x24, 0x00, 0x00, 0x24, 0x00, 0x00, 0x42, 0x00,
        0x00, 0x42, 0x00, 0x00, 0x42, 0x00, 0x00, 0x81, 0x00, 0x00, 0x81, 0x00,
        0xC0, 0x00, 0x03, 0x38, 0x3C, 0x1C, 0x06, 0x42, 0x60, 0x01, 0x42, 0x80,
        0x01, 0x42, 0x80, 0x06, 0x42, 0x60, 0x38, 0x3C, 0x1C, 0xC0, 0x00, 0x03,
        0x00, 0x81, 0x00, 0x00, 0x81, 0x00, 0x00, 0x42, 0x00, 0x00, 0x42, 0x00,
        0x00, 0x42, 0x00, 0x00, 0x24, 0x00, 0x00, 0x24, 0x00, 0x00, 0x18, 0x00, };

#define cross_fill_width 24
#define cross_fill_height 24
static const unsigned char cross_fill_bits[] U8X8_PROGMEM  = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x18, 0x64, 0x00, 0x26,
        0x84, 0x00, 0x21, 0x08, 0x81, 0x10, 0x08, 0x42, 0x10, 0x10, 0x3C, 0x08,
        0x20, 0x00, 0x04, 0x40, 0x00, 0x02, 0x80, 0x00, 0x01, 0x80, 0x18, 0x01,
        0x80, 0x18, 0x01, 0x80, 0x00, 0x01, 0x40, 0x00, 0x02, 0x20, 0x00, 0x04,
        0x10, 0x3C, 0x08, 0x08, 0x42, 0x10, 0x08, 0x81, 0x10, 0x84, 0x00, 0x21,
        0x64, 0x00, 0x26, 0x18, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };

#define cross_block_width 14
#define cross_block_height 14
static const unsigned char cross_block_bits[] U8X8_PROGMEM  = {
        0xFF, 0x3F, 0x01, 0x20, 0x01, 0x20, 0x01, 0x20, 0x01, 0x20, 0x01, 0x20,
        0xC1, 0x20, 0xC1, 0x20, 0x01, 0x20, 0x01, 0x20, 0x01, 0x20, 0x01, 0x20,
        0x01, 0x20, 0xFF, 0x3F, };

void u8g2_bitmap_overlay(uint8_t a) {
    uint8_t frame_size = 28;

    display.drawStr(0, 0, "Bitmap overlay");

    display.drawStr(0, frame_size + 12, "Solid / transparent");
    display.setBitmapMode(false);
    display.drawFrame(0, 10, frame_size, frame_size);
    display.drawXBMP(2, 12, cross_width, cross_height, cross_bits);
    if(a & 4)
        display.drawXBMP(7, 17, cross_block_width, cross_block_height, cross_block_bits);

    display.setBitmapMode(true);
    display.drawFrame(frame_size + 5, 10, frame_size, frame_size);
    display.drawXBMP(frame_size + 7, 12, cross_width, cross_height, cross_bits);
    if(a & 4)
        display.drawXBMP(frame_size + 12, 17, cross_block_width, cross_block_height, cross_block_bits);
}

void u8g2_bitmap_modes(uint8_t transparent) {
    const uint8_t frame_size = 24;

    display.drawBox(0, frame_size * 0.5, frame_size * 5, frame_size);
    display.drawStr(frame_size * 0.5, 50, "Black");
    display.drawStr(frame_size * 2, 50, "White");
    display.drawStr(frame_size * 3.5, 50, "XOR");

    if(!transparent) {
        display.setBitmapMode(false );
        display.drawStr(0, 0, "Solid bitmap");
    } else {
        display.setBitmapMode(true );
        display.drawStr(0, 0, "Transparent bitmap");
    }
    display.setDrawColor(0);// Black
    display.drawXBMP(frame_size * 0.5, 24, cross_width, cross_height, cross_bits);
    display.setDrawColor(1); // White
    display.drawXBMP(frame_size * 2, 24, cross_width, cross_height, cross_bits);
    display.setDrawColor(2); // XOR
    display.drawXBMP(frame_size * 3.5, 24, cross_width, cross_height, cross_bits);
}


uint8_t draw_state = 0;

void draw() {
    u8g2_prepare();
    switch(draw_state >> 3) {
        case 0: u8g2_box_title(draw_state&7); break;
        case 1: u8g2_box_frame(draw_state&7); break;
        case 2: u8g2_disc_circle(draw_state&7); break;
        case 3: u8g2_r_frame(draw_state&7); break;
        case 4: u8g2_string(draw_state&7); break;
        case 5: u8g2_line(draw_state&7); break;
        case 6: u8g2_triangle(draw_state&7); break;
        case 7: u8g2_ascii_1(); break;
        case 8: u8g2_ascii_2(); break;
        case 9: u8g2_extra_page(draw_state&7); break;
        case 10: u8g2_xor(draw_state&7); break;
        case 11: u8g2_bitmap_modes(0); break;
        case 12: u8g2_bitmap_modes(1); break;
        case 13: u8g2_bitmap_overlay(draw_state&7); break;
    }
}

#endif




/***************************************************** Functions s**********************************************/
// section Timer Functions
/***************************************************************************************************************/

#if USE_TIMERS
	void dotMatrixTimer(){
		#if USE_DOT
			pestoMatrix();
		#endif
	}

	void sensorTimer(){
		#if USE_COMPASS
			if (timerTwoActive && timerButton == L1){
				compass();
			}
		#endif

		#if USE_GYRO
			if (timerTwoActive && timerButton == R1){
				gyroFunc();
			}
		#endif
	}

	void resetTimers(){
		timerTwoActive = false;
		timerTreeActive = false;
		#if USE_ADAFRUIT
            display.clearDisplay();
        #endif
	}

    void mouthTimer(){
        #if USE_ADAFRUIT
            displayLoop();
        #endif
    }
#endif


/***************************************************** Servo PWM Angle s**********************************************/
// section Servo PWM Angle
/***************************************************************************************************************/

int pulseWidth(int angle){  //  pwm.setPWM(PWM_0, 0, pulseWidth(0));
    int pulse_wide, analog_value;
    pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
    return analog_value;
}

/***************************************************** Servo PWM Angle s**********************************************/
// section RGBled
/***************************************************************************************************************/
#if USE_PWM
	void RGBled(int r_val, int g_val, int b_val) {
		pwm.setPWM(PWM_8, 0, (16*b_val<4080) ? 16*b_val : 4080);
		pwm.setPWM(PWM_9, 0, (16*g_val<4080) ? 16*g_val : 4080);
		pwm.setPWM(PWM_10, 0, (16*r_val<4080) ? 16*r_val : 4080);
	}
#endif

void defaultLCD(){
    display.print("Hello Pesto!");
       //Set cursor to line 1
    #if USE_IRREMOTE
        display.print(previousIR, HEX);
    #endif
}

void printLog(const char *text, int size = 1, int16_t x = 0, int16_t y = lastY){
#if USE_ADAFRUIT
    display.setTextWrap(false);
    display.setCursor(x,y);             // Start at top-left corner
    display.setTextSize(size);             // Normal 1:1 pixel scale
    display.setTextColor(PIXEL_WHITE);        // Draw white text
    display.println(text);
    lastY = y + 8;
    #if USE_ADAFRUIT
    display.display();
#endif
#else
    display.drawStr(x, y, (const char *) text);
    display.drawFrame(0,0,display.getDisplayWidth(),display.getDisplayHeight() );
    lastY = y + 8;
#endif


}




/********************************************** Setup booting the arduino **************************************/
// section Setup
/***************************************************************************************************************/
void setup(){
    Wire.begin();

    #if USE_ADAFRUIT
        #if SMALL
            if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
                    }
        #else
            display.begin(SCREEN_ADDRESS, true); // Address 0x3C default
            #if USE_ADAFRUIT
                display.display();
            #endif
            display.clearDisplay();
        #endif
    #else
        display.begin();
        u8g2_prepare();
        display.firstPage();
    #endif

    delay(1);
    Serial.begin(115200); // Initialize the hardware serial port for debugging
    printLog("Serial started");
    delay(1);
    Serial1.begin(115200);
    printLog("Serial1 started");
    delay(1);
    SERIAL_AT.begin(115200);
    printLog("Serial_AT started");
    delay(1);

    #if USE_MATRIX
        matrix.loadSequence(animation);
        matrix.begin();
        //matrix.autoscroll(300);
        matrix.play(true);
    #endif
        
    pinMode(Trig_PIN, OUTPUT);    /***** 6 ******/
    pinMode(Echo_PIN, INPUT);     /***** 7 ******/

    pinMode(R_ROT, OUTPUT);     /***** 13 ******/
    pinMode(R_PWM, OUTPUT);      /***** 11 ******/
    pinMode(L_ROT, OUTPUT);     /***** 12 ******/
    pinMode(L_PWM, OUTPUT);      /***** 3 ******/
    digitalWrite(R_ROT, HIGH);
    digitalWrite(L_ROT, LOW);

    pinMode(LED_PIN, OUTPUT);     /***** 2 ******/
    pinMode(MIC_PIN, INPUT);

    pinMode(DotClockPIN,OUTPUT);/***** 5 ******/
    pinMode(DotDataPIN,OUTPUT); /***** 4 ******/
	digitalWrite(DotClockPIN,LOW);
	digitalWrite(DotDataPIN,LOW);

    baseSound = map(analogRead(MIC_PIN), 0, 1023, 0, 255); /***** A3 ******/

	#if USE_DOT
		matrix_display(clear);
		pestoMatrix();
	#endif

	#if USE_I2C_SCANNER
		I2CScanner();
		delay(500);
	#endif

    #if USE_GYRO
        #if USE_ADAFRUIT
            display.clearDisplay();
        #endif
        
        gyroSetup();
    #endif

    #if USE_COMPASS
        #if USE_ADAFRUIT
            display.clearDisplay();
        #endif
        compassSetup();
    #endif

    #if USE_BAROMETER
        #if USE_ADAFRUIT
        display.clearDisplay();
    #endif
        baroSetup();
    #endif

    #if USE_IRREMOTE
        // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
        IrReceiver.begin(IR_Pin, ENABLE_LED_FEEDBACK);
        timerButton = PSHOME;
        #if USE_ADAFRUIT
            display.clearDisplay();
        #endif
        
        display.print("InfraRed remote");
	#endif

	#if USE_PWM
		pwm.begin();
		pwm.setPWMFreq(FREQUENCY);  // Analog servos run at ~50 Hz updates
		pwm.setPWM(PWM_0, 0, pulseWidth(posXY));
		pwm.setPWM(PWM_1, 0, pulseWidth(posZ));
		delay(500);
	#endif

    #if USE_TIMERS
        timerOne.set(timerOnePeriod, dotMatrixTimer);
        timerTwo.set(timerTwoPeriod, sensorTimer);
        timerThree.set(timerThreePeriod, resetTimers);
        #if DISPLAY_DEMO
            timerMouth.set(timerMouthPeriod, mouthTimer);
        #endif
    #endif
    #if USE_ADAFRUIT
        display.clearDisplay();
    #endif
}

/*********************************** Loop **********************************/
// section Loop
/***************************************************************************/
unsigned long t = 0;

void loop(){
    #if USE_U8G2
        #if DISPLAY_DEMO
            display.firstPage();
            do {
                draw();
            } while( display.nextPage() );

            // increase the state
            draw_state++;
            if ( draw_state >= 14*8 )
                draw_state = 0;
        #endif
    #endif
    #if READ_ESP32
        // Read messages from Arduino R4 ESP32
        if (SERIAL_AT.available()) {
            display.print("ESP32 says: ");
            while (SERIAL_AT.available()) {
                Serial.write(SERIAL_AT.read());
                display.println(SERIAL_AT.read());
            }
        }
    #endif

    while (Serial1.available() > 0) {
        static char message[MAX_MESSAGE_LENGTH]; // Create char for serial1 message
        static unsigned int message_pos = 0;

        char inByte = Serial1.read();
        if (inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1)) { // Add the incoming byte to our message
            message[message_pos] = inByte;
            message_pos++;
        } else { // Full message received...
            message[message_pos] = '\0'; // Add null character to string to end string
            // Use the message
            Serial.println(message);
            display.println(message);

            //Or convert to integer and print
            int PS4input = atoi(message);
            if (PS4input > 4000){joystick(PS4input);}
            else{
                switch (PS4input) {
                    //**** Head movements    ****
                    case DPAD_U: pwm.setPWM(PWM_1, 0, pulseWidth(posZ+1));  break;
                    case DPAD_R: pwm.setPWM(PWM_0, 0, pulseWidth(posXY-1));  break;
                    case DPAD_D: pwm.setPWM(PWM_1, 0, pulseWidth(posZ-1));    break;
                    case DPAD_L: pwm.setPWM(PWM_0, 0, pulseWidth(posXY+1)); break;

                    case SQUARE: Car_left();   break;
                    case TRIANG: Car_front();  break;
                    case xCROSS: Car_Back();   break;
                    case CIRCLE: Car_right();  break;


                    case 3101:
                    case 3401:
                    case 3201:
                    case 3301:
                        Car_Stop(); break;


                    case xSHARE: posXY = 90; posZ = 45;  break;
                    case OPTION: posXY = 90; posZ = 15; break;
                    case L1:
                        #if USE_ADAFRUIT
                            display.clearDisplay();
                        #endif
                        timerTwoActive = !timerTwoActive;
                        timerTreeActive = false;
                        timerButton = L1;
                        delay(100);
                        break;
                    case TOUCHPD: dance(); break;
                    case R1:
                        #if USE_ADAFRUIT
                            display.clearDisplay();
                        #endif
                        timerTwoActive = !timerTwoActive;
                        timerTreeActive = false;
                        timerButton = R1;
                        delay(100);
                        break;
                    case L3: avoid(); break;
                    case R3: light_track(); break;
                    /*


                    CHARGE  3500
                    XAUDIO  3600

                    MIC     3700
                    PS4_Battery        3900 + Battery



                    */
                    default:
                        break;
                }
            }
            message_pos = 0; //Reset next message
        }
    }

    
    // React on messages from ESP32-CAM AI-Thinker
    while(Serial1.available()) {
        int c = Serial1.read();
        Serial.write(c);


    }

    #if USE_IRREMOTE
        irRemote();
	#endif

    //Car_Stop();
    distanceF = checkDistance();  /// assign the front distance detected by ultrasonic sensor to variable a
    if (distanceF < 35) {
        #if USE_PWM
            RGBled(230, 0, 0);
        #endif
        #if USE_DOT
            pestoMatrix();
        #endif
        double deltime = distanceF*3;
        delay(deltime);
    } else {
        int micStatus = analogRead(MIC_PIN);
        int mic255 = map(micStatus, 0, 1023, 0, 255);

        if (mic255 > baseSound) {
	#if USE_PWM
				RGBled(mic255, 0, mic255);
	#endif
		} else {
		#if USE_PWM
			RGBled(0, mic255, 0);
		#endif
		}
        //defaultLCD();
		#if USE_PWM
			RGBled(0, 0, lightSensor());
		#endif
	}
#if USE_GYRO
    gyroDetectMovement();
#endif

#if USE_TIMERS
    timerOne.update();
    timerTwo.update();
    timerThree.update();
    #if DISPLAY_DEMO
        timerMouth.update();
    #endif
#endif
}


