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

//#define USE_JOYSTICK
//#define USE_WIFI_SERVER
#define USE_MOUTH_DISPLAY_ADAFRUIT
//#define USE_MOUTH_DISPLAY_U8G2
#define USE_SMALL_DISPLAY
#define USE_MOUTH_DISPLAY

#define USE_GYRO
#define USE_COMPASS
#define USE_BAROMETER

#define USE_IRREMOTE
#define USE_I2C_SCANNER
#define USE_PWM
#define USE_DOT

#define USE_ROBOT
#define USE_TIMERS
//#define USE_TEXTFINDER

//#define USE_MATRIX
#define USE_LCD
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
#include <TimerEvent.h>


#ifdef USE_MATRIX
    #include "Arduino_LED_Matrix.h"
    ArduinoLEDMatrix matrix;
#endif

#ifdef USE_MOUTH_DISPLAY_ADAFRUIT
    #include <Adafruit_GFX.h>
    #include <Adafruit_SSD1306.h>
    #include <Adafruit_SH110X.h>
    #define SCREEN_WIDTH 128 // OLED display width, in pixels
    #define SCREEN_HEIGHT 64 // OLED display height, in pixels

    #define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
    #define SCREEN_ADDRESS 0x3C

    #define NUMFLAKES     4 // Number of snowflakes in the animation example

    #define LOGO_HEIGHT   16
    #define LOGO_WIDTH    16
#endif

#ifdef USE_IRREMOTE
    #define RAW_BUFFER_LENGTH  750
    #define DECODE_NEC
    #include <IRremote.hpp>
    uint64_t ir_rec, previousIR, timerButton; // set remote vars
#endif

#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define FREQUENCY             50
#define SEALEVELPRESSURE_HPA (1013.25)


#ifdef USE_TEXTFINDER
    #include <TextFinder.h>
    TextFinder finder(Serial1);
#endif

const int NUMBER_OF_FIELDS = 1; // how many comma seperated fields we expect
int values[NUMBER_OF_FIELDS];   // array holding values for all the fields
int fieldIndex = 0;            // the current field being received


/********************************************** Declare all the functions***************************************/
// section declaration
/***************************************************************************************************************/
void Car_front();
void Car_left();
void Car_right();
void Car_Stop();
void Car_Back();

#ifdef USE_I2C_SCANNER
    void I2CScanner();
#endif

void compass();
int pulseWidth(int);
double checkDistance();



#ifdef USE_ROBOT
	void dance();
	void avoid();
	void light_track();
#endif

#ifdef USE_DOT
    void IIC_start();
    void IIC_send(unsigned char send_data);
    void IIC_end();
    void matrix_display(unsigned char matrix_value[]);
    void pestoMatrix();
#endif

#ifdef USE_TIMERS
	void dotMatrixTimer();
	void sensorTimer();
    const int timerOnePeriod = 1000;
    const int timerTwoPeriod = 250;
    const int timerThreePeriod = 7000;
    const int timerMouthPeriod = 5000;

    boolean timerTwoActive = false;
    boolean timerTreeActive = false;
    unsigned long last_event = 0;
    TimerEvent timerOne;
    TimerEvent timerTwo;
    TimerEvent timerThree;
    TimerEvent timerMouth;
#endif

#ifdef USE_PWM
	void RGBled(int r_val, int g_val, int b_val);
#endif

#ifdef USE_GYRO
    void gyroCalibrate_sensor();
    void gyroDetectMovement();
    void gyroFunc();
    float ax, ay, az, gx, gy, gz, baseAx, baseAy, baseAz, baseGx, baseGy, baseGz, temperature;
    Adafruit_MPU6050 mpu; // Set the gyroscope
#endif

#ifdef USE_COMPASS
    void compass();
    Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
#endif

/********************************************** Make DotMatric Images*******************************************/
// section DotMatrix Images
/***************************************************************************************************************/
#ifdef USE_DOT
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

#ifdef USE_MOUTH_DISPLAY_ADAFRUIT
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

#ifdef USE_MATRIX
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


#define RX_PIN       0
#define TX_PIN       1
#define LED_PIN      2
#define MR_PWM       3   // define PWM control pin of right motor

#define DotDataPIN   4  // Set data  pin to 4
#define DotClockPIN  5  // Set clock pin to 5
#define Trig_PIN     6  // ultrasonic trig Pinz
#define Echo_PIN     7  // ultrasonic echo Pin

#define PIN_9        9
#define PIN_10      10
#define ML_PWM      11  // define PWM control pin of left motor
#define MR_Ctrl     12  // define the direction control pin of right motor
#define ML_Ctrl     13  // define the direction control pin of left motor


#define IR_Pin      A2
#define MIC_PIN     A3

#ifdef USE_JOYSTICK
    #define Joystick_X  A0
    #define Joystick_Y  A1
#else
    #define light_L_Pin A0
    #define light_R_Pin A1
#endif


#ifdef USE_PWM
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
int speedXY = 20;

int posZ = 45;   // set vertical servo position
int speedZ =  20;

int flag; // flag variable, it is used to entry and exist function

/***************************************************** the Sensor Assigns **************************************/
// section Sensor Assigns
/***************************************************************************************************************/

#ifdef  USE_LCD
    LiquidCrystal_I2C lcd(0x27,16,2);
#endif

#ifdef USE_MOUTH_DISPLAY_ADAFRUIT
    #ifdef USE_SMALL_DISPLAY
        Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
    #else
        Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
    #endif
#endif

#ifdef USE_MOUTH_DISPLAY_U8G2
    U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#endif


#ifdef USE_PWM
    uint8_t servonum = 0;
	Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#endif

#ifdef USE_BAROMETER
	Adafruit_BME280 bme;
#endif

#ifdef USE_SOFTWARESERIAL
    #include <SoftwareSerial.h>
    SoftwareSerial mySerial(PIN_9, PIN_10); // RX, TX pins for SoftwareSerial
#endif

/************************************************** the I2CScanner *********************************************/
// section I2CScanner
/***************************************************************************************************************/
#ifdef USE_I2C_SCANNER
	void I2CScanner() {
		byte error, address;
		int nDevices;
		lcd.clear();
		lcd.setCursor(0, top);
		lcd.print("I2C Scanning...");
		nDevices = 0;
		lcd.setCursor(0,bot);
		delay(200);
		lcd.clear();
		lcd.setCursor(0, top);
		for(address = 1; address < 127; address++ ) {
			Wire.beginTransmission(address);
			error = Wire.endTransmission();

			if (error == 0) {
				lcd.print(" 0x");
				if (address<16) {
					lcd.print("0");
				}
				lcd.print(address,HEX);
				nDevices++;
				delay(200);
			}
			else if (error==4) {
				lcd.clear();
				lcd.setCursor(0, top);
				lcd.print("Unknow error at address 0x");
				if (address<16) {
					lcd.setCursor(0,bot);
					lcd.print("0");
				}
				lcd.setCursor(0,bot);
				lcd.print(address,HEX);
			}
		}
		delay(20);
		lcd.clear();
		delay(20);
		lcd.setCursor(0, top);
		lcd.print(nDevices);
		delay(20);
		lcd.print(" devices");
		delay(100);
		if (nDevices == 0) {
			lcd.clear();
			lcd.setCursor(0, top);
			lcd.print("No I2C devices found");
		}
	}
#endif
/************************************************** the gyroscope **********************************************/
// section gyroRead
/***************************************************************************************************************/
#ifdef USE_GYRO
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
        lcd.setCursor(0, top); // Sets the location at which subsequent text written to the LCD will be displayed
        (ax > 0) ? lcd.print("+"), lcd.print(ax) : lcd.print(ax);
        lcd.print(" ");
        (ay > 0) ? lcd.print("+"), lcd.print(ay) : lcd.print(ay);
        lcd.print(" ");
        (az > 0) ? lcd.print("+"), lcd.print(az) : lcd.print(az);
        lcd.print("   ");
        lcd.setCursor(0,bot);
        (gx > 0) ? lcd.print("+"), lcd.print(gx) : lcd.print(gx);
        lcd.print(" ");
        (gy > 0) ? lcd.print("+"), lcd.print(gy) : lcd.print(gy);
        lcd.print(" ");
        (gz > 0) ? lcd.print("+"), lcd.print(gz) : lcd.print(gz);
        lcd.print("   ");
    }

    void gyroDetectMovement() {
        #ifdef USE_TIMERS
            gyroRead();
            if(( abs(ax) + abs(ay) + abs(az)) > THRESHOLD){
                timerTwoActive = true;
                timerTreeActive = true;
                timerButton = Rem_9;
            }
            if(( abs(gx) + abs(gy) + abs(gz)) > THRESHOLD){
                timerTwoActive = true;
                timerTreeActive = true;
                timerButton = Rem_7;
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
            lcd.setCursor(0,bot);
            lcd.print("MPU6050 not found");
            delay(500);

        } else {
            lcd.setCursor(2,bot);
            lcd.print("MPU6050 Found!    ");
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
#ifdef USE_COMPASS
    double readCompass(){
        lcd.setCursor(0, top);   //Set cursor to character 2 on line 0
        lcd.print("Compass ");
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
        lcd.print(headingDegrees);
        if (headingDegrees >= 0 && headingDegrees < 45){
            matrix_display(north);
            lcd.setCursor(4,bot);
            lcd.print("North       ");
        }
        if (headingDegrees >= 45 && headingDegrees < 135){
            matrix_display(east);
            lcd.setCursor(4,bot);
            lcd.print("East        ");
        }
        if (headingDegrees >= 135 && headingDegrees < 225){
            matrix_display(south);
            lcd.setCursor(4,bot);
            lcd.print("South       ");
        }
        if (headingDegrees >= 225 && headingDegrees < 315){
            matrix_display(west);
            lcd.setCursor(4,bot);
            lcd.print("West        ");
        }
        if (headingDegrees >= 315 && headingDegrees < 360){
            matrix_display(north);
            lcd.setCursor(4,bot);
            lcd.print("North       ");
        }
    }

    void compassSetup() {
        /* Initialise the sensor */
        if(!mag.begin()) {
            lcd.setCursor(0,top);
            lcd.print("HMC5883 not found   ");
            delay(500);

        } else {
            lcd.setCursor(0,top);
            lcd.print("HMC5883 Found!     ");
            delay(500);
        }
    }
#endif
/********************************************** control ultrasonic sensor***************************************/
// section BaroMeter
/***************************************************************************************************************/
#ifdef USE_BAROMETER
    void baroSetup() {
        lcd.clear();
        /* Initialise the sensor */
        if (!bme.begin(0x76)) {
            lcd.setCursor(0,top);
            lcd.print("BME280,not found!");
            delay(500);
        } else {
            lcd.setCursor(0,top);
            lcd.print("BME280 Found!     ");
            delay(500);
        }
    }

    void baroMeter() {
        lcd.clear();
        lcd.setCursor(0,top);
        lcd.print("Temp= ");
        lcd.print(bme.readTemperature());
        lcd.print("*C ");

        lcd.print("P= ");
        lcd.print(bme.readPressure() / 100.0F);
        lcd.print("hPa");

        lcd.setCursor(0,bot);
        lcd.print("Alt= ");
        lcd.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
        lcd.print("m ");

        lcd.print("H= ");
        lcd.print(bme.readHumidity());
        lcd.print("%");
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

/********************************************** arbitrary sequence *********************************************/
// section Dance
/***************************************************************************************************************/
#ifdef USE_ROBOT
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
		#ifdef USE_IRREMOTE
			if (IrReceiver.decode()) {
				ir_rec = IrReceiver.decodedIRData.decodedRawData;
				IrReceiver.resume();
				if (ir_rec == Rem_OK) {
					flag = 1;
				}
			}
		#endif
	}
#endif
/********************************************** Obstacle Avoidance Function*************************************/
// section Avoid
/***************************************************************************************************************/
#ifdef USE_ROBOT
	void avoid() {
		flag = 0; ///the design that enter obstacle avoidance function
		while (flag == 0) {
			random2 = random(1, 100);
			distanceF= checkDistance();
			if (distanceF < 25) {
				analogWrite (LED_PIN, 255);
				Car_Stop(); /// robot stops
				pwm.setPWM(PWM_1, 0, pulseWidth(115));
				delay(10); ///delay in 200ms
				pwm.setPWM(PWM_1, 0, pulseWidth(90));
				delay(10); ///delay in 200ms
				analogWrite (LED_PIN, 0);
				pwm.setPWM(PWM_0, 0, pulseWidth(160)); /// look left
				for (int j = 1; j <= 10; j = j + (1)) { ///  the data will be more accurate if sensor detect a few times.
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
					}
					else {
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
					}
					else
					{
						pwm.setPWM(PWM_0, 0, pulseWidth(90));
						Car_right(); ///robot turns right
						delay(500);
						Car_front(); ///go forward
					} } }
			else /// if (distanceF < 25) { If the front distance is greater than or equal, robot car will go forward
			{
				Car_front();
			}
		#ifdef USE_IRREMOTE
				if (IrReceiver.decode()) {
					ir_rec = IrReceiver.decodedIRData.decodedRawData;
					IrReceiver.resume();
					if (ir_rec == Rem_OK) {
						flag = 1;
					}
				}
		#endif
		}
	}
#endif
/*************************************************** Light Follow **********************************************/
// section Follow Light
/***************************************************************************************************************/
#ifdef USE_ROBOT
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
		#ifdef USE_IRREMOTE
				if (IrReceiver.decode()) {
					ir_rec = IrReceiver.decodedIRData.decodedRawData;
					IrReceiver.resume();
					if (ir_rec == Rem_OK) {
						flag = 1;
					}
				}
		#endif
		}
	}
#endif
/********************************************** the function for dot matrix display ****************************/
// section Pesto Matrix
/***************************************************************************************************************/
#ifdef USE_DOT
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
    digitalWrite(MR_Ctrl,HIGH);
    analogWrite(MR_PWM,200);
    digitalWrite(ML_Ctrl,HIGH);
    analogWrite(ML_PWM,200);
}

void Car_left(){
    digitalWrite(MR_Ctrl,LOW);
    analogWrite(MR_PWM,255);
    digitalWrite(ML_Ctrl,HIGH);
    analogWrite(ML_PWM,255);
}
void Car_right(){
    digitalWrite(MR_Ctrl,HIGH);
    analogWrite(MR_PWM,255);
    digitalWrite(ML_Ctrl,LOW);
    analogWrite(ML_PWM,255);
}
void Car_Stop(){
    digitalWrite(MR_Ctrl,LOW);
    analogWrite(MR_PWM,0);
    digitalWrite(ML_Ctrl,LOW);
    analogWrite(ML_PWM,0);
}

void Car_Back(){
    digitalWrite(MR_Ctrl,LOW);
    analogWrite(MR_PWM,200);
    digitalWrite(ML_Ctrl,LOW);
    analogWrite(ML_PWM,200);
}

/********************************************** Light Sensors **************************************/
// section Light Sensors
/***************************************************************************************************************/

double lightSensor(){
#ifdef USE_JOYSTICK
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

#ifdef USE_MOUTH_DISPLAY_ADAFRUIT
    void testdrawline() {
        int16_t i;

        display.clearDisplay(); // Clear display buffer

        for(i=0; i<display.width(); i+=4) {
            display.drawLine(0, 0, i, display.height()-1, SSD1306_WHITE);
            display.display(); // Update screen with each newly-drawn line
            delay(1);
        }
        for(i=0; i<display.height(); i+=4) {
            display.drawLine(0, 0, display.width()-1, i, SSD1306_WHITE);
            display.display();
            delay(1);
        }


        display.clearDisplay();

        for(i=0; i<display.width(); i+=4) {
            display.drawLine(0, display.height()-1, i, 0, SSD1306_WHITE);
            display.display();
            delay(1);
        }
        for(i=display.height()-1; i>=0; i-=4) {
            display.drawLine(0, display.height()-1, display.width()-1, i, SSD1306_WHITE);
            display.display();
            delay(1);
        }


        display.clearDisplay();

        for(i=display.width()-1; i>=0; i-=4) {
            display.drawLine(display.width()-1, display.height()-1, i, 0, SSD1306_WHITE);
            display.display();
            delay(1);
        }
        for(i=display.height()-1; i>=0; i-=4) {
            display.drawLine(display.width()-1, display.height()-1, 0, i, SSD1306_WHITE);
            display.display();
            delay(1);
        }


        display.clearDisplay();

        for(i=0; i<display.height(); i+=4) {
            display.drawLine(display.width()-1, 0, 0, i, SSD1306_WHITE);
            display.display();
            delay(1);
        }
        for(i=0; i<display.width(); i+=4) {
            display.drawLine(display.width()-1, 0, i, display.height()-1, SSD1306_WHITE);
            display.display();
            delay(1);
        }
    }

    void testdrawrect() {
        display.clearDisplay();

        for(int16_t i=0; i<display.height()/2; i+=2) {
            display.drawRect(i, i, display.width()-2*i, display.height()-2*i, SSD1306_WHITE);
            display.display(); // Update screen with each newly-drawn rectangle
            delay(1);
        }
    }

    void testfillrect() {
        display.clearDisplay();

        for(int16_t i=0; i<display.height()/2; i+=3) {
            // The INVERSE color is used so rectangles alternate white/black
            display.fillRect(i, i, display.width()-i*2, display.height()-i*2, SSD1306_INVERSE);
            display.display(); // Update screen with each newly-drawn rectangle
            delay(1);
        }
    }

    void testdrawcircle() {
        display.clearDisplay();

        for(int16_t i=0; i<max(display.width(),display.height())/2; i+=2) {
            display.drawCircle(display.width()/2, display.height()/2, i, SSD1306_WHITE);
            display.display();
            delay(1);
        }
    }

    void testfillcircle() {
        display.clearDisplay();

        for(int16_t i=max(display.width(),display.height())/2; i>0; i-=3) {
            // The INVERSE color is used so circles alternate white/black
            display.fillCircle(display.width() / 2, display.height() / 2, i, SSD1306_INVERSE);
            display.display(); // Update screen with each newly-drawn circle
            delay(1);
        }
    }

    void testdrawroundrect() {
        display.clearDisplay();
        display.drawRoundRect(0, 0, display.width()-2, display.height()-2,
                              display.height()/8, SSD1306_WHITE);
        display.display();
        delay(200);
    }

    void testfillroundrect() {
        display.clearDisplay();

        for(int16_t i=0; i<display.height()/2-2; i+=2) {
            // The INVERSE color is used so round-rects alternate white/black
            display.fillRoundRect(i, i, display.width()-2*i, display.height()-2*i,
                                  display.height()/4, SSD1306_INVERSE);
            display.display();
            delay(1);
        }
    }

    void testdrawtriangle() {
        display.clearDisplay();

        for(int16_t i=0; i<max(display.width(),display.height())/2; i+=5) {
            display.drawTriangle(
                    display.width()/2  , display.height()/2-i,
                    display.width()/2-i, display.height()/2+i,
                    display.width()/2+i, display.height()/2+i, SSD1306_WHITE);
            display.display();
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
                    display.width()/2+i, display.height()/2+i, SSD1306_INVERSE);
            display.display();
            delay(1);
        }
    }

    void testdrawchar() {
        display.clearDisplay();

        display.setTextSize(1);      // Normal 1:1 pixel scale
        display.setTextColor(SSD1306_WHITE); // Draw white text
        display.setCursor(0, 0);     // Start at top-left corner
        display.cp437(true);         // Use full 256 char 'Code Page 437' font

        // Not all the characters will fit on the display. This is normal.
        // Library will draw what it can and the rest will be clipped.
        for(int16_t i=0; i<256; i++) {
            if(i == '\n') display.write(' ');
            else          display.write(i);
        }

        display.display();
    }

    void testdrawstyles() {
        display.clearDisplay();

        display.setTextSize(2);             // Normal 1:1 pixel scale
        display.setTextColor(SSD1306_WHITE);        // Draw white text
        display.setCursor(0,0);             // Start at top-left corner
        display.println(F("Hellow, Pesto!"));

        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
        display.println(1337);

        display.setTextSize(2);             // Draw 2X-scale text
        display.setTextColor(SSD1306_WHITE);
        display.print(F("0x")); display.println(0xDEADBEEF, HEX);

        display.display();
    }

    void testscrolltext() {
        display.clearDisplay();

        display.setTextSize(2); // Draw 2X-scale text
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(10, 0);
        display.println(F("PESTO?"));
        display.display();      // Show initial text
        #ifdef USE_SMALL_DISPLAY
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
        display.display();
        delay(100);
    }



#endif


void displayLoop(){
    #ifdef USE_MOUTH_DISPLAY_ADAFRUIT
        switch (displaySwitch) {
            case 1:  testdrawline();       break;
            case 2:  testdrawrect();       break;
            case 3:  testfillrect();       break;
            case 4:  testdrawcircle();     break;
            case 5:  testfillcircle();     break;
            case 6:  testdrawroundrect();  break;
            case 7:  testfillroundrect();  break;
            case 8:  testdrawtriangle();   break;
            case 9:  testfilltriangle();   break;
            case 10: testdrawchar();       break;
            case 11: testdrawstyles();     break;
            case 12: testscrolltext();     break;
            case 13: testdrawbitmap();     break;
            default: display.display();
        }
        displaySwitch == 13 ? displaySwitch = 1 : displaySwitch += 1;
    #endif
}

void displaySetup() {
    #ifdef USE_MOUTH_DISPLAY_ADAFRUIT

        if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {

        } else {
            displayLoop();
        }
    #endif
}

/************************************************ Display u8g2  *************************************************/
// section Display u8g2
/***************************************************************************************************************/

#ifdef USE_MOUTH_DISPLAY_U8G2

#endif




/***************************************************** Functions s**********************************************/
// section Timer Functions
/***************************************************************************************************************/

#ifdef USE_TIMERS
	void dotMatrixTimer(){
		#ifdef USE_DOT
			pestoMatrix();
		#endif
	}

	void sensorTimer(){
		#ifdef USE_COMPASS
			if (timerTwoActive && timerButton == Rem_7){
				compass();
			}
		#endif

		#ifdef USE_GYRO
			if (timerTwoActive && timerButton == Rem_9){
				gyroFunc();
			}
		#endif
	}

	void resetTimers(){
		timerTwoActive = false;
		timerTreeActive = false;
		lcd.clear();
	}

    void mouthTimer(){
        #ifdef USE_MOUTH_DISPLAY_ADAFRUIT
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
#ifdef USE_PWM
	void RGBled(int r_val, int g_val, int b_val) {
		pwm.setPWM(PWM_8, 0, (16*b_val<4080) ? 16*b_val : 4080);
		pwm.setPWM(PWM_9, 0, (16*g_val<4080) ? 16*g_val : 4080);
		pwm.setPWM(PWM_10, 0, (16*r_val<4080) ? 16*r_val : 4080);
	}
#endif

void defaultLCD(){
    #ifdef USE_IRREMOTE
        lcd.setCursor(0,top);
        lcd.write(0); // ********* heart ********* //
        lcd.setCursor(2,top);   //Set cursor to character 2 on line 0
        lcd.print("Hello Pesto!");
        lcd.setCursor(0,bot);   //Set cursor to line 1

        lcd.print(previousIR, HEX);
    #endif
}



/********************************************** Setup booting the arduino **************************************/
// section Setup
/***************************************************************************************************************/
void setup(){
    Wire.begin();
    delay(1);
    Serial.begin(115200); // Initialize the hardware serial port for debugging
    delay(1);
    Serial1.begin(115200);
    delay(1);
    SERIAL_AT.begin(115200);
    delay(1);
    //mySerial.begin(115200); // Initialize the software serial port
    delay(1);


    #ifdef USE_MATRIX
        matrix.loadSequence(animation);
        matrix.begin();
        //matrix.autoscroll(300);
        matrix.play(true);
    #endif

    lcd.init();
    delay(10);
    lcd.backlight();      // Make sure backlight is on
    delay(10);
    lcd.createChar(0, Heart); // create a new characters
    delay(10);
    lcd.clear();
    delay(10);
    lcd.setCursor(0,top);
    delay(10);


    pinMode(Trig_PIN, OUTPUT);    /***** 6 ******/
    pinMode(Echo_PIN, INPUT);     /***** 7 ******/

    pinMode(ML_Ctrl, OUTPUT);     /***** 13 ******/
    pinMode(ML_PWM, OUTPUT);      /***** 11 ******/
    pinMode(MR_Ctrl, OUTPUT);     /***** 12 ******/
    pinMode(MR_PWM, OUTPUT);      /***** 3 ******/
    digitalWrite(ML_Ctrl, HIGH);
    digitalWrite(MR_Ctrl, HIGH);

    pinMode(LED_PIN, OUTPUT);     /***** 2 ******/
    pinMode(MIC_PIN, INPUT);

    pinMode(DotClockPIN,OUTPUT);/***** 5 ******/
    pinMode(DotDataPIN,OUTPUT); /***** 4 ******/
	digitalWrite(DotClockPIN,LOW);
	digitalWrite(DotDataPIN,LOW);

    baseSound = map(analogRead(MIC_PIN), 0, 1023, 0, 255); /***** A3 ******/

	#ifdef USE_DOT
		matrix_display(clear);
		pestoMatrix();
	#endif

	#ifdef USE_I2C_SCANNER
		I2CScanner();
		delay(500);
	#endif

    #ifdef USE_GYRO
        lcd.clear();
        lcd.setCursor(0,top);
        gyroSetup();
    #endif

    #ifdef USE_COMPASS
        lcd.clear();
        compassSetup();
    #endif

    #ifdef USE_BAROMETER
        lcd.clear();
        baroSetup();
    #endif

    #ifdef USE_IRREMOTE
        // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
        IrReceiver.begin(IR_Pin, ENABLE_LED_FEEDBACK);
        timerButton = Rem_OK;
        lcd.clear();
        lcd.setCursor(0,top);
        lcd.print("InfraRed remote");
	#endif

    #ifdef USE_MOUTH_DISPLAY_ADAFRUIT
        displaySetup();
    #else
        lcd.clear();
        lcd.setCursor(0,top);
        lcd.print("Not using the");
        lcd.setCursor(0,bot);
        lcd.print(" 128x64 display");
    #endif

	#ifdef USE_PWM
		pwm.begin();
		pwm.setPWMFreq(FREQUENCY);  // Analog servos run at ~50 Hz updates
		pwm.setPWM(PWM_0, 0, pulseWidth(posXY));
		pwm.setPWM(PWM_1, 0, pulseWidth(posZ));
		delay(500);
	#endif

    #ifdef USE_TIMERS
        timerOne.set(timerOnePeriod, dotMatrixTimer);
        timerTwo.set(timerTwoPeriod, sensorTimer);
        timerThree.set(timerThreePeriod, resetTimers);
        timerMouth.set(timerMouthPeriod, mouthTimer);
    #endif
    lcd.clear();
}

/*********************************** Loop **********************************/
// section Loop
/***************************************************************************/

void loop(){

    while (Serial1.available())        // read from Serial1 output to Serial
        Serial.write(Serial1.read());
    while (Serial.available()) {       // read from Serial outut to Serial1
        int inByte = Serial.read();
        //Serial.write(inByte);     // local echo if required
        Serial1.write(inByte);
    }


     // Read messages from Arduino R4 ESP32
//    if (SERIAL_AT.available()) {
//        lcd.print("ESP32 says: ");
//        while (SERIAL_AT.available()) {
//            Serial.write(SERIAL_AT.read());
//            lcd.println(SERIAL_AT.read());
//        }
//    }


/*     // Read messages from ESP32-CAM AI-Thinker
    if(Serial1.available()) {
        char ch = Serial1.read();
        if(ch >= '0' && ch <= '9') {// is this an ascii digit between 0 and 9?
            // yes, accumulate the value
            values[fieldIndex] = (values[fieldIndex] * 10) + (ch - '0');
        } else if (ch == ',') { // comma is our separator, so move on to the next field
            if(fieldIndex < NUMBER_OF_FIELDS-1)
                fieldIndex++;   // increment field index
        } else {
            // any character not a digit or comma ends the acquisition of fields
            // in this example it's the newline character sent by the Serial Monitor
            Serial.print( fieldIndex +1);
            Serial.println(" fields received:");
            lcd.setCursor(8,bot);
            for(int i=0; i <= fieldIndex; i++) {
                lcd.print(values[i]);
                Serial.print(values[i]);
                delay(50);
                values[i] = 0; // set the values to zero, ready for the next message
            }
            fieldIndex = 0;  // ready to start over
        }
    }*/

    #ifdef USE_TEXTFINDER
        // TEXT-finder
        int fieldIndex = 0;            // the current field being received
        finder.find("H");
        while(fieldIndex < NUMBER_OF_FIELDS)
            values[fieldIndex++] = finder.getValue();
        Serial.println("All fields received:");
        for(fieldIndex=0; fieldIndex < NUMBER_OF_FIELDS; fieldIndex++)
            Serial.println(values[fieldIndex]);
    #endif

    lcd.setCursor(0,bot);
    // React on messages from ESP32-CAM AI-Thinker
    while(Serial1.available()){
        delay(5);
        char32_t c = Serial1.read();
        lcd.print(c);

        switch (c) {
            //**** Head movements    ****
            case 1: posXY = min(180, posXY + speedXY); lcd.print("R"); break;//if (PS4.Right()) send(1);
            case 2: posZ = min(160, posZ + speedZ); lcd.print("D"); break; //if (PS4.Down()) send(2);
            case 3: posXY = max(0, posXY - speedXY); lcd.print("U"); break; //if (PS4.Up()) send(3);
            case 4: posXY = 90; posZ = 45; lcd.print("L"); break; //if (PS4.Left()) send(4);

            //if (PS4.UpRight()) send(5);
            //if (PS4.DownRight()) send(6);
            //if (PS4.UpLeft()) send(7);
            //if (PS4.DownLeft()) send(8);

            case 11: Car_left();  lcd.print("S"); break;//if (PS4.Square()) send(11);
            case 12: Car_front(); lcd.print("X"); break;//if (PS4.Cross()) send(12);
            case 13: Car_Back();  lcd.print("O"); break;//if (PS4.Circle()) send(13);
            case 14: Car_right(); lcd.print("T");break;//if (PS4.Triangle()) send(14);
            //if (PS4.L1()) send(21);
            //if (PS4.R1()) send(22);

            //if (PS4.L3()) send(24);
            //if (PS4.R3()) send(25);

            case 26: Car_Stop(); break;//if (PS4.PSButton()) send(26);
            //if (PS4.Touchpad()) send(27);

            case 28: posZ = max(0, posZ - speedZ); break; //if (PS4.Share()) send(28);
            case 29: posXY = 90; posZ = 15; break; //if (PS4.Options()) send(29);

            case 30: lcd.print("Charging"); break;//if (PS4.Charging()) send(30);
            case 31: lcd.print("Headphones"); break;//if (PS4.Audio()) send(31);
            case 32: lcd.print("Mic"); break;//if (PS4.Mic()) send(32);
            case 100: lcd.print("Charging"); break;//if (PS4.Battery()) send(100+ PS4.Battery());

            //if (PS4.L2()) { send(5127 + PS4.L2Value());  }
            //if (PS4.R2()) { send(6127 + PS4.R2Value());  }

            //if (PS4.LStickX() <= -15 || PS4.LStickX() >= 15 ) {
            //    send(1127 + PS4.LStickX());
            //}
            //if (PS4.LStickY() <= -15 || PS4.LStickY() >= 15 ) {
            //    send(2127 + PS4.LStickY());
            //}
            //if (PS4.RStickX() <= -15 || PS4.RStickX() >= 15 ) {
            //    send(3127 + PS4.RStickX());
            //}
            //if (PS4.RStickY() <= -15 || PS4.RStickY() >= 15 ) {
            //    send(4127 + PS4.RStickY());
            //}
            default:
                break;
        }
		#ifdef USE_PWM
				if (posXY != previousXY) {
					pwm.setPWM(PWM_0, 0, pulseWidth(posXY));
				}
				if (posZ != previousZ) {
					pwm.setPWM(PWM_1, 0, pulseWidth(posZ));
				}
		#endif
        previousXY = posXY;
        previousZ = posZ;
    }
    /***************************** IrReceiver **********************************/
    // section Loop IrReceiver
    /***************************************************************************/
    #ifdef USE_IRREMOTE
        if (IrReceiver.decode()) {  // Grab an IR code   At 115200 baud, printing takes 200 ms for NEC protocol and 70 ms for NEC repeat
            if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_WAS_OVERFLOW) {         // Check if the buffer overflowed
                lcd.clear();
                lcd.setCursor(0,top);
                lcd.print(F("Try to increase the \"RAW_BUFFER_LENGTH\" value of " STR(RAW_BUFFER_LENGTH) " in " __FILE__));
                delay(100);
            } else {
                lcd.clear();
                if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
                    ir_rec = previousIR;
                    lcd.setCursor(0,top);
                    lcd.print(F("?"));
                    delay(100);
                } else {
                    ir_rec = IrReceiver.decodedIRData.decodedRawData;
                }
                lcd.setCursor(0,bot);
                lcd.print(ir_rec, HEX);
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
                    lcd.clear();
                    timerTwoActive = !timerTwoActive;
                    timerTreeActive = false;
                    timerButton = Rem_7;
                    delay(100);
                    break;
                case Rem_8: dance(); break;
                case Rem_9:
                    lcd.clear();
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
	#endif

	#ifdef USE_GYRO
		gyroDetectMovement();
    #endif

    #ifdef USE_TIMERS
        timerOne.update();
        timerTwo.update();
        timerThree.update();
        timerMouth.update();
    #endif

    Car_Stop();
    distanceF = checkDistance();  /// assign the front distance detected by ultrasonic sensor to variable a
    if (distanceF < 35) {
        #ifdef USE_PWM
            RGBled(230, 0, 0);
        #endif
        #ifdef USE_DOT
            pestoMatrix();
        #endif
        double deltime = distanceF*3;
        delay(deltime);
    } else {
        int micStatus = analogRead(MIC_PIN);
        int mic255 = map(micStatus, 0, 1023, 0, 255);

        if (mic255 > baseSound) {
	#ifdef USE_PWM
				RGBled(mic255, 0, mic255);
	#endif
		} else {
		#ifdef USE_PWM
			RGBled(0, mic255, 0);
		#endif
		}
        //defaultLCD();
		#ifdef USE_PWM
			RGBled(0, 0, lightSensor());
		#endif
	}
}


