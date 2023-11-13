//
// Created by mr on 10/27/2023.
//

#ifndef MAIN_RA_H
    #define MAIN_RA_H


    /***************************************************************************************************************/
    // section define
    /***************************************************************************************************************/

    #define USE_JOYSTICK 0
    #define USE_ADAFRUIT 1
    #define USE_U8G2 0
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
    #include "pesto_matrix.h"
    #include "follow_light.h"
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




    /********************************************** Declare all the functions***************************************/
    // section declaration
    /***************************************************************************************************************/
    void Car_front();
    void Car_left();
    void Car_right();
    void Car_Stop();
    void Car_Back();
    void exitLoop();

    #if USE_I2C_SCANNER
    void I2CScanner();
    #endif

    void compass();
    int pulseWidth(int);
    double checkDistance();



    #if USE_ROBOT
    void dance();
    void avoid();
    #endif



    #if USE_TIMERS
    void dotMatrixTimer();
    void sensorTimer();
    const int timerOnePeriod = 1000;
    const int timerTwoPeriod = 250;
    const int timerThreePeriod = 7000;
    const int timerMouthPeriod = 1250;

    bool timerTwoActive = false;
    bool timerTreeActive = false;
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
    extern float ax, ay, az, gx, gy, gz, baseAx, baseAy, baseAz, baseGx, baseGy, baseGz, temperature;
    Adafruit_MPU6050 mpu; // Set the gyroscope
    #endif

    #if USE_COMPASS
    void compass();
    Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
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

#endif //MAIN_RA_H
