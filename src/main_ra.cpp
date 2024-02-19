/*
UNO BLE -->     DC:54:75:C3:D9:ED   -
PC USB Dongel   00:1F:E2:C8:82:BA
ESP 1           66:CB:3E:E9:02:8A
ESP small cam   3C:E9:0E:88:65:16
PS4 Controller: A4:AE:11:E1:8B:B3 (SONYWA) GooglyEyes
PS5 Controller: 88:03:4C:B5:00:66
*/


/***************************************************************************************************************/
// section include
/***************************************************************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>


#include <LiquidCrystal_I2C.h>
#include <cstring>
#include <cstdint>
#include <cmath>

#include <Adafruit_Sensor.h>

#include "index.h"
#include "PS4.h"
#include "secrets.h"
#include "main_ra.h"
#include "barometer.h"
#include "animation.h"
#include "pwm_board.h"
#include "timers.h"
#include "compass.h"
#include "gyroscope.h"
#include "IRreceiver.h"
#include "dancing.h"
#include "displayU8G2.h"
#include "displayAdafruit.h"
#include "pesto_matrix.h"
#include "follow_light.h"
#include "motor.h"
#include "avoid_objects.h"
#include "I2Cscanner.h"


#if USE_U8G2
U8G2_SH1106_128X64_NONAME_1_HW_I2C displayU8G2::display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#endif

#if USE_MATRIX
    #include "Arduino_LED_Matrix.h"
    ArduinoLEDMatrix matrix;
#endif

long baseSound;
int r,g,b;


static int flag = 0;

static int posXY = 90;  // set horizontal servo position
static int posZ = 45;   // set vertical servo position
int timers::timerButton;



void main::log(const char *text) {
    #if USE_U8G2
        displayU8G2::U8G2print((const char *) text);
    #endif
    Serial.print((const char *) text);
}
void main::logln(const char *text) {
    #if USE_U8G2
        displayU8G2::U8G2println((const char *) text);
    #endif
    Serial.println((const char *) text);
}

/********************************************** Setup booting the arduino **************************************/
// section Setup
/***************************************************************************************************************/
void setup(){
    unsigned char clear[] =  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    Wire.begin();

    #if USE_ADAFRUIT
        displayAdafruit::setupAdafruit();
    #endif
    #if USE_U8G2
        displayU8G2::U8G2setup();
    #endif

    #if USE_I2C_SCANNER
        I2Cscanner::scan();
        delay(500);
    #endif

    delay(1);
    Serial.begin(115200); // Initialize the hardware serial port for debugging
    main::logln("Serial started");
    delay(1);
    Serial1.begin(115200);
    main::logln("Serial1 started");
    delay(1);
    SERIAL_AT.begin(115200);
    main::logln("Serial_AT started");
    delay(1);

    pinMode(R_ROT, OUTPUT);     /***** 13 ******/
    pinMode(R_PWM, OUTPUT);      /***** 11 ******/
    pinMode(L_ROT, OUTPUT);     /***** 12 ******/
    pinMode(L_PWM, OUTPUT);      /***** 3 ******/
    digitalWrite(R_ROT, HIGH);
    digitalWrite(L_ROT, HIGH);
    main::logln("Motor initialized");

    pinMode(LED_PIN, OUTPUT);     /***** 2 ******/

    #if USE_SWITCH
        pinMode(SWITCH_8, INPUT_PULLUP);     /***** 2 ******/
        pinMode(SWITCH_9, INPUT_PULLUP);     /***** 2 ******/
//        pinMode(SWITCH_10, INPUT_PULLUP);     /***** 2 ******/
//        pinMode(SWITCH_11, INPUT_PULLUP);     /***** 2 ******/
        delay(50);
        int Switch_8_State, Switch_9_State, Switch_10_State, Switch_11_State;
        Switch_8_State = digitalRead(SWITCH_8);
        Switch_9_State = digitalRead(SWITCH_9);
//        Switch_10_State = digitalRead(SWITCH_10);
//        Switch_11_State = digitalRead(SWITCH_11);
        if (Switch_8_State == LOW) {  main::logln(" SWITCH_8 is on");} else {main::logln(" SWITCH_8 is off");}
        if (Switch_9_State == LOW) {  main::logln(" SWITCH_9 is on");} else {main::logln(" SWITCH_9 is off");}
//        if (Switch_10_State == LOW) {  main::logln(" SWITCH_10 is on");} else {main::logln(" SWITCH_10 is off");}
//        if (Switch_11_State == LOW) {  main::logln(" SWITCH_11 is on");} else {main::logln(" SWITCH_11 is off");}
    #endif

    #if USE_MATRIX
        matrix.loadSequence(animation);
            matrix.begin();
            //matrix.autoscroll(300);
            matrix.play(true);
    #endif

    #if USE_DISTANCE
        pinMode(Trig_PIN, OUTPUT);    /***** 6 ******/
        pinMode(Echo_PIN, INPUT);     /***** 7 ******/
        main::logln("Using Sonar Distance");
    #endif

    #if USE_DOT
        pinMode(DotClockPIN,OUTPUT);/***** 5 ******/
        pinMode(DotDataPIN,OUTPUT); /***** 4 ******/
        digitalWrite(DotClockPIN,LOW);
        digitalWrite(DotDataPIN,LOW);
        Pesto::matrix_display(clear);
        Pesto::pestoMatrix();
        main::logln("Using Dot Matrix");
    #endif

    #if USE_MIC
        pinMode(MIC_PIN, INPUT);
        baseSound = map(analogRead(MIC_PIN), 0, 1023, 0, 255); /***** A3 ******/
        printLog("Using Mic");
    #endif

    #if USE_GYRO
        #if USE_ADAFRUIT
            displayAdafruit::display.clearDisplay();
        #endif
        
        gyroscope::gyroSetup();
    #endif

    #if USE_COMPASS
        #if USE_ADAFRUIT
            displayAdafruit::display.clearDisplay();
        #endif
        compass::compassSetup();
    #endif

    #if USE_BAROMETER
        #if USE_ADAFRUIT
            displayAdafruit::display.clearDisplay();
        #endif
        barometer::baroSetup();
    #endif

    #if USE_IRREMOTE
            IRreceiver::setupIrRemote();
    #endif

	#if USE_PWM_BOARD
        pwm_board::setupPWM();
		delay(500);
	#endif

    #if USE_TIMERS
        timers::initTimers();
    #endif
    #if USE_ADAFRUIT
        displayAdafruit::display.clearDisplay();
    #endif
}

/*********************************** Loop **********************************/
// section Loop
/***************************************************************************/

void loop(){
    PS4::controller();

    #if USE_IRREMOTE
        IRreceiver::irRemote();
	#endif

    #if USE_DISTANCE
        avoid_objects::distanceF = avoid_objects::checkDistance();  /// assign the front distance detected by ultrasonic sensor to variable a
        if (avoid_objects::distanceF < 35) {
            #if USE_PWM_BOARD
                pwm_board::RGBled(230, 0, 0);
                pwm_board::leftLedStrip(255, 0, 0);
                pwm_board::rightLedStrip(255, 0, 0);
            #endif
            main::log("      Pesto  ");
            main::log("              Pesto");
            #if USE_DOT
                Pesto::pestoMatrix();
            #endif
            double deltime = avoid_objects::distanceF*3;
            delay(deltime);
        } else {
    #endif
            #if USE_MIC
                    int micStatus = analogRead(MIC_PIN);
                    int mic255 = map(micStatus, 0, 1023, 0, 255);

                    if (mic255 > baseSound) {
                        #if USE_PWM
                                pwm_board::RGBled(mic255, 0, mic255);
                        #endif
                    } else {
                        #if USE_PWM
                            pwm_board::RGBled(0, mic255, 0);
                        #endif
                    }
            #endif

            #if USE_PWM_BOARD
                pwm_board::RGBled(0, 0, Follow_light::lightSensor());
                pwm_board::leftLedStrip(0, 255, 0);
                pwm_board::rightLedStrip(0, 255, 0);
            #endif
    #if USE_DISTANCE
        }
    #endif

    #if USE_GYRO
        gyroscope::gyroDetectMovement();
    #endif

    #if USE_TIMERS
        timers::update();
    #endif

    #if READ_ESP32
        // Read messages from Arduino R4 ESP32
            if (SERIAL_AT.available()) {
                displayU8G2::display.print("ESP32 says: ");
                while (SERIAL_AT.available()) {
                    Serial.write(SERIAL_AT.read());
                    displayU8G2::display.println(SERIAL_AT.read());
                }
            }
    #endif
}



