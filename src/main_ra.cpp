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
//#include "secrets.h"
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


#if USE_MATRIX
    #include "Arduino_LED_Matrix.h"
    ArduinoLEDMatrix matrix;
#endif

long baseRSound, baseLSound;

int PS4::flag = 0;
int PS4::posXY = 90;  // set horizontal servo position
int PS4::posZ = 5;   // set vertical servo position


bool main::Found_Display = false;
bool main::Found_Gyro = false;
bool main::Found_Compass = false;
bool main::Found_Mics = false;
bool main::Found_PwmBoard = false;
bool main::Found_Switch = false;
bool main::Found_Sonar = false;
int timers::timerButton;


#if USE_U8G2
    U8G2_SH1106_128X64_NONAME_1_HW_I2C displayU8G2::display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#endif

void main::log(const char *text) {
    if (Found_Display) {
        displayU8G2::U8G2print((const char *) text);
    }
    Serial.print((const char *) text);
}
void main::logln(const char *text) {
    if (Found_Display) {
        displayU8G2::U8G2println((const char *) text);
    }
    Serial.println((const char *) text);
}

/********************************************** Setup booting the arduino **************************************/
// section Setup
/***************************************************************************************************************/
void setup(){
    Wire.begin();
    Serial.begin(115200); // Initialize the hardware serial port for debugging
    Serial.println("Debug Serial started");
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

    main::logln("Debug Serial started");
    delay(1);
    Serial1.begin(115200);
    main::logln("Serial1 started");
    delay(1);
    SERIAL_AT.begin(115200);
    main::logln("Serial_AT started");
    delay(1);

    Motor::motor_setup();

    pinMode(LED_PIN, OUTPUT);     /***** 2 ******/

    #if USE_SWITCH
        pinMode(SWITCH_8, INPUT_PULLUP);     /***** 8 ******/
        pinMode(SWITCH_9, INPUT_PULLUP);     /***** 9 ******/

        delay(50);
        int Switch_8_State, Switch_9_State;
        Switch_8_State = digitalRead(SWITCH_8);
        Switch_9_State = digitalRead(SWITCH_9);

        if (Switch_8_State == LOW) {  main::logln(" SWITCH_8 is on");} else {main::logln(" SWITCH_8 is off");}
        if (Switch_9_State == LOW) {  main::logln(" SWITCH_9 is on");} else {main::logln(" SWITCH_9 is off");}
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
        Pesto::setup_pestoMatrix();
    #endif

    #if USE_MIC
        pinMode(MIC_R_PIN, INPUT);
        baseRSound = map(analogRead(MIC_R_PIN), 0, 1023, 0, 255); /***** A3 ******/
        pinMode(MIC_L_PIN, INPUT);
        baseLSound = map(analogRead(MIC_L_PIN), 0, 1023, 0, 255); /***** A1 ******/
        main::log("     Left Mic: ");
        if (main::Found_Display) displayU8G2::u8g2log.println(baseLSound);
        main::log("    Right Mic: ");
        if (main::Found_Display) displayU8G2::u8g2log.println(baseRSound);
        main::logln("Using Mic");
    #endif

    #if USE_GYRO
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

    int Switch_8_State, Switch_9_State;
    Switch_8_State = digitalRead(SWITCH_8);
    Switch_9_State = digitalRead(SWITCH_9);


    if (Switch_9_State == LOW) {  } else { }


    #if USE_IRREMOTE
        IRreceiver::irRemote();
	#endif

    #if USE_DISTANCE
        avoid_objects::distanceF = avoid_objects::checkDistance();  /// assign the front distance detected by ultrasonic sensor to variable a
        if (avoid_objects::distanceF < 25) {
            #if USE_PWM_BOARD
                pwm_board::RGBled(230, 0, 0);
                if (Switch_8_State == LOW) {
                    pwm_board::leftLedStrip(255, 0, 0);
                    pwm_board::rightLedStrip(255, 0, 0);
                }
            #endif
            main::log("      Pesto  ");
            if (main::Found_Display) displayU8G2::u8g2log.println(avoid_objects::distanceF);
            #if USE_DOT
                Pesto::pestoMatrix();
            #endif
            double deltime = avoid_objects::distanceF*3;
            delay(deltime);
        } else {
    #endif
        #if USE_MIC
            int micRStatus = analogRead(MIC_R_PIN);
            int micR255 = map(micRStatus, 0, 1023, 0, 255);

            int micLStatus = analogRead(MIC_L_PIN);
            int micL255 = map(micLStatus, 0, 1023, 0, 255);

            if (micR255 > baseRSound) {
                if (main::Found_Display) displayU8G2::u8g2log.println(micR255);
                #if USE_PWM_BOARD
                        pwm_board::RGBled(micR255, micR255, 0);
                #endif
            } else if (micL255 > baseLSound) {
                if (main::Found_Display) displayU8G2::u8g2log.println(micL255);
                #if USE_PWM_BOARD
                    pwm_board::RGBled(0, micR255, micL255);
                #endif
            } else {
                #if USE_PWM_BOARD
                    pwm_board::RGBled(0, micR255, 0);
                #endif
            }
        #endif

        #if USE_PWM_BOARD
            pwm_board::RGBled(0, 0, Follow_light::lightSensor());
            if (Switch_8_State == LOW) {
                pwm_board::leftLedStrip(0, 255, 0);
                pwm_board::rightLedStrip(0, 255, 0);
            }
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
                displayU8G2::u8g2log.print("ESP32 says: ");
                while (SERIAL_AT.available()) {
                    Serial.write(SERIAL_AT.read());
                    displayU8G2::u8g2log.println(SERIAL_AT.read());
                }
            }
    #endif
}



