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
#include "MicStereo.h"
#include "avoid_objects.h"
#include "I2Cscanner.h"
#include "general_timer.h"

#if USE_MATRIX
    #include "Arduino_LED_Matrix.h"
// Define an array to hold pixel data for a single frame (4 pixels)
    uint32_t frame[] = {
            0, 0, 0, 0xFFFF
    };

    ArduinoLEDMatrix matrix;
#endif

bool main::Found_Display;
bool main::Found_Gyro = false;
bool main::Found_Compass = false;
bool main::Found_Mics = false;
bool main::Found_PwmBoard = false;
bool main::Found_Switch = false;
bool main::Found_Sonar = false;
int timers::timerButton;

int pwm_board::posXY = 90;  // set horizontal servo position
int pwm_board::posZ = 5;   // set vertical servo position



U8G2_SH1106_128X64_NONAME_1_HW_I2C displayU8G2::display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);



/********************************************** Setup booting the arduino **************************************/
// section Main Functions
/***************************************************************************************************************/
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
    Serial.println("Wall-Z Arduino Robot booting");

    #if USE_U8G2
        displayU8G2::U8G2setup();
    #endif

    #if USE_I2C_SCANNER
        I2Cscanner::scan();
        delay(500);
    #endif

    delay(1);

    main::log("Serial,\t");
    delay(1);
    Serial1.begin(115200);
    main::log("Serial1,  \t");
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

        if (Switch_8_State == LOW) {  main::log(" SWITCH_8 is on");} else {main::log(" SWITCH_8 is off");}
        if (Switch_9_State == LOW) {  main::logln(" SWITCH_9 is on");} else {main::logln(" SWITCH_9 is off");}

        delay(500);
    #endif

    #if USE_PWM_BOARD
        pwm_board::setupPWM();
        delay(500);
    #endif


    #if USE_MATRIX_PREVIEW
        matrix.begin();
    #elif USE_MATRIX
        matrix.loadSequence(animation);
        matrix.begin();
        matrix.autoscroll(300);
        matrix.play(true);
        delay(500);
        main::log(" R4 matrix ");
    #endif

    #if USE_DISTANCE
        pinMode(Trig_PIN, OUTPUT);    /***** 6 ******/
        pinMode(Echo_PIN, INPUT);     /***** 7 ******/
        main::log(" Sonar " );
        delay(500);
    #endif

    #if USE_DOT
        Pesto::setup_pestoMatrix();
        delay(500);
    #endif

    #if USE_MIC
        MicStereo::MicSetup();
        delay(500);
    #endif

    #if USE_GYRO
        gyroscope::gyroSetup();
        delay(500);
    #endif

    #if USE_COMPASS
        compass::compassSetup();
        delay(500);
    #endif

    #if USE_BAROMETER
        barometer::baroSetup();
        delay(500);
    #endif

    #if USE_IRREMOTE
            IRreceiver::setupIrRemote();
    #endif

    #if USE_TIMERS
        timers::initTimers();
        delay(500);

//        general_timer::setup_General_Timer();
//        delay(500);
#endif
}

/*********************************** Loop **********************************/
// section Loop
/***************************************************************************/

void loop(){

    #if USE_PS4
        PS4::controller();
    #endif

    #if USE_SWITCH
        int Switch_8_State, Switch_9_State;
        Switch_8_State = digitalRead(SWITCH_8);
        Switch_9_State = digitalRead(SWITCH_9);


        if (Switch_9_State == LOW) {  } else { }
    #endif

        #if USE_IRREMOTE
            IRreceiver::irRemote();
        #endif

    #if USE_GYRO
       if (main::Found_Gyro) gyroscope::gyroDetectMovement();
    #endif

    #if USE_DISTANCE
        avoid_objects::distanceF = avoid_objects::checkDistance();  /// assign the front distance detected by ultrasonic sensor to variable a
        if (avoid_objects::distanceF < 25) {
            #if USE_PWM_BOARD
                pwm_board::RGBled(230, 0, 0);
                if (Switch_8_State == LOW) {
                    pwm_board::leftLedStrip(255, 0, 0);
                    pwm_board::rightLedStrip(255, 0, 0);
                } else {
                    pwm_board::leftLedStrip(70, 0, 70);
                    pwm_board::rightLedStrip(70, 0, 70);
                }
            #endif
            if (main::Found_Display) displayU8G2::u8g2log.println(avoid_objects::distanceF);
            #if USE_DOT
                Pesto::pestoMatrix();
            #endif
            double deltime = avoid_objects::distanceF*3;
            delay(deltime);
        } else {
    #endif
        #if USE_MIC
            MicStereo::MicLoop();
        #endif

        #if USE_PWM_BOARD
            pwm_board::RGBled(0, 0, Follow_light::lightSensor());
            if (Switch_8_State == LOW) {
                pwm_board::leftLedStrip(0, 255, 0);
                pwm_board::rightLedStrip(0, 255, 0);
            } else {
                pwm_board::leftLedStrip(0, 70, 0);
                pwm_board::rightLedStrip(0, 70, 0);
            }
        #endif
    #if USE_DISTANCE
        }
    #endif

    #if USE_TIMERS
        timers::update();
//        general_timer::loop_General_Timer();
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

    #if USE_MATRIX_PREVIEW
        // Check if there are at least 12 bytes available in the serial buffer
        if(Serial.available() >= 12){
            // Read 4 bytes from the serial buffer and compose them into a 32-bit value for each element in the frame
            frame[0] = Serial.read() | Serial.read() << 8 | Serial.read() << 16 | Serial.read() << 24;
            frame[1] = Serial.read() | Serial.read() << 8 | Serial.read() << 16 | Serial.read() << 24;
            frame[2] = Serial.read() | Serial.read() << 8 | Serial.read() << 16 | Serial.read() << 24;

            // Load and display the received frame data on the LED matrix
            matrix.loadFrame(frame);
        }
    #endif

}



