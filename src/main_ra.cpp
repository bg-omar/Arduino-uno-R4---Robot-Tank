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
#include "secrets.h"
#include "main_ra.h"
#include "barometer.h"
#include "PS4.h"
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


U8G2_SH1106_128X64_NONAME_1_HW_I2C displayU8G2::display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

long baseSound;
int r,g,b;

int16_t lastY;

void printLog(const char *text, int size = 1, int16_t x = 0, int16_t y = lastY){
    #if USE_ADAFRUIT
        displayAdafruit::display.setTextWrap(false);
        displayAdafruit::display.setCursor(x,y);             // Start at top-left corner
        displayAdafruit::display.setTextSize(size);             // Normal 1:1 pixel scale
        displayAdafruit::display.setTextColor(PIXEL_WHITE);        // Draw white text
        displayU8G2::display.println(text);
        lastY = y + 8;
        #if USE_ADAFRUIT
            displayAdafruit::display.display();
        #endif
    #else
        U8G2_SH1106_128X64_NONAME_1_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
        displayU8G2::display.drawStr(x, y, (const char *) text);
        displayU8G2::display.drawFrame(0,0,display.getDisplayWidth(),display.getDisplayHeight() );
        lastY = y + 8;
    #endif
}





/********************************************** Setup booting the arduino **************************************/
// section Setup
/***************************************************************************************************************/
void setup(){
    unsigned char clear[] =  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    Wire.begin();

    #if USE_ADAFRUIT
        displayAdafruit::setupAdafruit();
    #else
        displayU8G2::display.begin();
        displayU8G2::u8g2_prepare();
        displayU8G2::display.firstPage();
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
        Pesto::matrix_display(clear);
        Pesto::pestoMatrix();
	#endif

	#if USE_I2C_SCANNER
		I2Cscanner::scan();
		delay(500);
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
        IRreceiver::setupIrRemote();
	#if USE_PWM
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
unsigned long t = 0;

void loop(){
    while (Serial1.available() > 0) {
        PS4::controller();
    }

    // React on messages from ESP32-CAM AI-Thinker
    while(Serial1.available()) {
        int c = Serial1.read();
        Serial.write(c);
    }

    #if USE_IRREMOTE
        IRreceiver::irRemote();
	#endif

    avoid_objects::distanceF = avoid_objects::checkDistance();  /// assign the front distance detected by ultrasonic sensor to variable a
    if (avoid_objects::distanceF < 35) {
        #if USE_PWM
        pwm_board::RGBled(230, 0, 0);
        #endif
        #if USE_DOT
            Pesto::pestoMatrix();
        #endif
        double deltime = avoid_objects::distanceF*3;
        delay(deltime);
    } else {
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
		#if USE_PWM
            pwm_board::RGBled(0, 0, Follow_light::lightSensor());
		#endif
	}
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


