//
// Created by mr on 11/13/2023.
//

#ifndef CONFIG_H
#define CONFIG_H

/***************************************************************************************************************/
// section define
/***************************************************************************************************************/

#define USE_JOYSTICK 0
#define USE_ADAFRUIT 0
#define USE_U8G2 1
#define SMALL 0
#define DISPLAY_DEMO 1

#define USE_GYRO 0
#define USE_COMPASS 0
#define USE_BAROMETER 0

#define USE_IRREMOTE 0
#define USE_I2C_SCANNER 1
#define USE_PWM 1
#define USE_DOT 1

#define USE_ROBOT 0
#define USE_TIMERS 1
#define USE_TEXTFINDER 0

#define USE_MATRIX 0
#define READ_ESP32 0
#define USE_LCD 0
#define ARDUINO_ARCH_RENESAS_UNO


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

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#include <Adafruit_BME280.h>

#include <U8g2lib.h>



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




#define RX_PIN       0
#define TX_PIN       1
#define LED_PIN      2



#define Trig_PIN     6  // ultrasonic trig Pinz
#define Echo_PIN     7  // ultrasonic echo Pin

#define PIN_9        9
#define PIN_10      10



#define IR_Pin      A2
#define MIC_PIN     A3

#if USE_JOYSTICK
#define Joystick_X  A0
    #define Joystick_Y  A1
#endif



#define THRESHOLD 5
#define top  0 // lcd screen top line
#define bot  1 // lcd screen bottom line

#if USE_U8G2
    #if SMALL
         U8G2_SSD1306_128X64_NONAME_1_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
    #else
        U8G2_SH1106_128X64_NONAME_1_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
    #endif
#endif


#endif //CONFIG_H
