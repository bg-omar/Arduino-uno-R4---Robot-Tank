//
// Created by mr on 11/13/2023.
//

#ifndef CONFIG_H
#define CONFIG_H

/***************************************************************************************************************/
// section define
/***************************************************************************************************************/

#define USE_JOYSTICK 0
#define USE_ADAFRUIT 1
#define USE_U8G2 0
#define SMALL 0
#define DISPLAY_DEMO 0

#define USE_GYRO 1
#define USE_COMPASS 1
#define USE_BAROMETER 1

#define USE_IRREMOTE 0
#define USE_I2C_SCANNER 1
#define USE_PWM 1
#define USE_DOT 1

#define USE_ROBOT 1
#define USE_TIMERS 1
#define USE_TEXTFINDER 0

#define USE_MATRIX 0
#define READ_ESP32 0
#define USE_LCD 0
#define ARDUINO_ARCH_RENESAS_UNO





/********************************************** PIN Defines ****************************************************/
// section pin define
/***************************************************************************************************************/

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define Rem_OK  0xBF407F
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




#define top  0 // lcd screen top line
#define bot  1 // lcd screen bottom line




#endif //CONFIG_H
