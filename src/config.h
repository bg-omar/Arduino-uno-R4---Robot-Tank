//
// Created by mr on 11/13/2023.
//

#ifndef CONFIG_H
#define CONFIG_H

/***************************************************************************************************************/
// section define
/***************************************************************************************************************/

#define USE_ADAFRUIT 0
#define USE_U8G2 1
#define SMALL 0
#define DISPLAY_DEMO 0

#define USE_GYRO 1
#define USE_COMPASS 1
#define USE_BAROMETER 0

#define USE_IRREMOTE 0
#define USE_I2C_SCANNER 1
#define USE_PWM_BOARD 1
#define USE_DOT 1
#define USE_MIC 0

#define USE_SWITCH 1

#define USE_ROBOT 0
#define USE_TIMERS 1
#define USE_DISTANCE 1

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
#define SWITCH_8     8
#define SWITCH_9     9
#define SWITCH_10   10
#define SWITCH_11   11
#define IR_Pin      A2
#define MIC_PIN     A3


#define top  0 // lcd screen top line
#define bot  1 // lcd screen bottom line

#define DPAD_U   1100
#define DPAD_R   1200
#define DPAD_D   1300
#define DPAD_L   1400
#define SQUARE   3100
#define xCROSS   3200
#define CIRCLE   3300
#define TRIANG   3400
#define OPTION   2900
#define xSHARE   2800
#define PSHOME   2500
#define L1       2100
#define L3       2300
#define R1       2200
#define R3       2400
#define TOUCHPD  2700
#define MIC      3700
#define CHARGING 3500
#define XAUDIO   3600
#define URIGHT   1500
#define DRIGHT   1600
#define D_LEFT   1700
#define UPLEFT   1800


#endif //CONFIG_H
