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
#define USE_ROUND 0
#define USE_MENU 1
#define LOG_DEBUG 1

#define USE_PS4 1
#define USE_SD_CARD 1

#define USE_GYRO 0
#define USE_COMPASS 0
#define USE_BAROMETER 0
#define USE_DISTANCE 1

#define USE_IRREMOTE 0
#define USE_I2C_SCANNER 1
#define USE_PWM_BOARD 1
#define USE_DOT 1
#define USE_MIC 0
#define USE_SWITCH 0

#define USE_ANALOG 0

#define USE_ROBOT 0
#define USE_TIMERS 0


#define USE_MATRIX 0
#define USE_MATRIX_PREVIEW 0
#define READ_ESP32 0
#define USE_LCD 0

#define USE_HM_10_BLE 0
#define ARDUINO_ARCH_RENESAS_UNO

/********************************************** PIN Defines ****************************************************/
// section pin define
/***************************************************************************************************************/

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define EXT_ANALOG_0 0
#define EXT_ANALOG_1 1
#define EXT_ANALOG_2 2
#define EXT_ANALOG_3 3


#define Rem_OK  0xBF407F
#define RX_PIN       0
#define TX_PIN       1
#define LAZER_PIN     2
#define L_PWM   	  3   // define PWM control pin of right motor

#define Trig_PIN      4  //   ultrasonic trig Pin
#define Echo_PIN      5  //   ultrasonic echo Pin



#define R_PWM  		  7  // define PWM control pin of left motor
#define L_ROT  		  8  // define the direction control pin of right motor
#define R_ROT    	  9  // define the direction control pin of left motor
#define TFT_CS 		 10  // CS: Chipselect
#define TFT_MOSI 	 6  // MOSI (SD-DI, DI, SIMO, SDO, DO, DOUT, SO, MTSR.)  SPI Master out Slave in
#define TFT_MISO	 12  // MISO : SOMI, SDI, DI, DIN, SI, MRS
#define TFT_SCK		 13  // SCLK : SCK, CLK.

#define DotDataPIN   A2 // Set data  pin to 4
#define DotClockPIN  A3  // Set clock pin to 5

#define SWITCH_8
#define SWITCH_9


#define light_L_PIN EXT_ANALOG_0
#define MIC_L_PIN   EXT_ANALOG_1
#define light_R_PIN EXT_ANALOG_2
#define MIC_R_PIN   EXT_ANALOG_3

//#define IR_Pin      A2

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



#define FPS 30

// 0 for SdFat/File, 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3
#define CS_PIN 10





#endif //CONFIG_H
