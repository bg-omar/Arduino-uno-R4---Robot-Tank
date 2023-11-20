//
// Created by mr on 11/19/2023.
//

#ifndef ARDUINO_R4_UNO_WALL_Z_DISPLAYADAFRUIT_H
#define ARDUINO_R4_UNO_WALL_Z_DISPLAYADAFRUIT_H

#include "Adafruit_SH110X.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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


class displayAdafruit {
public:
    static void testdrawroundrect();
    static void testfillroundrect();
    static void testdrawchar();
    static void testdrawstyles() ;
    static void testscrolltext();
    static void displayLoop();
    static void setupAdafruit();
#if SMALL
    static Adafruit_SSD1306 display;
#else
    static Adafruit_SH1106G display;
#endif

};



#endif //ARDUINO_R4_UNO_WALL_Z_DISPLAYADAFRUIT_H
