//
// Created by mr on 11/13/2023.
//

#ifndef MAIN_RA_H
#define MAIN_RA_H


#include "config.h"


#if USE_MATRIX
    #include "Arduino_LED_Matrix.h"
    ArduinoLEDMatrix matrix;
#endif

class main {

public:
    static void log(const char *text);
};

#if USE_LCD
LiquidCrystal_I2C lcd(0x27,16,2);
#endif

#endif //MAIN_RA_H
