//
// Created by mr on 11/13/2023.
//

#ifndef MAIN_RA_H
#define MAIN_RA_H


#include "config.h"

class main {

public:
    static void log(const char *text);

    static void logln(const char *text);
};

#if USE_LCD
LiquidCrystal_I2C lcd(0x27,16,2);
#endif

#endif //MAIN_RA_H
