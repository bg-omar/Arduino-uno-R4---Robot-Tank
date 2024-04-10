//
// Created by mr on 11/17/2023.
//

#ifndef ARDUINO_R4_UNO_WALL_Z_COMPASS_H
#define ARDUINO_R4_UNO_WALL_Z_COMPASS_H

#include <Adafruit_HMC5883_U.h>
#include "U8g2lib.h"


class compass {
private:
    static Adafruit_HMC5883_Unified mag;
    static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2;
public:
    static double readCompass();
    static void showCompass();
    static void compassSetup() ;
    static void displaySensorDetails();

    static void displayCompass();
};



#endif //ARDUINO_R4_UNO_WALL_Z_COMPASS_H
