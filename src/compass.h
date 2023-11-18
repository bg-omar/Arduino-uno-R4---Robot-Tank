//
// Created by mr on 11/17/2023.
//

#ifndef ARDUINO_R4_UNO_WALL_Z_COMPASS_H
#define ARDUINO_R4_UNO_WALL_Z_COMPASS_H

#include <Adafruit_HMC5883_U.h>

class compass {
private:
    static Adafruit_HMC5883_Unified mag;
public:
    static double readCompass();
    static void showCompass();
    static void compassSetup() ;
};


#endif //ARDUINO_R4_UNO_WALL_Z_COMPASS_H
