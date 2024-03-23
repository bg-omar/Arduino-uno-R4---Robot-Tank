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
    static bool Found_Display;
    static bool Found_Gyro;
    static bool Found_Compass;
    static bool Found_Mics;
    static bool Found_PwmBoard;
    static bool Found_Switch;
    static bool Found_Sonar;
};



#endif //MAIN_RA_H
