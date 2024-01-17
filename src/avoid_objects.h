//
// Created by mr on 11/18/2023.
//

#ifndef ARDUINO_R4_UNO_WALL_Z_AVOID_OBJECTS_H
#define ARDUINO_R4_UNO_WALL_Z_AVOID_OBJECTS_H

#include "config.h"



class avoid_objects {
public:
    static long random2;
    static void avoid();
    static double checkDistance();

    static double distanceF;
    static double distanceR, distanceL;

    static int exitLoop();
};


#endif //ARDUINO_R4_UNO_WALL_Z_AVOID_OBJECTS_H
