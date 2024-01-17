//
// Created by mr on 10/27/2023.
//

#ifndef FOLLOW_LIGHT_H
#define FOLLOW_LIGHT_H

#include "config.h"

#define light_L_Pin A0
#define light_R_Pin A1


class Follow_light {
private:
    static int flag;
public:
    static int lightSensorL, lightSensorR;
    static void light_track();
    static double lightSensor();

    static int exitLoop();
};

#endif //FOLLOW_LIGHT_H
