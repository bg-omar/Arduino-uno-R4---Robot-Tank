//
// Created by mr on 10/27/2023.
//

#ifndef FOLLOW_LIGHT_H
#define FOLLOW_LIGHT_H

#include "motor.h"
#include "PS4.h"

#define light_L_Pin A0
#define light_R_Pin A1
int lightSensorL, lightSensorR;

class Folloe_light {

public:
    static void light_track();

};





#endif //FOLLOW_LIGHT_H
