//
// Created by mr on 10/27/2023.
//

#include <Arduino.h>
#include "follow_light.h"
#include "motor.h"

/*************************************************** Light Follow **********************************************/
// section Follow Light
/***************************************************************************************************************/


void Folloe_light::light_track() {
    flag = 0;
    while (flag == 0) {
        lightSensorR = analogRead(light_R_Pin);
        lightSensorL = analogRead(light_L_Pin);
        if (lightSensorR > 650 && lightSensorL > 650) {
            Motor::Car_front();
        }
        else if (lightSensorR > 650 && lightSensorL <= 650) {
            Motor::Car_left();
        }
        else if (lightSensorR <= 650 && lightSensorL > 650) {
            Motor::Car_right();
        }
        else {
            Motor::Car_Stop();
        }
        exitLoop();
    }
}
