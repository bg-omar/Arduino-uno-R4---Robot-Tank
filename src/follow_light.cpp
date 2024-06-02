//
// Created by mr on 10/27/2023.
//

#include <Arduino.h>
#include "follow_light.h"
#include "motor.h"
#include "PS4.h"
#include "pwm_board.h"


/*************************************************** Light Follow **********************************************/
// section Follow Light
/***************************************************************************************************************/

int Follow_light::flag;
int Follow_light::lightSensorL;
int Follow_light::lightSensorR;

double Follow_light::lightSensor(){
    Follow_light::lightSensorL = analogRead(light_R_PIN);
    Follow_light::lightSensorR = analogRead(light_L_PIN);
    long outputValueR = map(Follow_light::lightSensorL, 0, 1023, 0, 255);
    long outputValueL = map(Follow_light::lightSensorR, 0, 1023, 0, 255);
    double calcValue = 255 - (outputValueR + outputValueL)*.5;
    return (calcValue < 0) ? 0 : calcValue;
}


void Follow_light::light_track() {
    flag =0;
    while (flag == 0) {
        lightSensor();
        pwm_board::RainbowColor();

        if (lightSensorR > 650 && lightSensorL > 650) {
            Motor::Car_front();
        }
        else if (lightSensorR > 650) {
            Motor::Car_left();
        }
        else if (lightSensorL > 650) {
            Motor::Car_right();
        }
        else {
            Motor::Car_Stop();
        }
        flag = PS4::exitLoop();
    }
}
