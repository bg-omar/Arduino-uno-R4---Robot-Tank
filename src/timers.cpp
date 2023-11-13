//
// Created by mr on 11/13/2023.
//

#include "timers.h"



/***************************************************** Functions s**********************************************/
// section Timer Functions
/***************************************************************************************************************/


void timers::dotMatrixTimer(){
    #if USE_DOT
        Pesto::pestoMatrix();
    #endif
}

void timers::sensorTimer(){
    #if USE_COMPASS
        if (timerTwoActive && timerButton == L1){
            compass();
        }
    #endif

    #if USE_GYRO
        if (timerTwoActive && timerButton == R1){
            gyroFunc();
        }
    #endif
}

void timers::resetTimers(){
    timerTwoActive = false;
    timerTreeActive = false;
    #if USE_ADAFRUIT
        display.clearDisplay();
    #endif
}

void timers::mouthTimer(){
    #if USE_ADAFRUIT
        displayLoop();
    #endif
}
