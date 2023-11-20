//
// Created by mr on 11/13/2023.
//

#include "timers.h"

bool timers::timerTwoActive = false;
bool timers::timerTreeActive = false;
TimerEvent timers::timerOne;
TimerEvent timers::timerTwo;
TimerEvent timers::timerThree;

/***************************************************** Functions s**********************************************/
// section Timer Functions
/***************************************************************************************************************/
void timers::initTimers() {
    timers::timerOne.set(timers::timerOnePeriod, timers::dotMatrixTimer);
    timers::timerTwo.set(timers::timerTwoPeriod, timers::sensorTimer);
    timers::timerThree.set(timers::timerThreePeriod, timers::resetTimers);
#if DISPLAY_DEMO
    timers::timerMouth.set(timers::timerMouthPeriod, timers::mouthTimer);
#endif
}

void timers::update(){
    timers::timerOne.update();
    timers::timerTwo.update();
    timers::timerThree.update();
#if DISPLAY_DEMO
    timers::timerMouth.update();
#endif
}
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
    timers::timerTwoActive = false;
    timers::timerTreeActive = false;
    #if USE_ADAFRUIT
        display.clearDisplay();
    #endif
}

void timers::mouthTimer(){
    #if USE_ADAFRUIT
        displayLoop();
    #endif
}
