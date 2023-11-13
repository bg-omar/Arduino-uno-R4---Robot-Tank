//
// Created by mr on 11/13/2023.
//

#ifndef TIMERS_H
#define TIMERS_H


#include "Arduino.h"
uint64_t timerButton;
#include <TimerEvent.h>


const int timerOnePeriod = 1000;
const int timerTwoPeriod = 250;
const int timerThreePeriod = 7000;
const int timerMouthPeriod = 1250;

bool timerTwoActive = false;
bool timerTreeActive = false;
unsigned long last_event = 0;
TimerEvent timerOne;
TimerEvent timerTwo;
TimerEvent timerThree;
TimerEvent timerMouth;


class timers {
public:
    static void dotMatrixTimer();
    static void sensorTimer();

    static void resetTimers();

    static void mouthTimer();
};


#endif //TIMERS_H
