//
// Created by mr on 11/13/2023.
//

#ifndef TIMERS_H
#define TIMERS_H


#include "Arduino.h"
#include <TimerEvent.h>




class timers {
private:
    static const int timerOnePeriod = 1000;
    static const int timerTwoPeriod = 250;
    static const int timerThreePeriod = 7000;
    static const int timerMouthPeriod = 1250;
    unsigned long last_event = 0;

public:
    static bool timerTwoActive;
    static bool timerTreeActive;
    static uint64_t timerButton;
    static void dotMatrixTimer();
    static void sensorTimer();
    static void resetTimers();
    static void mouthTimer();
    static void initTimers();

    static void update();


};


#endif //TIMERS_H
