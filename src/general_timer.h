//
// Created by mr on 2/27/2024.
//

#ifndef ARDUINO_R4_UNO_WALL_Z_GENERAL_TIMER_H
#define ARDUINO_R4_UNO_WALL_Z_GENERAL_TIMER_H


#include <r_timer_api.h>

class general_timer {

    static void setup_General_Timer() ;

    static void loop_General_Timer();

public:
    static void timer_callback(timer_callback_args_t __attribute((unused)) *p_args) ;

    static bool beginTimer(float rate) ;
};


#endif //ARDUINO_R4_UNO_WALL_Z_GENERAL_TIMER_H
