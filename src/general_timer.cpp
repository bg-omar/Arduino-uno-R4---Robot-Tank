//
// Created by mr on 2/27/2024.
//

#include "general_timer.h"
#include "FspTimer.h"

FspTimer audio_timer;
uint64_t count=0;
uint64_t start_time=0;

// callback method used by timer
void general_timer::timer_callback(timer_callback_args_t  __attribute((unused)) *p_args) {
    count++;
}

bool general_timer::beginTimer(float rate) {
    uint8_t timer_type = GPT_TIMER;
    int8_t tindex = FspTimer::get_available_timer(timer_type);
    if (tindex < 0){
        tindex = FspTimer::get_available_timer(timer_type, true);
    }
    if (tindex < 0){
        return false;
    }

    FspTimer::force_use_of_pwm_reserved_timer();

    if(!audio_timer.begin(TIMER_MODE_PERIODIC, timer_type, tindex, rate, 0.0f, general_timer::timer_callback)){
        return false;
    }

    if (!audio_timer.setup_overflow_irq()){
        return false;
    }

    if (!audio_timer.open()){
        return false;
    }

    if (!audio_timer.start()){
        return false;
    }
    return true;
}


void general_timer::setup_General_Timer() {
    Serial.begin(115200);
    general_timer::beginTimer(8000);
    start_time = millis();
}


void general_timer::loop_General_Timer() {
    // calculate the effective frequency
    int freq = 1000 * count / (millis()-start_time);
    Serial.println(freq);
    count = 0;
    start_time = millis();
    delay(1000);
}


