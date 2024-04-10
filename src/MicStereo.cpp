//
// Created by mr on 3/24/2024.
//

#include "MicStereo.h"
#include "Arduino.h"
#include "config.h"
#include "displayU8G2.h"
#include "main_ra.h"
#include "pwm_board.h"

long MicStereo::baseRSound, MicStereo::baseLSound = 0;


void MicStereo::MicSetup() {
    pinMode(MIC_R_PIN, INPUT);
    MicStereo::baseRSound = map(analogRead(MIC_R_PIN), 0, 1023, 0, 255); /***** A3 ******/
    pinMode(MIC_L_PIN, INPUT);
    MicStereo::baseLSound = map(analogRead(MIC_L_PIN), 0, 1023, 0, 255); /***** A1 ******/
#if LOG_DEBUG
    main::log(" L-Mic: ");
    if (main::Found_Display) displayU8G2::u8g2log.println(baseLSound);
    main::log(" R-Mic: ");
    if (main::Found_Display) displayU8G2::u8g2log.println(baseRSound);
#endif
}


void MicStereo::MicLoop() {
    int micRStatus = analogRead(MIC_R_PIN);
    int micR255 = map(micRStatus, 0, 1023, 0, 255);

    int micLStatus = analogRead(MIC_L_PIN);
    int micL255 = map(micLStatus, 0, 1023, 0, 255);

    if (micR255 > MicStereo::baseRSound) {
#if LOG_DEBUG
        if (main::Found_Display) displayU8G2::u8g2log.println(micR255);
#endif
#if USE_PWM_BOARD
        pwm_board::RGBled(micR255, micR255, 0);
#endif
    } else if (micL255 > MicStereo::baseLSound) {
#if LOG_DEBUG
        if (main::Found_Display) displayU8G2::u8g2log.println(micL255);
#endif
#if USE_PWM_BOARD
        pwm_board::RGBled(0, micR255, micL255);
#endif
    } else {
#if USE_PWM_BOARD
        pwm_board::RGBled(0, micR255, 0);
#endif
    }
}
