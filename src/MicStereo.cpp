//
// Created by mr on 3/24/2024.
//

#include "MicStereo.h"
#include "Arduino.h"
#include "config.h"
#include "logger.h"
#include "main_ra.h"
#include "pwm_board.h"
#include "analog.h"

long MicStereo::baseRSound, MicStereo::baseLSound = 0;
short analog::ext_analog_3, analog::ext_analog_1;

void MicStereo::MicSetup() {
    MicStereo::baseRSound = map(analog::ext_analog_3, 0, 1023, 0, 255); /***** A3 ******/
    MicStereo::baseLSound = map(analog::ext_analog_1, 0, 1023, 0, 255); /***** A1 ******/
	#if LOG_DEBUG
		logger::log(" L-Mic: ");
		if (main::Found_Display) logger::logFloatln(baseLSound);
		logger::log(" R-Mic: ");
		if (main::Found_Display) logger::logFloatln(baseRSound);
	#endif
}


void MicStereo::
MicLoop() {
    int micRStatus = analog::ext_analog_3;
    int micR255 = map(micRStatus, 0, 1023, 0, 255);

    int micLStatus = analog::ext_analog_1;
    int micL255 = map(micLStatus, 0, 1023, 0, 255);

    if (micR255 > MicStereo::baseRSound) {
		#if LOG_DEBUG
			if (main::Found_Display) logger::logIntln(micR255);
		#endif
		if ((USE_PWM_BOARD && !main::use_pwm_board) || main::use_pwm_board) {
			pwm_board::RGBled(micR255, micR255, 0);
		}
	} else if (micL255 > MicStereo::baseLSound) {
		#if LOG_DEBUG
			if (main::Found_Display) logger::logIntln(micL255);
		#endif
		if ((USE_PWM_BOARD && !main::use_pwm_board) || main::use_pwm_board) {
			pwm_board::RGBled(0, micR255, micL255);
		}
	} else {
		if ((USE_PWM_BOARD && !main::use_pwm_board) || main::use_pwm_board) {
			pwm_board::RGBled(0, micR255, 0);
		}
    }
}
