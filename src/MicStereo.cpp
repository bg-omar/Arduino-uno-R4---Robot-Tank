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

int sampleWindow = 50;  // Sample window width in milliseconds
unsigned int sampleL, sampleR;
unsigned long startMillis;
unsigned int peakToPeak, peakToPeakL = 0;

int MicStereo::baseRSound, MicStereo::baseLSound = 0;
short analog::ext_analog_3, analog::ext_analog_1;

void MicStereo::MicSetup() {
    MicStereo::baseRSound = analog::ext_analog_3; /***** A3 ******/
    MicStereo::baseLSound = analog::ext_analog_1; /***** A1 ******/
	#if LOG_DEBUG
		logger::log(" L-Mic: ");
		if (main::Found_Display) logger::logInt(baseLSound);
		logger::log(" R-Mic: ");
		if (main::Found_Display) logger::logIntln(baseRSound);
	#endif
}


void MicStereo::MicLoop() {
	unsigned int signalMaxL = 0;
	unsigned int signalMinL = 1024;

	unsigned int signalMaxR = 0;
	unsigned int signalMinR = 1024;
	startMillis = millis();


    int micRStatus = analog::ext_analog_3;
   	unsigned int micR255 = map(micRStatus, 0, 1023, 0, 255);

    int micLStatus = analog::ext_analog_1;
	unsigned int micL255 = map(micLStatus, 0, 1023, 0, 255);

	while (millis() - startMillis < sampleWindow) {
		sampleL = analog::ext_analog_1;
		sampleR = analog::ext_analog_3;
		if (sampleL < 1024) {
			if (sampleL > signalMaxL) signalMaxL = sampleL;
			if (sampleL < signalMinL) signalMinL = sampleL;
		}
		if (sampleR < 1024) {
			if (sampleR > signalMaxR) signalMaxR = sampleR;
			if (sampleR < signalMinR) signalMinR = sampleR;
		}
	}


    if (micR255 > MicStereo::baseRSound) {
		#if LOG_DEBUG
			if (main::Found_Display) {
				logger::log(" R-Mic: ");
				logger::logInt(micR255); }
		#endif
		if ((USE_PWM_BOARD && !main::use_pwm_board) || main::use_pwm_board) {
			pwm_board::RGBled(micR255, micR255, 0);
		}
	}
	if (micL255 > MicStereo::baseLSound) {
		#if LOG_DEBUG
			if (main::Found_Display) {
				logger::log(" L-Mic: ");
				logger::logIntln(micL255); }
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
