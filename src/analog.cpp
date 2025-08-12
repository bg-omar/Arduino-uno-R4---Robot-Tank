//
// Created by mr on 4/16/2024.
//

#include "analog.h"
#include "config.h"
#include "main_ra.h"
#include "logger.h"

#include "ADS1X15.h"
#include "Wire.h"

// Declare ADS as a pointer
ADS1015* ADS = nullptr;

void analog::analogSetup()
{
	// Initialize ADS1015 object
	ADS = new ADS1015(0x48);

	if (!ADS->begin()) {
		logger::log("ADS1015 LIB failed");
		main::use_analog = false;
	} else {
		logger::log("ADS1015 LIB loaded");
	}
}

void analog::analogLoop()
{
	if (!ADS) return; // Ensure ADS is initialized

	ADS->setGain(0);
	analog::ext_analog_0 = ADS->readADC(light_L_PIN);
	analog::ext_analog_1 = ADS->readADC(MIC_L_PIN);
	analog::ext_analog_2 = ADS->readADC(light_R_PIN);
	analog::ext_analog_3 = ADS->readADC(MIC_R_PIN);

#if LOG_VERBOSE
	float toVoltage = ADS->toVoltage(1);  // Voltage factor
	logger::log("light_L: ");
	logger::logInt(ext_analog_0);
	logger::log(" ");
	logger::log("light_R: ");
	logger::logInt(ext_analog_2);
	logger::log(" \t");
	logger::log(" MIC_L_PIN: ");
	logger::logInt(ext_analog_1);
	logger::log(" MIC_R_PIN: ");
	logger::logIntln(ext_analog_3);
#endif
}
