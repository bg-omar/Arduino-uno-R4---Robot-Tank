//
// Created by mr on 11/20/2023.
//

#include "logger.h"
#include "main_ra.h"
#include "I2Cscanner.h"
#include <Wire.h>

void I2Cscanner::scan() {
    byte error, address;
    int nDevices;

    logger::logln("I2C Scanning...");
    nDevices = 0;

    delay(200);
    for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
			logger::log(" 0x");
            if (address<16) {
				logger::log("0");
            }
#if LOG_DEBUG
            if (main::Found_Display) logger::logHexln(address, HEX);
#endif
			logger::log(reinterpret_cast<const char *>(address));
			logger::log(", ");
            nDevices++;
            delay(200);
        }
        else if (error==4) {
			logger::log("Unknown error at address 0x");
            if (address<16) {
				logger::log("0 ");
            }
#if LOG_DEBUG
            if (main::Found_Display) logger::logHexln(address, HEX);
#endif
			logger::log(reinterpret_cast<const char *>(address));
        }
    }
    delay(20);
	logger::log(" devices: ");
    logger::logln(reinterpret_cast<const char *>(nDevices));

    delay(100);
    if (nDevices == 0) {
        logger::logln("-- No I2C devices found--");
    }
}