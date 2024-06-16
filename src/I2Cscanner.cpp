//
// Created by mr on 11/20/2023.
//

#include "displayU8G2.h"
#include "main_ra.h"
#include "I2Cscanner.h"
#include <Wire.h>

void I2Cscanner::scan() {
    byte error, address;
    int nDevices;

    main::logln("I2C Scanning...");
    nDevices = 0;

    delay(200);
    for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
			main::log(" 0x");
            if (address<16) {
				main::log("0");
            }
#if LOG_DEBUG
            if (main::Found_Display) main::logHexln(address, HEX);
#endif
			main::log(reinterpret_cast<const char *>(address));
			main::log(", ");
            nDevices++;
            delay(200);
        }
        else if (error==4) {
			main::log("Unknown error at address 0x");
            if (address<16) {
				main::log("0 ");
            }
#if LOG_DEBUG
            if (main::Found_Display) main::logHexln(address, HEX);
#endif
			main::log(reinterpret_cast<const char *>(address));
        }
    }
    delay(20);
	main::log(" devices: ");
    main::logln(reinterpret_cast<const char *>(nDevices));

    delay(100);
    if (nDevices == 0) {
        main::logln("-- No I2C devices found--");
    }
}