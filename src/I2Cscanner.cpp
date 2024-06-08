//
// Created by mr on 11/20/2023.
//

#include "displayU8G2.h"
#include "displayAdafruit.h"
#include "main_ra.h"
#include "I2Cscanner.h"


void I2Cscanner::scan() {
    byte error, address;
    int nDevices;

    main::logln("I2C Scanning...");
    nDevices = 0;
	main::log("MOSI: ", 0);
	main::logln(reinterpret_cast<const char *>(MOSI));
	main::log("MISO: ", 0);
	main::logln(reinterpret_cast<const char *>(MISO));
	main::log("SCK: ", 0);
	main::logln(reinterpret_cast<const char *>(SCK));
	main::log("SS: ", 0);
	main::logln(reinterpret_cast<const char *>(SS));

	main::log("SDA: ", 0);
	main::logln(reinterpret_cast<const char *>(SDA));
	main::log("SCL: ", 0);
	main::logln(reinterpret_cast<const char *>(SCL));

    delay(200);
    for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
			main::log(" 0x", 0);
            if (address<16) {
				main::log("0", 0);
            }
#if LOG_DEBUG
            if (main::Found_Display) displayU8G2::u8g2log.print(address, HEX);
#endif
			main::log(reinterpret_cast<const char *>(address), HEX);
			main::log(", ", 0);
            nDevices++;
            delay(200);
        }
        else if (error==4) {
			main::log("Unknow error at address 0x", 0);
            if (address<16) {
				main::log("0 ", 0);
            }
#if LOG_DEBUG
            if (main::Found_Display) displayU8G2::u8g2log.print(address, HEX);
#endif
            main::log(reinterpret_cast<const char *>(address), HEX);
        }
    }
    delay(20);
	main::log(" devices: ", 0);
    main::logln(reinterpret_cast<const char *>(nDevices));
#if LOG_DEBUG
    displayU8G2::u8g2log.println(nDevices);
#endif

    delay(100);
    if (nDevices == 0) {
        main::logln("-- No I2C devices found--");
    }
}