//
// Created by mr on 11/20/2023.
//

#include "I2Cscanner.h"
#include "displayU8G2.h"
#include "displayAdafruit.h"
#include "main_ra.h"

void I2Cscanner::scan() {
    byte error, address;
    int nDevices;
#if USE_U8G2
    main::log("I2C Scanning...");
#endif
#if USE_ADAFRUIT
    displayAdafruit::display.println("I2C Scanning...");
    displayAdafruit::display.display();
#endif
    nDevices = 0;

    delay(200);
    for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            #if USE_U8G2
                        main::log(" 0x");
            #endif
            #if USE_ADAFRUIT
                        displayAdafruit::display.print(" 0x");
            #endif
            if (address<16) {
                #if USE_U8G2
                main::log("0");
                #endif
#if USE_ADAFRUIT
                displayAdafruit::display.print("0");
#endif
            }
#if USE_U8G2
            main::log(reinterpret_cast<const char *>(address, HEX));
#endif
#if USE_ADAFRUIT
            displayAdafruit::display.println(address, HEX);
            displayAdafruit::display.display();
#endif
            nDevices++;
            delay(200);
        }
        else if (error==4) {
            main::log("Unknow error at address 0x");
            if (address<16) {

                main::log("0");
            }
            main::log(reinterpret_cast<const char *>(address, HEX));
#if USE_ADAFRUIT
            displayAdafruit::display.println(address, HEX);
            displayAdafruit::display.display();
#endif
        }
    }
#if USE_U8G2
    delay(20);
    main::log(reinterpret_cast<const char *>(nDevices));
    delay(20);
    main::log(" devices");
#endif
#if USE_ADAFRUIT
    delay(20);
    displayAdafruit::display.print(nDevices);
    delay(20);
    displayAdafruit::display.println(" devices");
    displayAdafruit::display.display();
#endif
    delay(100);
    if (nDevices == 0) {
#if USE_U8G2
        main::log("No I2C devices found");
#endif
#if USE_ADAFRUIT
        displayAdafruit::display.println("No I2C devices found");
        displayAdafruit::display.display();
#endif
    }
}