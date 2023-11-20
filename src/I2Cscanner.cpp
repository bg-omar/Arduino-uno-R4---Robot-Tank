//
// Created by mr on 11/20/2023.
//

#include "I2Cscanner.h"
#include "displayU8G2.h"
#include "displayAdafruit.h"


void I2Cscanner::scan() {
    byte error, address;
    int nDevices;

    displayU8G2::display.println("I2C Scanning...");
#if USE_ADAFRUIT
    displayAdafruit::display.display();
#endif
    nDevices = 0;

    delay(200);
    for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            displayU8G2::display.print(" 0x");
            if (address<16) {
                displayU8G2::display.print("0");
            }
            displayU8G2::display.println(address, HEX);
#if USE_ADAFRUIT
            displayAdafruit::display.display();
#endif
            nDevices++;
            delay(200);
        }
        else if (error==4) {
            displayU8G2::display.print("Unknow error at address 0x");
            if (address<16) {

                displayU8G2::display.print("0");
            }
            displayU8G2::display.println(address, HEX);
#if USE_ADAFRUIT
            displayAdafruit::display.display();
#endif
        }
    }
    delay(20);
    displayU8G2::display.print(nDevices);
    delay(20);
    displayU8G2::display.println(" devices");
#if USE_ADAFRUIT
    displayAdafruit::display.display();
#endif
    delay(100);
    if (nDevices == 0) {
        displayU8G2::display.println("No I2C devices found");
#if USE_ADAFRUIT
        displayAdafruit::display.display();
#endif
    }
}