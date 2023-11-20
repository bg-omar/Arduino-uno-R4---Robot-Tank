//
// Created by mr on 11/20/2023.
//

#include "barometer.h"
#include "displayAdafruit.h"
#include "displayU8G2.h"

/********************************************** control ultrasonic sensor***************************************/
// section BaroMeter
/***************************************************************************************************************/
Adafruit_BME280 barometer::bme;
void barometer::baroSetup() {
        displayAdafruit::display.clearDisplay();
        /* Initialise the sensor */
        if (!bme.begin(0x76)) {

            displayU8G2::display.print("BME280,not found!");
            delay(500);
        } else {

            displayU8G2::display.print("BME280 Found!     ");
            delay(500);
        }
}

void barometer::baroMeter() {
    displayAdafruit::display.clearDisplay();

    displayU8G2::display.print("Temp= ");
    displayU8G2::display.print(bme.readTemperature());
    displayU8G2::display.print("*C ");

    displayU8G2::display.print("P= ");
    displayU8G2::display.print(bme.readPressure() / 100.0F);
    displayU8G2::display.print("hPa");


    displayU8G2::display.print("Alt= ");
    displayU8G2::display.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    displayU8G2::display.print("m ");

    displayU8G2::display.print("H= ");
    displayU8G2::display.print(bme.readHumidity());
    displayU8G2::display.print("%");
    delay(500);
}
