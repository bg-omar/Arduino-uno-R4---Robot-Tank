//
// Created by mr on 11/20/2023.
//

#include "barometer.h"
#include "displayAdafruit.h"
#include "displayU8G2.h"
#include "pwm_board.h"

/********************************************** control ultrasonic sensor***************************************/
// section BaroMeter
/***************************************************************************************************************/
Adafruit_BME280 barometer::bme;
void barometer::baroSetup() {
        displayAdafruit::display.clearDisplay();
        /* Initialise the sensor */
        if (!bme.begin(0x76)) {
#if LOG_DEBUG
            displayU8G2::u8g2log.println("BME280,not found!");
#endif
            delay(500);
        } else {
#if LOG_DEBUG
            displayU8G2::u8g2log.println("BME280 Found!     ");
#endif
            delay(500);
        }
}

void barometer::baroMeter() {
    displayAdafruit::display.clearDisplay();
    pwm_board::rightLedStrip(0,0,244);
    pwm_board::leftLedStrip(0,0,244);
#if LOG_DEBUG
    displayU8G2::u8g2log.print("Temp= ");
    displayU8G2::u8g2log.print(bme.readTemperature());
    displayU8G2::u8g2log.print("*C ");

    displayU8G2::u8g2log.print("P= ");
    displayU8G2::u8g2log.print(bme.readPressure() / 100.0F);
    displayU8G2::u8g2log.println("hPa");


    displayU8G2::u8g2log.print("Alt= ");
    displayU8G2::u8g2log.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    displayU8G2::u8g2log.print("m ");

    displayU8G2::u8g2log.print("H= ");
    displayU8G2::u8g2log.print(bme.readHumidity());
    displayU8G2::u8g2log.println("%");
#endif
    delay(500);


}
