//
// Created by mr on 11/20/2023.
//

#include "barometer.h"
#include <Wire.h>

#include "main_ra.h"
#define SEALEVELPRESSURE_HPA (1013.25)

/********************************************** control ultrasonic sensor***************************************/
// section BaroMeter
/***************************************************************************************************************/
Adafruit_BMP280 barometer::bmp; // I2C

void barometer::baroSetup() {
	if (!barometer::bmp.begin()) {
		if(main::log_debug) {
			main::logln("No valid BMP280 sensor,\n check wiring or try a different address!");
			main::log("SensorID was: 0x");
			main::logHexln(barometer::bmp.sensorID(), 16);
		}
		main::use_barometer = false;
		delay(500);
	} else {
		main::logln(" Barometer initialized");
	}

	/* Default settings from datasheet. */
	barometer::bmp.setSampling(
			Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
			Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
			Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
			Adafruit_BMP280::FILTER_X16,      /* Filtering. */
			Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void barometer::baroMeter() {
	if (bmp.takeForcedMeasurement()) {
		// can now print out the new measurements
		main::log(("Temp = "));
		float temp = (bmp.readTemperature());
		main::logFloat(temp);
		main::logln(" *C");

		main::log(("Press = "));
		float press = (bmp.readPressure() / 100);
		main::logFloat(press);
		main::logln(" Pa");

		main::log(("Alt = "));
		float altit = (bmp.readAltitude(SEALEVELPRESSURE_HPA)/100);
		main::logFloat(altit); /* Adjusted to local forecast! */
		main::logln(" m");

		main::logln();
		delay(2000);
	} else {
		main::logln("Forced measurement failed!");
	}
}
