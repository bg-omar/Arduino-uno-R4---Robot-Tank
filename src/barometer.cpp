//
// Created by mr on 11/20/2023.
//

#include "barometer.h"
#include <Wire.h>

#include "main_ra.h"
#include "logger.h"
#define SEALEVELPRESSURE_HPA (1013.25)

/********************************************** control ultrasonic sensor***************************************/
// section BaroMeter
/***************************************************************************************************************/
Adafruit_BMP280 barometer::bmp; // I2C

void barometer::baroSetup() {
	if (!barometer::bmp.begin()) {
		if(main::log_debug) {
			logger::logln("No valid BMP280 sensor,\n check wiring or try a different address!");
			logger::log("SensorID was: 0x");
			logger::logHexln(barometer::bmp.sensorID(), 16);
		}
		main::use_barometer = false;
		delay(500);
	} else {
		logger::logln(" Barometer initialized");
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
		logger::log(("Temp = "));
		float temp = (bmp.readTemperature());
		logger::logFloat(temp);
		logger::logln(" *C");

		logger::log(("Press = "));
		float press = (bmp.readPressure() / 100);
		logger::logFloat(press);
		logger::logln(" Pa");

		logger::log(("Alt = "));
		float altit = (bmp.readAltitude(SEALEVELPRESSURE_HPA)/100);
		logger::logFloat(altit); /* Adjusted to local forecast! */
		logger::logln(" m");

		logger::logln("");
		delay(2000);
	} else {
		logger::logln("Forced measurement failed!");
	}
}
