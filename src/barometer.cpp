//
// Created by mr on 11/20/2023.
//

#include "barometer.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#include "main_ra.h"


/********************************************** control ultrasonic sensor***************************************/
// section BaroMeter
/***************************************************************************************************************/
Adafruit_BMP280 barometer::bmp; // I2C
Adafruit_Sensor *bmp_temp = barometer::bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = barometer::bmp.getPressureSensor();

void barometer::baroSetup() {
	unsigned status;
	//status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
	status = bmp.begin(0x58);
	if (!status) {
		#if LOG_DEBUG
		main::logln(reinterpret_cast<const char *>(F("Could not find a valid BMP280 sensor, check wiring or "
													 "try a different address!")));
		main::log("SensorID was: 0x"); main::logHexln(bmp.sensorID(),16);
		main::log("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
		main::log("   ID of 0x56-0x58 represents a BMP 280,\n");
		main::log("        ID of 0x60 represents a BME 280.\n");
		main::log("        ID of 0x61 represents a BME 680.\n");


		#endif
		main::use_barometer = false;
		delay(500);
	}

	/* Default settings from datasheet. */
	bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
					Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
					Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
					Adafruit_BMP280::FILTER_X16,      /* Filtering. */
					Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

	bmp_temp->printSensorDetails();
}

void barometer::baroMeter() {
	sensors_event_t temp_event, pressure_event;
	bmp_temp->getEvent(&temp_event);
	bmp_pressure->getEvent(&pressure_event);

	main::log(("Temperature = "));
	main::log(reinterpret_cast<const char *>(char(temp_event.temperature)));
	main::logln(" *C");

	main::log(("Pressure = "));
	main::log(reinterpret_cast<const char *>(char(pressure_event.pressure)));
	main::logln(" hPa");

	main::logln();
	delay(2000);



}
