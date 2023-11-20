//
// Created by mr on 11/20/2023.
//

#ifndef ARDUINO_R4_UNO_WALL_Z_BAROMETER_H
#define ARDUINO_R4_UNO_WALL_Z_BAROMETER_H

#define SEALEVELPRESSURE_HPA (1013.25)

#include <Adafruit_BME280.h>


class barometer {
public:
    static Adafruit_BME280 bme;
    static void baroSetup();
    static void baroMeter();
};


#endif //ARDUINO_R4_UNO_WALL_Z_BAROMETER_H
