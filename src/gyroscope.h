//
// Created by mr on 11/17/2023.
//

#ifndef ARDUINO_R4_UNO_WALL_Z_GYROSCOPE_H
#define ARDUINO_R4_UNO_WALL_Z_GYROSCOPE_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


class gyroscope {
private:

    sensors_event_t a, gyro, temp;
public:
    static float ax,ay, az, gx, gy, gz, baseAx, baseAy, baseAz, baseGx, baseGy, baseGz, temperature;

    static void gyroRead();
    static void gyroFunc();
    static void gyroDetectMovement();
    static void gyroCalibrate_sensor() ;
    static void gyroSetup() ;
};


#endif //ARDUINO_R4_UNO_WALL_Z_GYROSCOPE_H
