//
// Created by mr on 11/17/2023.
//

#include "gyroscope.h"
#include "displayU8G2.h"
#include "timers.h"
#include "PS4.h"

Adafruit_MPU6050 mpu; // Set the gyroscope
void gyroscope::gyroRead(){
    sensors_event_t a, gyro, temp;
    mpu.getEvent(&a, &gyro, &temp);

    gyroscope::temperature = temp.temperature;
    gyroscope::gyroscope::ax = a.acceleration.x - gyroscope::baseAx;
    gyroscope::ay = a.acceleration.y - gyroscope::baseAy;
    gyroscope::az = a.acceleration.z - gyroscope::baseAz;
    gyroscope::gx = gyro.gyro.x - gyroscope::baseGx;
    gyroscope::gy = gyro.gyro.y - gyroscope::baseGy;
    gyroscope::gz = gyro.gyro.z - gyroscope::baseGz;
}

void gyroscope::gyroFunc(){
    gyroscope::gyroRead();
    (gyroscope::ax > 0) ? displayU8G2::display.print("+"), displayU8G2::display.print(gyroscope::ax) : displayU8G2::display.print(gyroscope::ax);
    displayU8G2::display.print(" ");
    (gyroscope::ay > 0) ? displayU8G2::display.print("+"), displayU8G2::display.print(gyroscope::ay) : displayU8G2::display.print(gyroscope::ay);
    displayU8G2::display.print(" ");
    (gyroscope::az > 0) ? displayU8G2::display.print("+"), displayU8G2::display.print(gyroscope::az) : displayU8G2::display.print(gyroscope::az);
    displayU8G2::display.print("   ");

    (gyroscope::gx > 0) ? displayU8G2::display.print("+"), displayU8G2::display.print(gyroscope::gx) : displayU8G2::display.print(gyroscope::gx);
    displayU8G2::display.print(" ");
    (gyroscope::gy > 0) ? displayU8G2::display.print("+"), displayU8G2::display.print(gyroscope::gy) : displayU8G2::display.print(gyroscope::gy);
    displayU8G2::display.print(" ");
    (gyroscope::gz > 0) ? displayU8G2::display.print("+"), displayU8G2::display.print(gyroscope::gz) : displayU8G2::display.print(gyroscope::gz);
    displayU8G2::display.print("   ");
}

void gyroscope::gyroDetectMovement() {
#if USE_TIMERS
    gyroscope::gyroRead();
            if(( abs(gyroscope::ax) + abs(gyroscope::ay) + abs(gyroscope::az)) > THRESHOLD){
                timers::timerTwoActive = true;
                timers::timerTreeActive = true;
                timers::timerButton = 2200; // PS4::R1;
            }
            if(( abs(gyroscope::gx) + abs(gyroscope::gy) + abs(gyroscope::gz)) > THRESHOLD){
                timers::timerTwoActive = true;
                timers::timerTreeActive = true;
                timers::timerButton = 2100; // PS4::L1;
            }

#endif
}
void gyroscope::gyroCalibrate_sensor() {
    float totX = 0;
    float totY = 0;
    float totZ = 0;
    float totgX = 0;
    float totgY = 0;
    float totgZ = 0;
    sensors_event_t a, gyro, temp;
    delay(10);
    for (size_t i = 0; i < 10; i++) {
        mpu.getEvent(&a, &gyro, &temp);
        delay(10);
        totX += a.acceleration.x;
        delay(10);
        totY += a.acceleration.y;
        delay(10);
        totZ += a.acceleration.z;
        delay(10);
        totgX += gyro.gyro.x;
        delay(10);
        totgY += gyro.gyro.y;
        delay(10);
        totgZ += gyro.gyro.z;
        delay(10);
    }
    gyroscope::baseAx = totX / 10;
    gyroscope::baseAy = totY / 10;
    gyroscope::baseAz = totZ / 10;
    gyroscope::baseGx = totgX / 10;
    gyroscope::baseGy = totgY / 10;
    gyroscope::baseGz = totgZ / 10;
}

void gyroscope::gyroSetup() {
    // Try to initialize!
    if (!mpu.begin()) {

        displayU8G2::display.print("MPU6050 not found");
        delay(500);

    } else {
        displayU8G2::display.print("MPU6050 Found!    ");
        delay(500);
        mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_260_HZ); /// 5, 10, 21, 44, 94, 184, 260(off)
        gyroscope::gyroCalibrate_sensor();
        delay(500);
    }
}