#include <Arduino.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu; // Set the gyroscope
float ax, ay, az, gx, gy, gz;
float baseAx, baseAy, baseAz, baseGx, baseGy, baseGz;

void calibrate_sensor();
void detectMovement();
void gyroFunc();

void gyroRead(){
    // Get new sensor events with the readings
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    ax = a.acceleration.x - baseAx;
    ay = a.acceleration.y - baseAy;
    az = a.acceleration.z - baseAz;
    gx = g.gyro.x - baseGx;
    gy = g.gyro.y - baseGy;
    gz = g.gyro.z - baseGz;
}


void calibrate_sensor() {
    float totX = 0;
    float totY = 0;
    float totZ = 0;
    float totgX = 0;
    float totgY = 0;
    float totgZ = 0;
    sensors_event_t a, g, temp;
    delay(10);
    for (size_t i = 0; i < 10; i++) {
        mpu.getEvent(&a, &g, &temp);
        delay(10);
        totX += a.acceleration.x;
        delay(10);
        totY += a.acceleration.y;
        delay(10);
        totZ += a.acceleration.z;
        delay(10);
        totgX += g.gyro.x;
        delay(10);
        totgY += g.gyro.y;
        delay(10);
        totgZ += g.gyro.z;
        delay(10);
    }
    baseAx = totX / 10;
    baseAy = totY / 10;
    baseAz = totZ / 10;
    baseGx = totgX / 10;
    baseGy = totgY / 10;
    baseGz = totgZ / 10;
}