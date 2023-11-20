//
// Created by mr on 11/17/2023.
//

#include "compass.h"

#include "pesto_matrix.h"
#include "displayU8G2.h"

Adafruit_HMC5883_Unified compass::mag;
double compass::readCompass(){
    displayU8G2::display.print("Compass ");
    sensors_event_t event; /// Get a new sensor event */
    mag.getEvent(&event);

    double heading = atan2(event.magnetic.y, event.magnetic.x);
    double declinationAngle = 0.035;
    heading += declinationAngle;

    if(heading < 0) {
        heading += 2 * PI;
    }
    if(heading > 2*PI) {
        heading -= 2 * PI;
    }
    double headingDegrees = (heading * 180/M_PI) - 90;
    return (headingDegrees < 0) ? 360 + headingDegrees : headingDegrees;
}

void compass::showCompass(){
    unsigned char north[] =  {0x00,0x00,0x00,0x00,0x00,0x7e,0x04,0x08,0x10,0x20,0x7e,0x00,0x00,0x00,0x00,0x00};
    unsigned char east[]  =  {0x00,0x00,0x00,0x00,0x00,0xfe,0x92,0x92,0x92,0x92,0x82,0x00,0x00,0x00,0x00,0x00};
    unsigned char south[] =  {0x00,0x00,0x00,0x00,0x00,0x9e,0x92,0x92,0x92,0x92,0xf2,0x00,0x00,0x00,0x00,0x00};
    unsigned char west[]  =  {0x00,0x00,0x00,0x3c,0x40,0x40,0x40,0x78,0x78,0x40,0x40,0x40,0x3c,0x00,0x00,0x00};

    double headingDegrees = readCompass();
    displayU8G2::display.print(headingDegrees);
    if (headingDegrees >= 0 && headingDegrees < 45){
        Pesto::matrix_display(north);
        displayU8G2::display.print("North       ");
    }
    if (headingDegrees >= 45 && headingDegrees < 135){
        Pesto::matrix_display(east);
        displayU8G2::display.print("East        ");
    }
    if (headingDegrees >= 135 && headingDegrees < 225){
        Pesto::matrix_display(south);
        displayU8G2::display.print("South       ");
    }
    if (headingDegrees >= 225 && headingDegrees < 315){
        Pesto::matrix_display(west);
        displayU8G2::display.print("West        ");
    }
    if (headingDegrees >= 315 && headingDegrees < 360){
        Pesto::matrix_display(north);
        displayU8G2::display.print("North       ");
    }
}

void compass::compassSetup() {
    /* Initialise the sensor */
    if(!mag.begin()) {

        displayU8G2::display.print("HMC5883 not found   ");
        delay(500);

    } else {

        displayU8G2::display.print("HMC5883 Found!     ");
        delay(500);
    }
}