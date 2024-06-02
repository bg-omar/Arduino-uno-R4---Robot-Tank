//
// Created by mr on 3/24/2024.
//
#include "arduino.h"
#include "BLE.h"
#include "motor.h"
#include "Follow_light.h"
#include "avoid_objects.h"
#include "dancing.h"
#include <SoftwareSerial.h>

SoftwareSerial HM10(0, 1); // RX = 2, TX = 3

char appData; char bluetooth_val;

String inData = "";

void BLE::BLEsetup(){
	Serial.println("HM10 serial started at 115200");

	HM10.begin(115200); // set HM10 serial at 115200 baud rate
	pinMode(13, OUTPUT); // onboard LED
	digitalWrite(13, LOW); // switch OFF LED
}



void BLE::readBLE() {
    if (Serial.available()) {
        bluetooth_val = Serial.read();
        Serial.println(bluetooth_val);
    }
    switch (bluetooth_val) {
        case 'F': ///Forward instruction
            Motor::Car_front();
            break;
        case 'B': ///Back instruction
            Motor::Car_Back();
            break;
        case 'L': ///left-turning instruction
            Motor::Car_left();
            break;
        case 'R': ///right-turning instruction
            Motor::Car_right();
            break;
        case 'S': ///stop instruction
            Motor::Car_Stop();
            break;
        case 'Y':
            Follow_light::light_track();
            break;
        case 'U':
            avoid_objects::avoid();
            break;
        case 'X':
            dancing::dance();
            break;
    }
};





void BLE::BLEloop() {
	BLE::readBLE();
	while (HM10.available() > 0) {   // if HM10 sends something then read
    appData = HM10.read();
    inData = String(appData);  // save the data in string format
    Serial.write(appData);
  }

  if (Serial.available()) {           // Read user input if available.
    delay(10);
    HM10.write(Serial.read());
  }

  if ( inData == "F") {
    Serial.println("LED OFF");
    digitalWrite(13, LOW); // switch OFF LED
    delay(500);
  }

  if ( inData == "N") {
		Serial.println("LED ON");
		digitalWrite(13, HIGH); // switch ON LED
		delay(500);
		digitalWrite(13, HIGH); // switch ON LED
		delay(500);
		digitalWrite(13, LOW); // switch OFF LED
		delay(500);
		digitalWrite(13, HIGH); // switch ON LED
		delay(500);
		digitalWrite(13, LOW); // switch OFF LED
		delay(500);
		digitalWrite(13, HIGH); // switch ON LED
		delay(500);
		digitalWrite(13, LOW); // switch OFF LED
		delay(500);
		digitalWrite(13, HIGH); // switch ON LED
		delay(500);
		digitalWrite(13, LOW); // switch OFF LED
		delay(500);
		digitalWrite(13, HIGH); // switch ON LED
		delay(500);
		digitalWrite(13, LOW); // switch OFF LED
		delay(500);

  }

}
