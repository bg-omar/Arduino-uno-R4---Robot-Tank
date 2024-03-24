//
// Created by mr on 3/24/2024.
//
#include "arduino.h"
#include "BLE.h"
#include "motor.h"
#include "Follow_light.h"
#include "avoid_objects.h"
#include "dancing.h"

char bluetooth_val;

void readBLE() {
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
