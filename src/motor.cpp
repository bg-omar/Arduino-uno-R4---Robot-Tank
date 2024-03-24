//
// Created by mr on 11/13/2023.
//
#include <Arduino.h>
#include "motor.h"
#include "main_ra.h"

/********************************************** the function to run motor **************************************/
// section motor function
/***************************************************************************************************************/

void Motor::Car_front(){
    // Serial.println('F');
    digitalWrite(L_ROT,HIGH);
    analogWrite(L_PWM,200);
    digitalWrite(R_ROT,LOW);
    analogWrite(R_PWM,200);
}

void Motor::Car_left(){
    // Serial.println('L');
    digitalWrite(L_ROT,LOW);
    analogWrite(L_PWM,255);
    digitalWrite(R_ROT,LOW);
    analogWrite(R_PWM,255);
}
void Motor::Car_right(){
    //Serial.println('R');
    digitalWrite(L_ROT,HIGH);
    analogWrite(L_PWM,255);
    digitalWrite(R_ROT,HIGH);
    analogWrite(R_PWM,255);
}
void Motor::Car_Stop(){
    //Serial.println("Stop");
    digitalWrite(L_ROT,LOW);
    analogWrite(L_PWM,0);
    digitalWrite(R_ROT,HIGH);
    analogWrite(R_PWM,0);
}

void Motor::Car_Back(){
    //Serial.println('B');
    digitalWrite(L_ROT,LOW);
    analogWrite(L_PWM,200);
    digitalWrite(R_ROT,HIGH);
    analogWrite(R_PWM,200);
}

void Motor::motor_setup() {
    pinMode(R_ROT, OUTPUT);     /***** 13 ******/
    pinMode(R_PWM, OUTPUT);      /***** 11 ******/
    pinMode(L_ROT, OUTPUT);     /***** 12 ******/
    pinMode(L_PWM, OUTPUT);      /***** 3 ******/
    digitalWrite(R_ROT, HIGH);
    digitalWrite(L_ROT, HIGH);
    main::logln("Motor initialized \n Pins R: 11, 13 \t L: 3, 12  ");
    delay(500);
}
