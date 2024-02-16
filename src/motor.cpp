//
// Created by mr on 11/13/2023.
//
#include <Arduino.h>
#include "motor.h"

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
    // Serial.println('R');
    digitalWrite(L_ROT,HIGH);
    analogWrite(L_PWM,255);
    digitalWrite(R_ROT,HIGH);
    analogWrite(R_PWM,255);
}
void Motor::Car_Stop(){
    Serial.println('Stop');
    digitalWrite(L_ROT,LOW);
    analogWrite(L_PWM,0);
    digitalWrite(R_ROT,HIGH);
    analogWrite(R_PWM,0);
}

void Motor::Car_Back(){
    // Serial.println('B');
    digitalWrite(L_ROT,LOW);
    analogWrite(L_PWM,200);
    digitalWrite(R_ROT,HIGH);
    analogWrite(R_PWM,200);
}
