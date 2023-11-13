//
// Created by mr on 11/13/2023.
//
#include <Arduino.h>
#include "motor.h"

/********************************************** the function to run motor **************************************/
// section motor function
/***************************************************************************************************************/

void Motor::Car_front(){
    digitalWrite(L_ROT,HIGH);
    analogWrite(L_PWM,200);
    digitalWrite(R_ROT,HIGH);
    analogWrite(R_PWM,200);
    delay(10);
}

void Motor::Car_left(){
    digitalWrite(L_ROT,LOW);
    analogWrite(L_PWM,255);
    digitalWrite(R_ROT,HIGH);
    analogWrite(R_PWM,255);
    delay(10);
}
void Motor::Car_right(){
    digitalWrite(L_ROT,HIGH);
    analogWrite(L_PWM,255);
    digitalWrite(R_ROT,LOW);
    analogWrite(R_PWM,255);
    delay(10);
}
void Motor::Car_Stop(){
    digitalWrite(L_ROT,LOW);
    analogWrite(L_PWM,0);
    digitalWrite(R_ROT,LOW);
    analogWrite(R_PWM,0);
    delay(10);
}

void Motor::Car_Back(){
    digitalWrite(L_ROT,LOW);
    analogWrite(L_PWM,200);
    digitalWrite(R_ROT,LOW);
    analogWrite(R_PWM,200);
    delay(10);
}
