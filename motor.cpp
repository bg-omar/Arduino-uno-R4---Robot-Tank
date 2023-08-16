#include <Arduino.h>

#define ML_PWM     11  // define PWM control pin of left motor
#define MR_Ctrl    12  // define the direction control pin of right motor
#define MR_PWM     3   // define PWM control pin of right motor
#define ML_Ctrl    13  // define the direction control pin of left motor

void Car_front(){
    digitalWrite(MR_Ctrl,HIGH);
    analogWrite(MR_PWM,255);
    digitalWrite(ML_Ctrl,HIGH);
    analogWrite(ML_PWM,255);
}

void Car_left(){
    digitalWrite(MR_Ctrl,LOW);
    analogWrite(MR_PWM,255);
    digitalWrite(ML_Ctrl,HIGH);
    analogWrite(ML_PWM,255);
}
void Car_right(){
    digitalWrite(MR_Ctrl,HIGH);
    analogWrite(MR_PWM,255);
    digitalWrite(ML_Ctrl,LOW);
    analogWrite(ML_PWM,255);
}
void Car_Stop(){
    digitalWrite(MR_Ctrl,LOW);
    analogWrite(MR_PWM,0);
    digitalWrite(ML_Ctrl,LOW);
    analogWrite(ML_PWM,0);
}

void Car_Back(){
    digitalWrite(MR_Ctrl,LOW);
    analogWrite(MR_PWM,255);
    digitalWrite(ML_Ctrl,LOW);
    analogWrite(ML_PWM,255);
}