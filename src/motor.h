//
// Created by mr on 11/13/2023.
//

#ifndef MOTOR_H
#define MOTOR_H

#define L_PWM     3   // define PWM control pin of right motor
#define R_PWM     11  // define PWM control pin of left motor
#define L_ROT     12  // define the direction control pin of right motor
#define R_ROT     13  // define the direction control pin of left motor

class Motor {

public:
    static void Car_front() ;
    static void Car_left();
    static void Car_right() ;
    static void Car_Stop() ;
    static void Car_Back();
};

#endif //MOTOR_H
