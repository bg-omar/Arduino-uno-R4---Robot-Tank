//
// Created by mr on 11/18/2023.
//

#include "avoid_objects.h"

#include "motor.h"
#include "pwm_board.h"
#include "PS4.h"

/********************************************** control ultrasonic sensor***************************************/
// section UltraSonic
/***************************************************************************************************************/
double  avoid_objects::distanceF,  avoid_objects::distanceR,  avoid_objects::distanceL = 0;
long  avoid_objects::random2 = 0;

double avoid_objects::checkDistance() {
    digitalWrite(Trig_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig_PIN, LOW);
    double checkDistance = pulseIn(Echo_PIN, HIGH) / 58.00;  //58.20, that is, 2*29.1=58.2
    delay(10);
    return checkDistance;
}

void avoid_objects::avoid() {
    int flag = 0; ///the design that enter obstacle avoidance function
    while (flag == 0) {
        pwm_board::RainbowColor();
        random2 = random(1, 100);
        distanceF = checkDistance();
        if (distanceF < 25) {
            analogWrite(LAZER_PIN, 255);
            Motor::Car_Stop(); /// robot stops
            pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(115));
            delay(10); ///delay in 200ms
            pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(90));
            delay(10); ///delay in 200ms
            analogWrite(LAZER_PIN, 0);
            pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(160)); /// look left
            for (int j = 1;
                 j <= 10; j = j + (1)) { ///  the data will be more accurate if sensor detect a few times.
                distanceL = checkDistance();
            }
            delay(200);
            pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(20)); /// look right
            for (int k = 1; k <= 10; k = k + (1)) {
                distanceR = checkDistance();
            }
            if (distanceL < 50 || distanceR < 50) {
                if (distanceL > distanceR) {
                    pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(90));
                    Motor::Car_left();
                    delay(500); ///turn left 500ms
                    Motor::Car_front();
                } else {
                    pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(90));
                    Motor::Car_right();
                    delay(500);
                    Motor::Car_front();
                }
            } else {  /// not (distanceL < 50 || distanceR < 50)
                if ((long) (random2) % (long) (2) == 0) ///when the random number is even
                {
                    pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(90));
                    Motor::Car_left(); ///robot turns left
                    delay(500);
                    Motor::Car_front(); ///go forward
                } else {
                    pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(90));
                    Motor::Car_right(); ///robot turns right
                    delay(500);
                    Motor::Car_front(); ///go forward
                }
            }
        } else /// if (distanceF < 25) { If the front distance is greater than or equal, robot car will go forward
        {
            Motor::Car_front();
        }
        flag = PS4::exitLoop();
    }
}