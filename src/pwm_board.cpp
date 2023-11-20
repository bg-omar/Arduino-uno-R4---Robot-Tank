//
// Created by mr on 11/13/2023.
//

#include <Arduino.h>
#include "pwm_board.h"


/***************************************************** Servo PWM Angle s**********************************************/
// section Servo PWM Angle
/***************************************************************************************************************/

Adafruit_PWMServoDriver pwm_board::pwm = Adafruit_PWMServoDriver();

void pwm_board::setupPWM(){
    pwm_board::pwm.begin();
    pwm_board::pwm.setPWMFreq(FREQUENCY);  // Analog servos run at ~50 Hz updates
    pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(90));
    pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(45));
}


int pwm_board::pulseWidth(int angle){  //  pwm.setPWM(PWM_0, 0, pulseWidth(0));
    int pulse_wide, analog_value;
    pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
    return analog_value;
}

/***************************************************** Servo PWM Angle s**********************************************/
// section RGBled
/***************************************************************************************************************/

void pwm_board::RGBled(int r_val, int g_val, int b_val) {
		pwm.setPWM(PWM_8, 0, (16*b_val<4080) ? 16*b_val : 4080);
		pwm.setPWM(PWM_9, 0, (16*g_val<4080) ? 16*g_val : 4080);
		pwm.setPWM(PWM_10, 0, (16*r_val<4080) ? 16*r_val : 4080);
	}
