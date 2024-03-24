//
// Created by mr on 11/13/2023.
//

#include <Arduino.h>
#include "pwm_board.h"
#include "PS4.h"


/***************************************************** Servo PWM Angle s**********************************************/
// section Servo PWM Angle
/***************************************************************************************************************/

Adafruit_PWMServoDriver pwm_board::pwm = Adafruit_PWMServoDriver();

int pwm_board::r = 0;
int pwm_board::g = 0;
int pwm_board::b = 0;
int pwm_board::a = 0;


void pwm_board::setupPWM(){
    pwm_board::pwm.begin();
    pwm_board::pwm.setPWMFreq(FREQUENCY);  // Analog servos run at ~50 Hz updates
    pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(PS4::posXY));
    pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(PS4::posZ));
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
    pwm.setPWM(PWM_12, 0, (a*b_val<4080) ? a*b_val : 4080);
    pwm.setPWM(PWM_13, 0, (a*g_val<4080) ? a*g_val : 4080);
    pwm.setPWM(PWM_14, 0, (a*r_val<4080) ? a*r_val : 4080);
}

void pwm_board::leftLedStrip(int r_val, int g_val, int b_val) {
    r_val = 255 - r_val;
    g_val = 255 - g_val;
    b_val = 255 - b_val;
    pwm.setPWM(PWM_8, 0, (a*b_val<4080) ? a*b_val : 4080);
    pwm.setPWM(PWM_9, 0, (a*g_val<4080) ? a*g_val : 4080);
    pwm.setPWM(PWM_10, 0, (a*r_val<4080) ? a*r_val : 4080);
}

void pwm_board::rightLedStrip(int r_val, int g_val, int b_val) {
    r_val = 255 - r_val;
    g_val = 255 - g_val;
    b_val = 255 - b_val;
    pwm.setPWM(PWM_5, 0, (a*b_val<4080) ? a*b_val : 4080);
    pwm.setPWM(PWM_6, 0, (a*g_val<4080) ? a*g_val : 4080);
    pwm.setPWM(PWM_7, 0, (a*r_val<4080) ? a*r_val : 4080);
}


void  pwm_board::RainbowColor() {
    if (pwm_board::r > 0 && pwm_board::b == 0) {
        pwm_board::r--;
        pwm_board::g++;
    }
    if (pwm_board::g > 0 && pwm_board::r == 0) {
        pwm_board::g--;
        pwm_board::b++;
    }
    if (pwm_board::b > 0 && pwm_board::g == 0) {
        pwm_board::r++;
        pwm_board::b--;
    }
    pwm_board::rightLedStrip(pwm_board::r,pwm_board::g,pwm_board::b);
    pwm_board::leftLedStrip(pwm_board::r,pwm_board::g,pwm_board::b);
}