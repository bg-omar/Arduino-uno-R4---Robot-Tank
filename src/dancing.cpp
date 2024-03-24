//
// Created by mr on 11/18/2023.
//

#include "dancing.h"

#include "pwm_board.h"
#include "PS4.h"


/********************************************** arbitrary sequence *********************************************/
// section Dance
/***************************************************************************************************************/
long  dancing::randomXY, dancing::randomZ;
int flag = 0;

void dancing::dance() {
    while (flag == 0) {
        pwm_board::RainbowColor();
        randomXY = random(1, 180);
        randomZ = random(1, 160);
        pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(randomXY));
        delay(500);
        pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(randomZ));
        delay(500);

        // One pixel, row by row
        randomXY = random(1, 180);
        randomZ = random(1, 160);
        pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(randomXY));
        delay(500);
        pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(randomZ));
        delay(500);


        randomXY = random(1, 180);
        randomZ = random(1, 160);
        pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(randomXY));
        delay(500);
        pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(randomZ));
        delay(500);
        flag = PS4::exitLoop();
    }
}



