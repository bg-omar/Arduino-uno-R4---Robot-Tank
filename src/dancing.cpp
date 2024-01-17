//
// Created by mr on 11/18/2023.
//

#include "dancing.h"

#include "pwm_board.h"


/********************************************** arbitrary sequence *********************************************/
// section Dance
/***************************************************************************************************************/
long  dancing::randomXY, dancing::randomZ;
int flag = 0;

int dancing::exitLoop() {
    if (Serial1.available()) {
        static char message[30]; // Create char for serial1 message
        static unsigned int message_pos = 0;
        char inByte = Serial1.read();
        if (inByte != '\n' && (message_pos < 30 - 1)) { // Add the incoming byte to our message
            message[message_pos] = inByte;
            message_pos++;
        } else { // Full message received...
            //message[message_pos] = '\0'; // Add null character to string to end string
            int PS4input = atoi(message);
            if (PS4input == PSHOME){
                return 1;
            }
        }
    }
#if USE_IRREMOTE
    if (IrReceiver.decode()) {
                IRRawDataType ir_rec = IrReceiver.decodedIRData.decodedRawData;
                IrReceiver.resume();
                if (ir_rec == Rem_OK) {
                    flag = 1;
                }
            }
#endif
    return 0;
}

void dancing::dance() {
    while (flag == 0) {
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
        flag = exitLoop();
    }
}



