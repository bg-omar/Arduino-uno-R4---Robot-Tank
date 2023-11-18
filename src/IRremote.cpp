//
// Created by mr on 11/17/2023.
//

#include "IRremote.h"

uint64_t ir_rec, previousIR = 0;
int16_t lastY = 0;
void IRremote::irRemote() {
    if (IrReceiver.decode()) {  // Grab an IR code   At 115200 baud, printing takes 200 ms for NEC protocol and 70 ms for NEC repeat
        if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_WAS_OVERFLOW) {         // Check if the buffer overflowed
            displayU8G2::display.clearDisplay();

            displayU8G2::display.print(F("Try to increase the \"RAW_BUFFER_LENGTH\" value of " STR(RAW_BUFFER_LENGTH) " in " __FILE__));
            delay(100);
        } else {
            displayU8G2::display.clearDisplay();
            if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
                ir_rec = previousIR;

                displayU8G2::display.print(F("?"));
                delay(100);
            } else {
                ir_rec = IrReceiver.decodedIRData.decodedRawData;
            }

            displayU8G2::display.print(ir_rec, HEX);
        }
        IrReceiver.resume();                            // Prepare for the next value

        switch (ir_rec) {
            /****** Head movements    ******/
            case Rem_1:  pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(posZ+1));  break;
            case Rem_2:  pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(posXY-1));  break;
            case Rem_3:  pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(posZ-1));    break;
            case Rem_4:  posXY = 90; posZ = 45;  break;

            case Rem_5:  pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(posXY+1)); break;
            case Rem_6:                     posXY = 90; posZ = 15; break;

                /****** Options & Sensors ******/
            case Rem_7:
                displayU8G2::display.clearDisplay();
                timers::timerTwoActive = !timers::timerTwoActive;
                timers::timerTreeActive = false;
                timers::timerButton = Rem_7;
                delay(100);
                break;
            case Rem_8: dance(); break;
            case Rem_9:
                displayU8G2::display.clearDisplay();
                timers::timerTwoActive = !timers::timerTwoActive;
                timers::timerTreeActive = false;
                timers::timerButton = Rem_9;
                delay(100);
                break;
            case Rem_x: avoid(); break;
            case Rem_y: Follow_light::light_track(); break;

                /****** Engine - driving  ******/
            case Rem_OK:
                Motor::Car_Stop(); break;
            case Rem_U:
                Motor::Car_front(); break;
            case Rem_D:
                Motor::Car_Back(); break;
            case Rem_L:
                Motor::Car_left(); break;
            case Rem_R:
                Motor::Car_right(); break;
            default:
                break;
        }
        if (posXY != previousXY) {
            pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(posXY));
        }
        if (posZ != previousZ) {
            pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(posZ));
        }
        previousIR = ir_rec;
        previousXY = posXY;
        previousZ = posZ;
        Serial1.write(" sending PESTO!!!");
    }
}