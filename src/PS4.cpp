//
// Created by mr on 11/13/2023.
//

#include "PS4.h"

#include "Arduino.h"
#include <U8g2lib.h>
#include "motor.h"
#include "pwm_board.h"
#include "timers.h"
#include "displayU8G2.h"
#include "IRreceiver.h"
#include "dancing.h"
#include "avoid_objects.h"
#include "follow_light.h"


int PS4::flag = 0;
int PS4::posXY = 90;  // set horizontal servo position
int PS4::posZ = 45;   // set vertical servo position
int PS4::R_velocity, PS4::L_velocity, PS4::R_velocityR, PS4::L_velocityL  = 0;

int PS4::exitLoop() {
    if (Serial1.available()) {
        static char message[MAX_MESSAGE_LENGTH]; // Create char for serial1 message
        static unsigned int message_pos = 0;
        char inByte = Serial1.read();
        if (inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1)) { // Add the incoming byte to our message
            message[message_pos] = inByte;
            message_pos++;
        } else { // Full message received...
            message[message_pos] = '\0'; // Add null character to string to end string
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

void PS4::joystick(int PS4input) {
/*

    //---------------------------------------------- RIGHT THUMBSTICK
    if (PS4input >= 9000 && PS4input <= 9118) {
        int yMapped = map(PS4input - 9000, 128, 0, 45, 0);
        Serial.print(yMapped);
        pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(yMapped));
    } else if (PS4input >= 9138 && PS4input <= 9255) {
        int yMapped = map(PS4input - 9000, 128, 255, 45, 70);
        Serial.print(yMapped);
        pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(yMapped));
    }


    if (PS4input >= 8000 && PS4input <= 8118) {
        int xMapped = map(PS4input - 8000, 128, 0, 90, 160);
        Serial.print(xMapped);
        pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(xMapped));
    } else if (PS4input >= 8138 && PS4input <= 8255) {
        int xMapped = map(PS4input - 8000, 128, 255, 90, 20);
        Serial.print(xMapped);
        pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(xMapped));
    }
*/

    //----------------------------------------------- R2 L2
    if (PS4input >= 4045 && PS4input <= 4255) {
        digitalWrite(L_ROT, HIGH);
        digitalWrite(R_ROT, LOW);
        L_velocity = PS4input - 4000;
        R_velocity = PS4input - 4000;
    }
    if (PS4input >= 5045 && PS4input <= 5255) {
        digitalWrite(L_ROT, LOW);
        digitalWrite(R_ROT, HIGH);
        L_velocity = PS4input - 5000;
        R_velocity = PS4input - 5000;
    }

    //----------------------------------------------- LEFT THUMBSTICK
    if (PS4input >= 7000 && PS4input <= 7118) {
        digitalWrite(L_ROT, HIGH);
        digitalWrite(R_ROT, LOW);
        int yMapped = map(PS4input - 7000, 128, 0, 0, 255);
        L_velocity =  yMapped;
        R_velocity =  yMapped;
    } else if (PS4input >= 7138 && PS4input <= 7255) {
        digitalWrite(L_ROT, LOW);
        digitalWrite(R_ROT, HIGH);
        int yMapped = map(PS4input - 7000, 128, 255, 0, 255);
        L_velocity =  yMapped;
        R_velocity =  yMapped;
    }

    L_velocityL = L_velocity;
    R_velocityR = R_velocity;

    if (PS4input >= 6000 && PS4input <= 6118) {  // X-axis used for left and right control
        int xMapped = map(PS4input - 6000, 128, 0, 0, 255);
        Serial.println(xMapped);
        digitalWrite(L_ROT, LOW);
        L_velocityL = L_velocity - xMapped;
        R_velocityR = R_velocity + xMapped;
    } else if (PS4input >= 6138 && PS4input <= 6255) {
        int xMapped = map(PS4input - 6000, 128, 255, 0, 255);
        Serial.println(xMapped);
        digitalWrite(R_ROT, HIGH);
        L_velocityL = L_velocity + xMapped;
        R_velocityR = R_velocity - xMapped;
    }
    if (R_velocityR < 50) { R_velocityR = 0; }
    if (L_velocityL < 50) { L_velocityL = 0; }
    if (R_velocityR > 255) { R_velocityR = 255; }
    if (L_velocityL > 255) { L_velocityL = 255; }

    analogWrite(R_PWM, R_velocityR);
    analogWrite(L_PWM, L_velocityL);

    Serial.print("PS4input: ");
    Serial.println(PS4input);
    delay(1);

    Serial.print("---------------------------- R_velocity: ");
    Serial.println(R_velocityR);
    delay(1);

    Serial.print("----------------L_velocity: ");
    Serial.println(L_velocityL);
    delay(1);

};

void PS4::controller() {
    while (Serial1.available() > 0) {
        static char message[MAX_MESSAGE_LENGTH]; // Create char for serial1 message
        static unsigned int message_pos = 0;

        char inByte = Serial1.read();
        if (inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1)) { // Add the incoming byte to our message
            message[message_pos] = inByte;
            message_pos++;
        } else { // Full message received...
            message[message_pos] = '\0'; // Add null character to string to end string
            // Use the message
            //Serial.println(message);
            String s = message;

            #if USE_U8G2
                        U8G2_SH1106_128X64_NONAME_1_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
                    displayU8G2::display.println(message);
            #endif
            //Or convert to integer and print
            int PS4input = atoi(message);
            delay(1);
            //Serial.println(PS4input);
            delay(1);

            if (PS4input > 4000) {
                PS4::joystick(PS4input);
            } else {
                switch (PS4input) {
                    case SQUARE:
                        Motor::Car_left();
                        break;
                    case TRIANG:
                        Motor::Car_front();
                        break;
                    case xCROSS:
                        Motor::Car_Back();
                        break;
                    case CIRCLE:
                        Motor::Car_right();
                        break;

                        //**** Head movements    ****
                    case DPAD_U:
                        pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(posZ += 1));
                        break;
                    case DPAD_R:
                        pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(posXY -= 1));
                        break;
                    case DPAD_D:
                        pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(posZ -= 1));
                        break;
                    case DPAD_L:
                        pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(posXY += 1));
                        break;


                    case 3101:
                    case 3401:
                    case 3201:
                    case 3301:
                        Motor::Car_Stop();
                        break;


                    case xSHARE:posXY = 90;posZ = 45;break;
                    case OPTION:posXY = 90;posZ = 15;break;
                    case L1:
                      #if USE_U8G2
                          displayU8G2::display.clearDisplay();
                      #endif
                      timers::timerTwoActive = !timers::timerTwoActive;
                      timers::timerTreeActive = false;
                      timers::timerButton = L1;
                      delay(100);
                      break;
                    case TOUCHPD:
                      #if USE_ROBOT
                          dancing::dance(); break;
                      #endif
                    case R1:
                      #if USE_U8G2
                          displayU8G2::display.clearDisplay();
                      #endif
                      timers::timerTwoActive = !timers::timerTwoActive;
                      timers::timerTreeActive = false;
                      timers::timerButton = R1;
                      delay(100);
                      break;
                    case L3:
                      #if USE_ROBOT
                          avoid_objects::avoid(); break;
                      #endif

                    case R3:
                      #if USE_ROBOT
                          Follow_light::light_track(); break;
                      #endif
                        //                    CHARGE  3500
                        //                    XAUDIO  3600
                        //                    MIC     3700
                        //                    PS4_Battery        3900 + Battery

                    default:
                        break;
                }
            }
            message_pos = 0; //Reset next message
            delay(5);
        }

    }
}
