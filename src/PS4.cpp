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
unsigned int PS4::message_pos = 0;
int timers::timerButton;


int PS4::exitLoop() {
    if (Serial1.available()) {
        static char message[MAX_MESSAGE_LENGTH]; // Create char for serial1 message
        char inByte = Serial1.read();
        if (inByte != '\n' && (PS4::message_pos < MAX_MESSAGE_LENGTH - 1)) { // Add the incoming byte to our message
            message[PS4::message_pos] = inByte;
            PS4::message_pos++;
        } else { // Full message received...
            //message[PS4::message_pos] = '\0'; // Add null character to string to end string
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
    int LStickX, LStickY, RStickX, RStickY, L2_TRIG, R2_TRIG, R_velocity, L_velocity = 0;
    if (PS4input >= 4000 && PS4input <= 4255) { L2_TRIG = PS4input - 4000; }
    else if (PS4input >= 5000 && PS4input <= 5255) { R2_TRIG = PS4input - 5000; }
    else if (PS4input >= 6000 && PS4input <= 6255) { LStickX = PS4input - 6000; }
    else if (PS4input >= 7000 && PS4input <= 7255) { LStickY = PS4input - 7000; }
    else if (PS4input >= 8000 && PS4input <= 8255) { RStickX = PS4input - 8000; }
    else if (PS4input >= 9000 && PS4input <= 9255) { RStickY = PS4input - 9000; }

    if (L2_TRIG > 45) {
        digitalWrite(R_ROT, LOW);
        digitalWrite(L_ROT, LOW);
        analogWrite(R_PWM, L2_TRIG); // Send PWM signal to motor A
        analogWrite(L_PWM, L2_TRIG); // Send PWM signal to motor B
    }
    if (R2_TRIG > 45) {
        digitalWrite(R_ROT, HIGH);
        digitalWrite(L_ROT, HIGH);
        analogWrite(R_PWM, R2_TRIG); // Send PWM signal to motor A
        analogWrite(L_PWM, R2_TRIG); // Send PWM signal to motor B
    }
    //---------------------------------------------- RIGHT THUMBSTICK
    if (RStickY < 128) {
        int yMapped = map(RStickY, 128, 0, 45, 0);
        pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(yMapped));
    } else if (RStickY > 128) {
        int yMapped = map(RStickY, 128, 255, 45, 70);
        pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(yMapped));
    }
    if (RStickX < 128) {
        int xMapped = map(RStickX, 128, 0, 90, 160);
        pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(xMapped));
    } else if (RStickX > 128) {
        int xMapped = map(RStickX, 128, 255, 90, 20);
        pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(xMapped));
    }

    //----------------------------------------------- LEFT THUMBSTICK
    if (LStickY < 128) {
        digitalWrite(R_ROT, LOW);
        digitalWrite(L_ROT, LOW);
        int yMapped = map(LStickY, 128, 0, 0, 255);
        L_velocity = L_velocity + yMapped;
        R_velocity = R_velocity + yMapped;
    } else if (LStickY > 128) {
        digitalWrite(R_ROT, HIGH);
        digitalWrite(L_ROT, HIGH);
        int yMapped = map(LStickY, 128, 0, 0, 255);
        L_velocity = L_velocity - yMapped;
        R_velocity = R_velocity - yMapped;
    } else { // If joystick stays in middle the motors are not moving
        R_velocity = 0;
        L_velocity = 0;
    }

    if (LStickX < 128) {  // X-axis used for left and right control
        int xMapped = map(LStickX, 128, 0, 0, 255);
        L_velocity = L_velocity - xMapped;
        R_velocity = R_velocity + xMapped;
        if (L_velocity < 0) { L_velocity = 0; }
        if (R_velocity > 255) { R_velocity = 255; }
    }
    if (LStickX > 128) {
        int xMapped = map(LStickX, 128, 255, 0, 255);
        L_velocity = L_velocity + xMapped;
        R_velocity = R_velocity - xMapped;
        if (L_velocity > 255) { L_velocity = 255; }
        if (R_velocity < 0) { R_velocity = 0; }
    }
    if (R_velocity < 100) { R_velocity = 0; }
    if (L_velocity < 100) { L_velocity = 0; }
    analogWrite(R_PWM, R_velocity); // Send PWM signal to motor A
    analogWrite(L_PWM, L_velocity); // Send PWM signal to motor B

};

int PS4::getInput () {
    static char message[MAX_MESSAGE_LENGTH]; // Create char for serial1 message
    char inByte = Serial1.read();
    if (inByte != '\n' && (PS4::message_pos < MAX_MESSAGE_LENGTH - 1)) { // Add the incoming byte to our message
        message[PS4::message_pos] = inByte;
        PS4::message_pos++;
    } else { // Full message received...
        message[PS4::message_pos] = '\0'; // Add null character to string to end string
        // Use the message
        Serial.println(message);
        U8G2_SH1106_128X64_NONAME_1_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
        displayU8G2::display.println(message);

        //Or convert to integer and print
       return atoi(message);
    }
};

void PS4::controller() {
        int PS4input = PS4::getInput();
        if (PS4input > 4000) { PS4::joystick(PS4input); }
        else {
            switch (PS4input) {
                //**** Head movements    ****
                case DPAD_U:pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(posZ += 1));break;
                case DPAD_R:pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(posXY -= 1));break;
                case DPAD_D:pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(posZ -= 1));break;
                case DPAD_L:pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(posXY += 1));break;

                case SQUARE:Motor::Car_left();break;
                case TRIANG:Motor::Car_front();break;
                case xCROSS:Motor::Car_Back();break;
                case CIRCLE:Motor::Car_right();break;


                case 3101:
                case 3401:
                case 3201:
                case 3301:
                    Motor::Car_Stop();
                    break;


                case xSHARE:posXY = 90;posZ = 45;break;
                case OPTION:posXY = 90;posZ = 15;break;
                case L1:
                    #if USE_ADAFRUIT
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
                    #if USE_ADAFRUIT
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
                    /*
                    CHARGE  3500
                    XAUDIO  3600
                    MIC     3700
                    PS4_Battery        3900 + Battery

                    */
                default:
                    break;
            }
        }
        PS4::message_pos = 0; //Reset next message
 }


