//
// Created by mr on 11/13/2023.
//

#include "PS4.h"
#include "Arduino.h"

void PS4::exitLoop() {
    if (Serial1.available()) {
        static char message[MAX_MESSAGE_LENGTH]; // Create char for serial1 message
        static unsigned int message_pos = 0;
        char inByte = Serial1.read();
        if (inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1)) { // Add the incoming byte to our message
            message[message_pos] = inByte;
            message_pos++;
        } else { // Full message received...
            //message[message_pos] = '\0'; // Add null character to string to end string
            int PS4input = atoi(message);
            if (PS4input == PSHOME)flag = 1;
        }
    }
}


void PS4::joystick(int PS4input) {
    if      (PS4input >= 4000 && PS4input <= 4255){ L2_TRIG = PS4input - 4000; }
    else if (PS4input >= 5000 && PS4input <= 5255){ R2_TRIG = PS4input - 5000; }
    else if (PS4input >= 6000 && PS4input <= 6255){ LStickX = PS4input - 6000; }
    else if (PS4input >= 7000 && PS4input <= 7255){ LStickY = PS4input - 7000; }
    else if (PS4input >= 8000 && PS4input <= 8255){ RStickX = PS4input - 8000; }
    else if (PS4input >= 9000 && PS4input <= 9255){ RStickY = PS4input - 9000; }

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
        pwm.setPWM(PWM_1, 0, pulseWidth(yMapped));
    }
    else if (RStickY > 128) {
        int yMapped = map(RStickY, 128, 255, 45, 70);
        pwm.setPWM(PWM_1, 0, pulseWidth(yMapped));
    }
    if (RStickX < 128) {
        int xMapped = map(RStickX, 128, 0, 90, 160);
        pwm.setPWM(PWM_0, 0, pulseWidth(xMapped));
    } else  if (RStickX > 128) {
        int xMapped = map(RStickX, 128, 255, 90, 20);
        pwm.setPWM(PWM_0, 0, pulseWidth(xMapped));
    }

    //----------------------------------------------- LEFT THUMBSTICK
    if (LStickY < 128) {
        digitalWrite(R_ROT, LOW);
        digitalWrite(L_ROT, LOW);
        int yMapped = map(LStickY, 128, 0, 0, 255);
        L_velocity = L_velocity + yMapped;
        R_velocity = R_velocity + yMapped;
    }
    else if (LStickY > 128) {
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
        if (L_velocity < 0) {  L_velocity = 0;  }
        if (R_velocity > 255) { R_velocity = 255; }
    }
    if (LStickX > 128) {
        int xMapped = map(LStickX, 128, 255, 0, 255);
        L_velocity = L_velocity + xMapped;
        R_velocity = R_velocity - xMapped;
        if (L_velocity > 255) { L_velocity = 255; }
        if (R_velocity < 0) { R_velocity = 0;  }
    }
    if (R_velocity < 100) { R_velocity = 0; }
    if (L_velocity < 100) { L_velocity = 0; }
    analogWrite(R_PWM, R_velocity); // Send PWM signal to motor A
    analogWrite(L_PWM, L_velocity); // Send PWM signal to motor B

}



#if USE_IRREMOTE
    if (IrReceiver.decode()) {
            ir_rec = IrReceiver.decodedIRData.decodedRawData;
            IrReceiver.resume();
            if (ir_rec == Rem_OK) {
                flag = 1;
            }
        }
#endif
}