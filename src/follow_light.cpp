//
// Created by mr on 10/27/2023.
//

#include <Arduino.h>
#include "follow_light.h"
#include "motor.h"


/*************************************************** Light Follow **********************************************/
// section Follow Light
/***************************************************************************************************************/

int Follow_light::lightSensorL = analogRead(light_R_Pin);
int Follow_light::lightSensorR = analogRead(light_L_Pin);
int Follow_light::flag;

double Follow_light::lightSensor(){
    long outputValueR = map(Follow_light::lightSensorL, 0, 1023, 0, 255);
    long outputValueL = map(Follow_light::lightSensorR, 0, 1023, 0, 255);
    double calcValue = 255 - (outputValueR + outputValueL)*.5;
    return (calcValue < 0) ? 0 : calcValue;
}

int Follow_light::exitLoop() {
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

void Follow_light::light_track() {
    while (flag == 0) {
        if (lightSensorR > 650 && lightSensorL > 650) {
            Motor::Car_front();
        }
        else if (lightSensorR > 650 && lightSensorL <= 650) {
            Motor::Car_left();
        }
        else if (lightSensorR <= 650 && lightSensorL > 650) {
            Motor::Car_right();
        }
        else {
            Motor::Car_Stop();
        }
        flag = exitLoop();
    }
}
