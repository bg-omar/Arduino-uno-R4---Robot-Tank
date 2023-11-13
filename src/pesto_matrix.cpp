//
// Created by mr on 10/27/2023.
//

#include <Arduino.h>
#include "pesto_matrix.h"


/********************************************** the function for dot matrix display ****************************/
// section Pesto Matrix
/***************************************************************************************************************/
//the condition to start conveying data
void Pesto::IIC_start() {
    digitalWrite(DotClockPIN,HIGH);
    delayMicroseconds(3);
    digitalWrite(DotDataPIN,HIGH);
    delayMicroseconds(3);
    digitalWrite(DotDataPIN,LOW);
    delayMicroseconds(3);
}
//Convey data
void Pesto::IIC_send(unsigned char send_data) {
    for(char i = 0;i < 8;i++){  //Each byte has 8 bits 8bit for every character
        digitalWrite(DotClockPIN,LOW);  // pull down clock pin DotClockPIN to change the signal of SDA
        delayMicroseconds(3);
        if(send_data & 0x01){  //set high and low level of DotDataPIN according to 1 or 0 of every bit
            digitalWrite(DotDataPIN,HIGH);
        } else {
            digitalWrite(DotDataPIN,LOW);
        }
        delayMicroseconds(3);
        digitalWrite(DotClockPIN,HIGH); //pull up the clock pin DotClockPIN to stop transmission
        delayMicroseconds(3);
        send_data = send_data >> 1;  // detect bit by bit, shift the data to the right by one
    }
}

//The sign of ending data transmission
void Pesto::IIC_end() {
    digitalWrite(DotClockPIN,LOW);
    delayMicroseconds(3);
    digitalWrite(DotDataPIN,LOW);
    delayMicroseconds(3);
    digitalWrite(DotClockPIN,HIGH);
    delayMicroseconds(3);
    digitalWrite(DotDataPIN,HIGH);
    delayMicroseconds(3);
}

void Pesto::matrix_display(unsigned char matrix_value[]) {
    IIC_start();  // use the function of the data transmission start condition
    IIC_send(0xc0);  //select address

    for(int i = 0;i < 16;i++) { //pattern data has 16 bits
        IIC_send(matrix_value[i]); //convey the pattern data
    }

    IIC_end();   //end the transmission of pattern data
    IIC_start();
    IIC_send(0x8A);  //display control, set pulse width to 4/16 s
    IIC_end();
}


void Pesto::pestoMatrix() {
    switch (screen) {
        case 1: matrix_display(STOP01); break;
        case 2: matrix_display(hou);    break;
        case 3: matrix_display(op);     break;
        case 4: matrix_display(met);    break;
        case 5: matrix_display(pesto);  break;
        case 6: matrix_display(bleh);   break;
        default:matrix_display(bleh);
    }
    screen == 6 ? screen = 0 : screen += 1;
}
