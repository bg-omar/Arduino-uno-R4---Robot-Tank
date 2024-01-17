//
// Created by mr on 11/17/2023.
//

#ifndef ARDUINO_R4_UNO_WALL_Z_IRRECEIVER_H
#define ARDUINO_R4_UNO_WALL_Z_IRRECEIVER_H

#define IR_SEND_PIN        9
#define RAW_BUFFER_LENGTH  750
#define DECODE_NEC
#include <cstdint>

class IRreceiver {
private:
    #define Rem_OK  0xBF407F00
    #define Rem_U   0xB946FF00
    #define Rem_D   0xEA15FF00
    #define Rem_L   0xBB44FF00
    #define Rem_R   0xBC43FF00

    #define Rem_1   0xE916FF00
    #define Rem_2   0xE619FE00
    #define Rem_3   0xF20DFE00
    #define Rem_4   0xF30CFF00
    #define Rem_5   0xE718FF00
    #define Rem_6   0xA15EFD00
    #define Rem_7   0xF708FF00
    #define Rem_8   0xE31CFF00
    #define Rem_9   0xA55AFF00
    #define Rem_0   0xAD52FF00
    #define Rem_x   0xBD42FF00
    #define Rem_y   0xB54ADF00
    #define IRepeat 0xFFFFFF00
    static const unsigned int MAX_MESSAGE_LENGTH = 30;
    static unsigned int message_pos;
    static int flag;

    static int posXY;  // set horizontal servo position
    static int posZ;   // set vertical servo position

public:
    static int previousXY, previousZ;
    static uint64_t ir_rec, previousIR; // set remote vars

    static void irRemote();
    static void setupIrRemote();
};


#endif //ARDUINO_R4_UNO_WALL_Z_IRRECEIVER_H



