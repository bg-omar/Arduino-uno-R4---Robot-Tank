//
// Created by mr on 11/17/2023.
//

#ifndef ARDUINO_R4_UNO_WALL_Z_IRREMOTE_H
#define ARDUINO_R4_UNO_WALL_Z_IRREMOTE_H

#define IR_SEND_PIN        9
#define RAW_BUFFER_LENGTH  750
#define DECODE_NEC
#include <IRremote.hpp>
#include <cstdint>

#include "follow_light.h"
#include "motor.h"
#include "PS4.h"
#include "pwm_board.h"
#include "timers.h"
#include "compass.h"


class IRremote {
private:
    #define Rem_OK  0xBF407F
    #define Rem_U   0xB946FF
    #define Rem_D   0xEA15FF
    #define Rem_L   0xBB44FF
    #define Rem_R   0xBC43FF

    #define Rem_1   0xE916FF
    #define Rem_2   0xE619FE
    #define Rem_3   0xF20DFE
    #define Rem_4   0xF30CFF
    #define Rem_5   0xE718FF
    #define Rem_6   0xA15EFD
    #define Rem_7   0xF708FF
    #define Rem_8   0xE31CFF
    #define Rem_9   0xA55AFF
    #define Rem_0   0xAD52FF
    #define Rem_x   0xBD42FF
    #define Rem_y   0xB54ADF
    #define IRepeat 0xFFFFFF
    static const unsigned int MAX_MESSAGE_LENGTH = 30;
    static unsigned int message_pos;
    static int flag;

    static int posXY;  // set horizontal servo position
    static int posZ;   // set vertical servo position
    static int16_t lastY;
public:
    static uint64_t ir_rec, previousIR; // set remote vars

    static void irRemote();
};


#endif //ARDUINO_R4_UNO_WALL_Z_IRREMOTE_H
