//
// Created by mr on 11/13/2023.
//

#ifndef ARDUINO_R4_UNO_WALL_Z_PS4_H
#define ARDUINO_R4_UNO_WALL_Z_PS4_H


#define DPAD_U  1100
#define DPAD_R  1200
#define DPAD_D  1300
#define DPAD_L  1400
#define SQUARE  3100
#define xCROSS  3200
#define CIRCLE  3300
#define TRIANG  3400
#define OPTION  2900
#define xSHARE  2800
#define PSHOME  2500
#define CHARGE  3500
#define XAUDIO  3600
#define URIGHT  1500
#define DRIGHT  1600
#define D_LEFT  1700
#define UPLEFT  1800
#define L1      2100
#define L3      2300
#define R1      2200
#define R3      2400
#define TOUCHPD 2700
#define MIC     3700

const unsigned int MAX_MESSAGE_LENGTH = 30;

int LStickX, LStickY, RStickX, RStickY, L2_TRIG, R2_TRIG;
int flag = 0;

int R_velocity = 0;
int L_velocity = 0;

class PS4 {
    static void exitLoop();
    static void joystick(int PS4input);

};


#endif //ARDUINO_R4_UNO_WALL_Z_PS4_H
