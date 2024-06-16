//
// Created by mr on 4/16/2024.
//

#ifndef ARDUINO_R4_UNO_WALL_Z_ANALOG_H
#define ARDUINO_R4_UNO_WALL_Z_ANALOG_H


#include <cstdint>

class analog {

public:
    static void analogSetup();

    static void analogLoop();

	static int16_t ext_analog_0;
	static int16_t ext_analog_1;
	static int16_t ext_analog_2;
	static int16_t ext_analog_3;
};


#endif //ARDUINO_R4_UNO_WALL_Z_ANALOG_H
