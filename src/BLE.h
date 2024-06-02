//
// Created by mr on 3/24/2024.
//

#ifndef ARDUINO_R4_UNO_WALL_Z_BLE_H
#define ARDUINO_R4_UNO_WALL_Z_BLE_H


class BLE {

	static void readBLE();

public:
	static void BLEsetup();

	static void BLEloop();
};


#endif //ARDUINO_R4_UNO_WALL_Z_BLE_H
