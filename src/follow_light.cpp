//
// Created by mr on 10/27/2023.
//

#include "follow_light.h"

/*************************************************** Light Follow **********************************************/
// section Follow Light
/***************************************************************************************************************/

void light_track() {
		flag = 0;
		while (flag == 0) {
			lightSensorR = analogRead(light_R_Pin);
			lightSensorL = analogRead(light_L_Pin);
			if (lightSensorR > 650 && lightSensorL > 650) {
				Car_front();
			}
			else if (lightSensorR > 650 && lightSensorL <= 650) {
				Car_left();
			}
			else if (lightSensorR <= 650 && lightSensorL > 650) {
				Car_right();
			}
			else {
				Car_Stop();
			}
            exitLoop();
		}
	}
