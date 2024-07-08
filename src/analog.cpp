//
// Created by mr on 4/16/2024.
//

#include "analog.h"
#include "Wire.h"
#include "config.h"
#include "ADS1X15.h"
#include "main_ra.h"
#include "logger.h"

ADS1015 ADS(0x48);

void analog::analogSetup()
{
    if(!ADS.begin()){
		logger::log("ADS1015 LIB failed");
		main::use_analog = false;
	} else {
		logger::log("ADS1015 LIB loaded");
	};
}


void analog::analogLoop()
{
    ADS.setGain(0);

    analog::ext_analog_0 = ADS.readADC( light_L_PIN );
    analog::ext_analog_1 = ADS.readADC( MIC_L_PIN   );
    analog::ext_analog_2 = ADS.readADC( light_R_PIN );
    analog::ext_analog_3 = ADS.readADC( MIC_R_PIN   );

	#if LOG_VERBOSE
		float f = ADS.toVoltage(1);  // voltage factor
		Serial.print("light_L_PIN: \t"); Serial.print(ext_analog_0); Serial.print('\t'); Serial.println(ext_analog_0 * f, 3);
		Serial.print("light_R_PIN: \t"); Serial.print(ext_analog_2); Serial.print('\t'); Serial.println(ext_analog_2 * f, 3);

		Serial.print("MIC_L_PIN: \t"); Serial.print(ext_analog_1); Serial.print('\t'); Serial.println(ext_analog_1 * f, 3);
		Serial.print("MIC_R_PIN: \t"); Serial.print(ext_analog_3); Serial.print('\t'); Serial.println(ext_analog_3 * f, 3);
		Serial.println();
		delay(100);
	#endif

}