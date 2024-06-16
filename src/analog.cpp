//
// Created by mr on 4/16/2024.
//

#include "analog.h"

#include "config.h"
#include "ADS1X15.h"


ADS1115 ADS(0x48);


void analog::analogSetup()
{
    Serial.println(__FILE__);
    Serial.print("ADS1X15_LIB_VERSION: ");
    Serial.println(ADS1X15_LIB_VERSION);
    ADS.begin();
}


void analog::analogLoop()
{
    ADS.setGain(0);

    analog::ext_analog_0 = ADS.readADC( light_L_PIN );
    analog::ext_analog_1 = ADS.readADC( MIC_L_PIN   );
    analog::ext_analog_2 = ADS.readADC( light_R_PIN );
    analog::ext_analog_3 = ADS.readADC( MIC_R_PIN   );

	#if LOG_VERBOSE
		float f = ADS.toVoltage(2);  // voltage factor
		Serial.print("\tlight_L_PIN: "); Serial.print(ext_analog_0); Serial.print('\t'); Serial.println(ext_analog_0 * f, 3);
		Serial.print("\tlight_R_PIN: "); Serial.print(ext_analog_2); Serial.print('\t'); Serial.println(ext_analog_2 * f, 3);

		Serial.print("\tMIC_L_PIN: "); Serial.print(ext_analog_1); Serial.print('\t'); Serial.println(ext_analog_1 * f, 3);
		Serial.print("\tMIC_R_PIN: "); Serial.print(ext_analog_3); Serial.print('\t'); Serial.println(ext_analog_3 * f, 3);
		Serial.println();
		delay(100);
	#endif

}