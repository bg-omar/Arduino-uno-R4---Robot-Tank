/*
UNO BLE -->     DC:54:75:C3:D9:ED   -
PC USB Dongel   00:1F:E2:C8:82:BA
ESP 1           66:CB:3E:E9:02:8A
ESP small cam   3C:E9:0E:88:65:16
PS4 Controller: A4:AE:11:E1:8B:B3 (SONYWA) GooglyEyes
PS5 Controller: 88:03:4C:B5:00:66
USB-Shield BT   00:1a:7d:da:71:13
				00:1a:7d:da:71:13
*/


/***************************************************************************************************************/
// section include
/***************************************************************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>



#include <wiring_private.h>
#include <cstring>
#include <cstdint>
#include <cmath>

#include <Adafruit_Sensor.h>

#include "PS4.h"
#include "secrets.h"
#include "main_ra.h"
#include "menu.h"

#include "logger.h"
#include "barometer.h"
#include "animation.h"
#include "pwm_board.h"
#include "timers.h"
#include "compass.h"
#include "gyroscope.h"

#include "dancing.h"
#include "displayAdafruit.h"
#include "pesto_matrix.h"
#include "follow_light.h"
#include "motor.h"
#include "MicStereo.h"
#include "avoid_objects.h"
#include "I2Cscanner.h"

#include "analog.h"
#include "SD_card.h"


bool main::Found_Display;
bool main::Found_Gyro = false;
bool main::Found_Compass = false;
bool main::Found_Mics = false;
bool main::Found_PwmBoard = false;
bool main::Found_Switch = false;
bool main::Found_Sonar = false;

bool main::use_adafruit = false;
bool main::use_u8g2 = false;
bool main::small = false;
bool main::display_demo = false;
bool main::use_round = false;
bool main::use_menu = false;
bool main::log_debug = false;
bool main::use_ps4 = false;
bool main::use_sd_card = false;
bool main::use_gyro = false;
bool main::use_compass = false;
bool main::use_barometer = false;
bool main::use_distance = false;
bool main::use_irremote = false;
bool main::use_i2c_scanner = false;
bool main::use_pwm_board = false;
bool main::use_dot = false;
bool main::use_mic = false;
bool main::use_switch = false;
bool main::use_analog = false;
bool main::use_robot = false;
bool main::use_timers = false;
bool main::use_matrix = false;
bool main::use_matrix_preview = false;
bool main::read_esp32 = false;
bool main::use_lcd = false;
bool main::use_hm_10_ble = false;

//int timers::timerButton;

int pwm_board::posXY = 90;  // set horizontal servo position
int pwm_board::posZ = 135;   // set vertical servo position

int Switch_8_State, Switch_9_State;

// Define an array to hold pixel data for a single frame (4 pixels)
uint32_t frame[] = {0, 0, 0, 0xFFFF};
//ArduinoLEDMatrix matrix;


/********************************************** Setup booting the arduino **************************************/
// section Setup
/***************************************************************************************************************/
void setup(){
    Wire.begin();
	Serial.begin(9600);// Initialize the hardware serial port for debugging
	displayAdafruit::setupAdafruit();

	logger::logln("Wall-Z Arduino Robot booting\n");

	delay(1);
	logger::log("Serial");
	delay(1);
	Serial1.begin(115200);
	logger::log("Serial1 & ");
	delay(1);
	SERIAL_AT.begin(115200);
	logger::logln("Serial_AT 115200");
	delay(1);

	Motor::motor_setup();

	SD_card::initSD();

	if(main::use_sd_card) {
		if(main::use_i2c_scanner)	I2Cscanner::scan();
		delay(500);
		if(main::use_analog) {
			analog::analogSetup();

			if(main::use_mic) {
				MicStereo::MicSetup();
				delay(500);
			}
		}

		if(main::use_switch) {
			pinMode(8, INPUT_PULLUP);     /***** 8 ******/
			pinMode(9, INPUT_PULLUP);     /***** 9 ******/

			delay(50);
			Switch_8_State = digitalRead(8);
			Switch_9_State = digitalRead(9);

			if (Switch_8_State == LOW) { logger::log(" SWITCH_8 is on"); } else { logger::log(" SWITCH_8 is off"); }
			if (Switch_9_State == LOW) { logger::logln(" SWITCH_9 is on"); } else { logger::logln(" SWITCH_9 is off"); }

			delay(500);
		}

		if(main::use_pwm_board) { pwm_board::setupPWM();   logger::logln(" PWM board enabled");}
		delay(500);
//
//		if(main::use_matrix_preview) {matrix.begin();}
//		else if(main::use_matrix) {
//			matrix.loadSequence(animation);
//			matrix.begin();
//			matrix.autoscroll(300);
//			matrix.play(true);
//			delay(500);
//			logger::logln(" R4 matrix ");
//		}


		if(main::use_distance) {
			pinMode(Trig_PIN, OUTPUT);    /***** 6 ******/
			pinMode(Echo_PIN, INPUT);     /***** 7 ******/
			logger::logln(" Sonar --> use_distance ");
			delay(500);
		}

		if(main::use_dot) {
			Pesto::setup_pestoMatrix();
			delay(500);
		}

		if(main::use_gyro) {
			gyroscope::gyroSetup();
			delay(500);
		}

		if(main::use_compass) {
			compass::compassSetup();
			delay(500);
		}

		if(main::use_barometer) {
			barometer::baroSetup();
			delay(500);
		}

//		if(main::use_timers) {
//			timers::initTimers();
//			delay(500);
//
//        }
		logger::logln("Setup from SD Complete");
	} else {
		logger::logln("Setup from config.h");
		#if USE_ROUND
			displayMenu::menuSetup();
		#endif

		#if USE_I2C_SCANNER
			I2Cscanner::scan();
			delay(500);
		#endif

		#if USE_ANALOG
			analog::analogSetup();
		#endif

		#if USE_SWITCH
			pinMode(SWITCH_8, INPUT_PULLUP);     /***** 8 ******/
			pinMode(SWITCH_9, INPUT_PULLUP);     /***** 9 ******/

			delay(50);
			Switch_8_State = digitalRead(SWITCH_8);
			Switch_9_State = digitalRead(SWITCH_9);

			if (Switch_8_State == LOW) {  logger::log(" SWITCH_8 is on");} else {logger::log(" SWITCH_8 is off");}
			if (Switch_9_State == LOW) {  logger::logln(" SWITCH_9 is on");} else {logger::logln(" SWITCH_9 is off");}

			delay(500);
		#endif

		#if USE_PWM_BOARD
			pwm_board::setupPWM();
			delay(500);
		#endif


		#if USE_MATRIX_PREVIEW
			matrix.begin();
		#elif USE_MATRIX
			matrix.loadSequence(animation);
			matrix.begin();
			matrix.autoscroll(300);
			matrix.play(true);
			delay(500);
			logger::log(" R4 matrix ");
		#endif

		#if USE_DISTANCE
			pinMode(Trig_PIN, OUTPUT);    /***** 6 ******/
			pinMode(Echo_PIN, INPUT);     /***** 7 ******/
			logger::log(" Sonar ");
			delay(500);
		#endif

		#if USE_DOT
			Pesto::setup_pestoMatrix();
			delay(500);
		#endif

		#if USE_MIC
			MicStereo::MicSetup();
			delay(500);
		#endif

		#if USE_COMPASS
			compass::compassSetup();
			delay(500);
		#endif

		#if USE_BAROMETER
			barometer::baroSetup();
			delay(500);
		#endif

		#if USE_GYRO
			gyroscope::gyroSetup();
			delay(500);
		#endif

		#if USE_TIMERS
			timers::initTimers();
			delay(500);

		//        general_timer::setup_General_Timer();
		//        delay(500);
		#endif

		#if USE_HM_10_BLE
			BLE::BLEsetup();
		#endif
		logger::logln("Setup from config.h Complete");
	}
	logger::logln("Starting loop");
}

/*********************************** Loop **********************************/
// section Loop
/***************************************************************************/

void loop(){
    if ((USE_PS4 && !main::use_sd_card) || main::use_ps4) {
		PS4::controller();
	}

	if ((USE_ADAFRUIT && !main::use_adafruit) || main::use_adafruit) {
		displayAdafruit::displayLoop();
	}

	if (main::use_menu) {
		menu::loopMenu();
	}

	if ((USE_SWITCH && !main::use_switch) || main::use_switch) {
        //Switch_8_State = digitalRead(SWITCH_8);
        //Switch_9_State = digitalRead(SWITCH_9);
        delay(5);
        //if (Switch_8_State == HIGH) { displayAdafruit::petStatus = 0;} else { displayAdafruit::petStatus = 1;}
        //if (Switch_9_State == HIGH) { compass::displayCompass();} else { displayAdafruit::animateScreen(); }
        delay(50);
    }

	if ((USE_ANALOG && !main::use_sd_card) || main::use_analog) {
		analog::analogLoop();
	}

    if((USE_GYRO && !main::use_sd_card) || main::use_gyro || main::Found_Gyro){
		gyroscope::gyroDetectMovement();
	}

    if ((USE_DISTANCE && !main::use_sd_card) || main::use_distance) {
		avoid_objects::distanceF = avoid_objects::checkDistance();  /// assign the front distance detected by ultrasonic sensor to variable a

		int lazer_brightness = map(avoid_objects::distanceF, 0, 1000, 0, 4000);
		pwm_board::pwm.setPWM(LAZER_PIN, 0, lazer_brightness);
		if (avoid_objects::distanceF < 25) {
			if ((USE_PWM_BOARD && !main::use_pwm_board) || main::use_pwm_board) {
				pwm_board::RGBled(230, 0, 0);
					pwm_board::leftLedStrip(255, 0, 0);
					pwm_board::rightLedStrip(255, 0, 0);
			}
		}

		#if LOG_DEBUG
			if (main::Found_Display) logger::logFloatln(avoid_objects::distanceF);
		#endif
		#if USE_DOT
			Pesto::pestoMatrix();
		#endif
		double deltime = avoid_objects::distanceF * 3;
		delay(deltime);
	} else {
		pwm_board::leftLedStrip(70, 0, 70);
		pwm_board::rightLedStrip(70, 0, 70);
	}

	#if USE_MIC
		MicStereo::MicLoop();
	#endif

    #if USE_TIMERS
        timers::update();
    #endif

    #if READ_ESP32
        // Read messages from Arduino R4 ESP32
            if (SERIAL_AT.available()) {
                main::log("ESP32 says: ");
                while (SERIAL_AT.available()) {
                    Serial.write(SERIAL_AT.read());
                    main::logln(SERIAL_AT.read());
                }
            }
    #endif

    #if USE_MATRIX_PREVIEW
        // Check if there are at least 12 bytes available in the serial buffer
        if(Serial.available() >= 12){
            // Read 4 bytes from the serial buffer and compose them into a 32-bit value for each element in the frame
            frame[0] = Serial.read() | Serial.read() << 8 | Serial.read() << 16 | Serial.read() << 24;
            frame[1] = Serial.read() | Serial.read() << 8 | Serial.read() << 16 | Serial.read() << 24;
            frame[2] = Serial.read() | Serial.read() << 8 | Serial.read() << 16 | Serial.read() << 24;

            // Load and display the received frame data on the LED matrix
            matrix.loadFrame(frame);
        }
    #endif

}



