//
// Created by mr on 6/8/2024.
//

#include "SD_card.h"
#include "main_ra.h"
#include "config.h"

#include <SPI.h>
#include "SdFat.h"
#include "logger.h"
#include <iostream>
#include <vector>
#include <cstring>


SdFat SD;
File file;
char line[40];
char str[] = "";
const char* delim = ",";

std::vector<std::string> configName;
std::vector<int> configValue = {};

void openFile(int rule);

//------------------------------------------------------------------------------
#define errorHalt(msg) {Serial.println(F(msg)); while (true) {}}
//------------------------------------------------------------------------------
void SD_card::initSD() {
	// Wait for USB Serial
	while (!Serial) {
		yield();
	}

	// Initialize the SD.
	if (!SD.begin(CS_PIN, SD_SCK_MHZ(16))) {
		logger::logln("SD card init failed");
		main::use_sd_card = false;
	} else {
		logger::logln("SD card initialized");
		main::use_sd_card = true;
		configLoadSD();
	}
}

//------------------------------------------------------------------------------
// Store error strings in flash to save RAM.
#define error(s) sd.errorHalt(&Serial, F(s))
//------------------------------------------------------------------------------
// Check for extra characters in field or find minus sign.
char* skipSpace(char* str) {
	while (isspace(*str)) str++;
	return str;
}
//------------------------------------------------------------------------------

bool parseLine(char* str) {
	char* ptr;

	// Set strtok start of line.
	str = strtok(str, delim);
	if (!str) return false;

	configName.emplace_back(str); // Append each token to the vector

	// Subsequent calls to strtok expects a null pointer.
	str = strtok(nullptr, delim);
	if (!str) return false;

	// Convert string to long integer.
	int32_t i32 = strtol(str, &ptr, 0);
	if (str == ptr || *skipSpace(ptr)) return false;
	configValue.push_back(i32);


	// Check for extra fields.
	return strtok(nullptr, delim) == nullptr;
}
//------------------------------------------------------------------------------

void SD_card::configLoadSD() {
	openFile(FILE_WRITE);
	file.rewind();  // Rewind the file for read.

	while (file.available()) {
		int n = file.fgets(line, sizeof(line));
		if (n <= 0) {
			errorHalt("fgets failed");
		}
		if (line[n-1] != '\n' && n == (sizeof(line) - 1)) {
			errorHalt("line too long");
		}
		if (!parseLine(line)) {
			errorHalt("parseLine failed");
		}
	}

	#if LOG_VERBOSE
		std::cout << "configName:" << std::endl;
		for (const auto& tok : configName) {
			std::cout << tok  << " ";
		}
		std::cout << std::endl;

		std::cout << "configValue: ";
		for (int i : configValue) {
			std::cout << i << " ";
		}
		std::cout << std::endl;
	#endif

	logger::logln("configName & configValue arrays build");

	main::use_adafruit = 		 configValue[0] == 1;
	main::use_u8g2 = 			 configValue[1] == 1;
	main::small = 				 configValue[2] == 1;
	main::display_demo = 		 configValue[3] == 1;
	main::use_round = 			 configValue[4] == 1;
	main::use_menu = 			 configValue[5] == 1;
	main::log_debug = 			 configValue[6] == 1;
	main::use_ps4 = 			 configValue[7] == 1;
	main::use_sd_card = 		 configValue[8] == 1;
	main::use_gyro = 			 configValue[9] == 1;
	main::use_compass = 		 configValue[10] == 1;
	main::use_barometer = 		 configValue[11] == 1;
	main::use_distance = 		 configValue[12] == 1;
	main::use_irremote = 		 configValue[13] == 1;
	main::use_i2c_scanner = 	 configValue[14] == 1;
	main::use_pwm_board = 		 configValue[15] == 1;
	main::use_dot = 			 configValue[16] == 1;
	main::use_mic = 			 configValue[17] == 1;
	main::use_switch = 			 configValue[18] == 1;
	main::use_analog = 			 configValue[19] == 1;
	main::use_robot = 			 configValue[20] == 1;
	main::use_timers = 			 configValue[21] == 1;
	main::use_matrix = 			 configValue[22] == 1;
	main::use_matrix_preview =	 configValue[23] == 1;
	main::read_esp32 = 			 configValue[24] == 1;
	main::use_lcd = 			 configValue[25] == 1;
	main::use_hm_10_ble = 		 configValue[26] == 1;

	logger::logln("Settings loaded from SD");

	logger::log("use_adafruit: ");
	logger::logln	(main::use_adafruit ? "true": "false");
	logger::log("use_u8g2: ");
	logger::logln	(main::use_u8g2  ? "true": "false");
	logger::log("small: ");
	logger::logln	(main::small ? "true": "false");
	logger::log("display_demo: ");
	logger::logln	(main::display_demo ? "true": "false");
	logger::log("use_round: ");
	logger::logln	(main::use_round ? "true": "false");
	logger::log("use_menu: ");
	logger::logln	(main::use_menu ? "true": "false");
	logger::log("log_debug: ");
	logger::logln	(main::log_debug ? "true": "false");
	logger::log("use_ps4: ");
	logger::logln	(main::use_ps4 ? "true": "false");
	logger::log("use_sd_card: ");
	logger::logln	(main::use_sd_card ? "true": "false");
	logger::log("use_gyro: ");
	logger::logln	(main::use_gyro ? "true": "false");
	logger::log("use_compass: ");
	logger::logln	(main::use_compass ? "true": "false");
	logger::log("use_barometer: ");
	logger::logln	(main::use_barometer ? "true": "false");
	logger::log("use_distance: ");
	logger::logln	(main::use_distance ? "true": "false");
	logger::log("use_irremote: ");
	logger::logln	(main::use_irremote ? "true": "false");
	logger::log("use_i2c_scanner: ");
	logger::logln	(main::use_i2c_scanner ? "true": "false");
	logger::log("use_pwm_board: ");
	logger::logln	(main::use_pwm_board ? "true": "false");
	logger::log("use_dot: ");
	logger::logln	(main::use_dot ? "true": "false");
	logger::log("use_mic: ");
	logger::logln	(main::use_mic ? "true": "false");
	logger::log("use_switch: ");
	logger::logln	(main::use_switch ? "true": "false");
	logger::log("use_analog: ");
	logger::logln	(main::use_analog ? "true": "false");
	logger::log("use_robot: ");
	logger::logln	(main::use_robot ? "true": "false");
	logger::log("use_timers: ");
	logger::logln	(main::use_timers ? "true": "false");
	logger::log("use_matrix: ");
	logger::logln	(main::use_matrix ? "true": "false");
	logger::log("use_matrix_preview: ");
	logger::logln	(main::use_matrix_preview ? "true": "false");
	logger::log("read_esp32: ");
	logger::logln	(main::read_esp32 ? "true": "false");
	logger::log("use_lcd: ");
	logger::logln	(main::use_lcd ? "true": "false");
	logger::log("use_hm_10_ble: ");
	logger::logln	(main::use_hm_10_ble ? "true": "false");

 	logger::logln("Config Loaded from SD");
	file.close();
}

void SD_card::configSaveSD() {// Create or open the file.
	openFile(FILE_WRITE);
	logger::logln("Saving to SD");
	file.rewind(); // Rewind file so config data is not appended.

	String use_adafruit_string =	 	"USE_ADAFRUIT," + 		String(main::use_adafruit) + "\r\n";
	String use_u8g2_string =		 	"USE_U8G2," + 			String(main::use_u8g2) + "\r\n";
	String small_string =			 	"SMALL," + 				String(main::small) + "\r\n";
	String display_demo_string =	 	"DISPLAY_DEMO," + 		String(main::display_demo) + "\r\n";
	String use_round_string =		 	"USE_ROUND," + 			String(main::use_round) + "\r\n";
	String use_menu_string =		 	"USE_MENU," + 			String(main::use_menu) + "\r\n";
	String log_debug_string =		 	"LOG_DEBUG," + 			String(main::log_debug) + "\r\n";
	String use_ps4_string =			 	"USE_PS4," + 			String(main::use_ps4) + "\r\n";
	String use_sd_card_string =		 	"USE_SD_CARD," + 		String(main::use_sd_card) + "\r\n";
	String use_gyro_string =		 	"USE_GYRO," + 			String(main::use_gyro) + "\r\n";
	String use_compass_string =		 	"USE_COMPASS," + 		String(main::use_compass) + "\r\n";
	String use_barometer_string =	 	"USE_BAROMETER," + 		String(main::use_barometer) + "\r\n";
	String use_distance_string =	 	"USE_DISTANCE," + 		String(main::use_distance) + "\r\n";
	String use_irremote_string =	 	"USE_IRREMOTE," + 		String(main::use_irremote) + "\r\n";
	String use_i2c_scanner_string =	 	"USE_I2C_SCANNER," + 	String(main::use_i2c_scanner) + "\r\n";
	String use_pwm_board_string =	 	"USE_PWM_BOARD," + 		String(main::use_pwm_board) + "\r\n";
	String use_dot_string =			 	"USE_DOT," + 			String(main::use_dot) + "\r\n";
	String use_mic_string =			 	"USE_MIC," + 			String(main::use_mic) + "\r\n";
	String use_switch_string =		 	"USE_SWITCH," + 		String(main::use_switch) + "\r\n";
	String use_analog_string =		 	"USE_ANALOG," + 		String(main::use_analog) + "\r\n";
	String use_robot_string =		 	"USE_ROBOT," + 			String(main::use_robot) + "\r\n";
	String use_timers_string =		 	"USE_TIMERS," + 		String(main::use_timers) + "\r\n";
	String use_matrix_string =		 	"USE_MATRIX," + 		String(main::use_matrix) + "\r\n";
	String use_matrix_preview_string =	"USE_MATRIX_PREVIEW," + String(main::use_matrix_preview) + "\r\n";
	String read_esp32_string =		 	"READ_ESP32," + 		String(main::read_esp32) + "\r\n";
	String use_lcd_string =			 	"USE_LCD," + 			String(main::use_lcd) + "\r\n";
	String use_hm_10_ble_string =	 	"USE_HM_10_BLE," + 		String(main::use_hm_10_ble);

	// Write config data.
	file.print(F(
			use_adafruit_string +
			use_u8g2_string +
			small_string +
			display_demo_string +
			use_round_string +
			use_menu_string +
			log_debug_string +
			use_ps4_string +
			use_sd_card_string +
			use_gyro_string +
			use_compass_string +
			use_barometer_string +
			use_distance_string +
			use_irremote_string +
			use_i2c_scanner_string +
			use_pwm_board_string +
			use_dot_string +
			use_mic_string +
			use_switch_string +
			use_analog_string +
			use_robot_string +
			use_timers_string +
			use_matrix_string +
			use_matrix_preview_string +
			read_esp32_string +
			use_lcd_string +
			use_hm_10_ble_string
   ));

	file.close();
	logger::logln("Save Complete");
}

void openFile(int rule) {
	file = SD.open("SETUP.TXT", rule);
	if (!file) {
		errorHalt("open failed");
	}
}





