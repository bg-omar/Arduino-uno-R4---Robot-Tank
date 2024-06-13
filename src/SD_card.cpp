//
// Created by mr on 6/8/2024.
//

#include "SD_card.h"
#include "main_ra.h"
#include "config.h"

#include <SPI.h>
#include <SdFat.h>
#define CS_PIN 10

// 5 X 4 array
#define ROW_DIM 27
#define COL_DIM 2

SdFat SD;
File file;

void configSaveSD();

void openFile(int rule);

bool stringToBool(String str) {
	str.toLowerCase();
	if (str == "true" || str == "1") {
		return true;
	} else {
		return false;
	}
}

size_t readField(File* file, char* str, size_t size, const char* delim) {
	char ch;
	size_t n = 0;
	while ((n + 1) < size && file->read(&ch, 1) == 1) {
		// Delete CR.
		if (ch == '\r') {
			continue;
		}
		str[n++] = ch;
		if (strchr(delim, ch)) {
			break;
		}
	}
	str[n] = '\0';
	return n;
}
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
		main::logln("SD card init failed");
		return;
	} else {
		main::logln("SD card initialized");
	}

}
void SD_card::configLoadSD() {
	// Array for data.
	openFile(FILE_READ);
	file.rewind();  // Rewind the file for read.

	String array[ROW_DIM][COL_DIM];
	// Access an element of the array
	String i_str, j_str;  // Strings to hold i and j values
	int i = 0;     // First array index.
	int j = 0;     // Second array index
	size_t n;      // Length of returned field with delimiter.
	String value = array[i][j];
	char str[56];  // Must hold longest field with delimiter and zero byte.
	char *ptr;     // Test for valid field.

	// Read the file and store the data.
	for (i = 0; i < ROW_DIM; i++) {
		for (j = 0; j < COL_DIM; j++) {
			n = readField(&file, str, sizeof(str), ",\n");
			if (n == 0) {
				errorHalt("Too few lines");
			}

			array[i][j] = String(str);

			while (*ptr == ' ') {
				ptr++;
			}
			if (*ptr != ',' && *ptr != '\n' && *ptr != '\0') {
				errorHalt("extra characters in field");
			}
			if (j < (COL_DIM-1) && str[n-1] != ',') {
				errorHalt("line with too few fields");
			}
		}
		// Allow missing endl at eof.
		if (str[n-1] != '\n' && file.available()) {
			errorHalt("missing endl");
		}
	}
	// Print the array.
	for (i = 0; i < ROW_DIM; i++) {
		for (j = 0; j < COL_DIM; j++) {
			if (j) {
				Serial.print(' ');
			}
			main::log((array[i][j]).c_str());
		}
		main::logln("");
	}
	Serial.println("Done");
	file.close();

	main::use_adafruit = 		(array[0][1] == "1");
	main::use_u8g2 = 			(array[1][1] == "1");
	main::small = 				(array[2][1] == "1");
	main::display_demo = 		(array[3][1] == "1");
	main::use_round = 			(array[4][1] == "1");
	main::use_menu = 			(array[5][1] == "1");
	main::log_debug = 			(array[6][1] == "1");
	main::use_ps4 = 			(array[7][1] == "1");
	main::use_sd_card = 		(array[8][1] == "1");
	main::use_gyro = 			(array[9][1] == "1");
	main::use_compass = 		(array[10][1] == "1");
	main::use_barometer = 		(array[11][1] == "1");
	main::use_distance = 		(array[12][1] == "1");
	main::use_irremote = 		(array[13][1] == "1");
	main::use_i2c_scanner = 	(array[14][1] == "1");
	main::use_pwm_board = 		(array[15][1] == "1");
	main::use_dot = 			(array[16][1] == "1");
	main::use_mic = 			(array[17][1] == "1");
	main::use_switch = 			(array[18][1] == "1");
	main::use_analog = 			(array[19][1] == "1");
	main::use_robot = 			(array[20][1] == "1");
	main::use_timers = 			(array[21][1] == "1");
	main::use_matrix = 			(array[22][1] == "1");
	main::use_matrix_preview =	(array[23][1] == "1");
	main::read_esp32 = 			(array[24][1] == "1");
	main::use_lcd = 			(array[25][1] == "1");
	main::use_hm_10_ble = 		(array[26][1] == "1");
}

void configSaveSD() {// Create or open the file.
	openFile(FILE_WRITE);
	// Rewind file so test data is not appended.
	file.rewind();

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
	String use_hm_10_ble_string =	 	"USE_HM_10_BLE," + 		String(main::use_hm_10_ble) + "\r\n";


	// Write test data.
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
			use_hm_10_ble_string +
		    "END,0"
   ));

	file.close();
}

void openFile(int rule) {
	file = SD.open("SETUP.TXT", rule);
	if (!file) {
		errorHalt("open failed");
	}
}

//------------------------------------------------------------------------------




