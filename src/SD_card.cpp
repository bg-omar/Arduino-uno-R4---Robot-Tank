//
// Created by mr on 6/8/2024.
//

#include "SD_card.h"
#include "main_ra.h"
#include "config.h"


// Functions to read a CSV text file one field at a time.
//
#include <climits>
#include <SPI.h>

// next line for SD.h
//#include <SD.h>

// next two lines for SdFat
#include <SdFat.h>
SdFat SD;

// example can use comma or semicolon
#define CSV_DELIM ','

File file;

/*
 * Read a file one field at a time.
 *
 * file - File to read.
 *
 * str - Character array for the field.
 *
 * size - Size of str array.
 *
 * delim - csv delimiter.
 *
 * return - negative value for failure.
 *          delimiter, '\n' or zero(EOF) for success.
 */
int csvReadText(File* file, char* str, size_t size, char delim) {
	char ch;
	int rtn;
	size_t n = 0;
	while (true) {
		// check for EOF
		if (!file->available()) {
			rtn = 0;
			break;
		}
		if (file->read(&ch, 1) != 1) {
			// read error
			rtn = -1;
			break;
		}
		// Delete CR.
		if (ch == '\r') {
			continue;
		}
		if (ch == delim || ch == '\n') {
			rtn = ch;
			break;
		}
		if ((n + 1) >= size) {
			// string too long
			rtn = -2;
			n--;
			break;
		}
		str[n++] = ch;
	}
	str[n] = '\0';
	return rtn;
}
//------------------------------------------------------------------------------
int csvReadInt32(File* file, int32_t* num, char delim) {
	char buf[20];
	char* ptr;
	int rtn = csvReadText(file, buf, sizeof(buf), delim);
	if (rtn < 0) return rtn;
	*num = strtol(buf, &ptr, 10);
	if (buf == ptr) return -3;
	while(isspace(*ptr)) ptr++;
	return *ptr == 0 ? rtn : -4;
}
//------------------------------------------------------------------------------
int csvReadInt16(File* file, int16_t* num, char delim) {
	int32_t tmp;
	int rtn = csvReadInt32(file, &tmp, delim);
	if (rtn < 0) return rtn;
	if (tmp < INT_MIN || tmp > INT_MAX) return -5;
	*num = tmp;
	return rtn;
}
//------------------------------------------------------------------------------
int csvReadUint32(File* file, uint32_t* num, char delim) {
	char buf[20];
	char* ptr;
	int rtn = csvReadText(file, buf, sizeof(buf), delim);
	if (rtn < 0) return rtn;
	*num = strtoul(buf, &ptr, 10);
	if (buf == ptr) return -3;
	while(isspace(*ptr)) ptr++;
	return *ptr == 0 ? rtn : -4;
}
//------------------------------------------------------------------------------
int csvReadUint16(File* file, uint16_t* num, char delim) {
	uint32_t tmp;
	int rtn = csvReadUint32(file, &tmp, delim);
	if (rtn < 0) return rtn;
	if (tmp > UINT_MAX) return -5;
	*num = tmp;
	return rtn;
}
//------------------------------------------------------------------------------
int csvReadDouble(File* file, double* num, char delim) {
	char buf[20];
	char* ptr;
	int rtn = csvReadText(file, buf, sizeof(buf), delim);
	if (rtn < 0) return rtn;
	*num = strtod(buf, &ptr);
	if (buf == ptr) return -3;
	while(isspace(*ptr)) ptr++;
	return *ptr == 0 ? rtn : -4;
}
//------------------------------------------------------------------------------
int csvReadFloat(File* file, float* num, char delim) {
	double tmp;
	int rtn = csvReadDouble(file, &tmp, delim);
	if (rtn < 0)return rtn;
	// could test for too large.
	*num = tmp;
	return rtn;
}
//------------------------------------------------------------------------------
void SD_card::initSD() {
	main::logln("Initializing SD card...");

	// Wait for USB Serial
	while (!Serial) {
		yield();
	}
	main::logln("Type any character to start");
	while (!Serial.available()) {
		yield();
	}
	// Initialize the SD.
	if (!SD.begin(CS_PIN)) {
		main::logln("begin failed");
		return;
	}
	// Remove existing file.
	SD.remove("READTEST.TXT");

	// Create the file.
	file = SD.open("READTEST.TXT", FILE_WRITE);
	if (!file) {
		main::logln("open failed");
		return;
	}
	// Write test data.
	file.print(F(
	#if CSV_DELIM == ','
				"36,23.20,20.70,57.60,79.50,01:08:14,23.06.16\r\n"
				"37,23.21,20.71,57.61,79.51,02:08:14,23.07.16\r\n"
	#elif CSV_DELIM == ';'
				"36;23.20;20.70;57.60;79.50;01:08:14;23.06.16\r\n"
				"37;23.21;20.71;57.61;79.51;02:08:14;23.07.16\r\n"
	#else
		#error "Bad CSV_DELIM"
	#endif
	));

	// Rewind the file for read.
	file.seek(0);

	// Read the file and print fields.
	int16_t tcalc;
	const char *h2;
	const char *h1;
	const char *t2;
	const char *t1;
	// Must be dim 9 to allow for zero byte.
	char timeS[9], dateS[9];
	while (file.available()) {
		if (csvReadInt16(&file, &tcalc, CSV_DELIM) != CSV_DELIM
			|| csvReadFloat(&file, (float *) &t1, CSV_DELIM) != CSV_DELIM
			|| csvReadFloat(&file, (float *) &t2, CSV_DELIM) != CSV_DELIM
			|| csvReadFloat(&file, (float *) &h1, CSV_DELIM) != CSV_DELIM
			|| csvReadFloat(&file, (float *) &h2, CSV_DELIM) != CSV_DELIM
			|| csvReadText(&file, timeS, sizeof(timeS), CSV_DELIM) != CSV_DELIM
			|| csvReadText(&file, dateS, sizeof(dateS), CSV_DELIM) != '\n') {
			main::logln("read error");
			int ch;
			int nr = 0;
			// print part of file after error.
			while ((ch = file.read()) > 0 && nr++ < 100) {
				Serial.write(ch);
			}
			break;
		}
		main::log(reinterpret_cast<const char *>(tcalc), 0);
		main::log(reinterpret_cast<const char *>(CSV_DELIM), 0);
		main::log(t1, 0);
		main::log(reinterpret_cast<const char *>(CSV_DELIM), 0);
		main::log(t2, 0);
		main::log(reinterpret_cast<const char *>(CSV_DELIM), 0);
		main::log(h1, 0);
		main::log(reinterpret_cast<const char *>(CSV_DELIM), 0);
		main::log(h2, 0);
		main::log(reinterpret_cast<const char *>(CSV_DELIM), 0);
		main::log(reinterpret_cast<const char *>(timeS), 0);
		Serial.print(CSV_DELIM);
		main::logln(dateS);
	}
	file.close();
}
