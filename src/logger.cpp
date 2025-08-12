//
// Created by mr on 7/8/2024.
//
#include "Arduino.h"
#include "config.h"
#include "main_ra.h"
#include "displayAdafruit.h"
#include "logger.h"

/********************************************** Setup booting the arduino **************************************/
// section Main Functions
/***************************************************************************************************************/
// Implementation


void logger::log(const char* text) {
	Serial.print(text);
	displayAdafruit::text(text);
}

void logger::logln(const char* text) {
	Serial.println(text);
	displayAdafruit::textln(text);
}

void logger::logDoubble(double floaty) {
	Serial.print(floaty);
	displayAdafruit::Doubble(floaty);
}

void logger::logDoubbleln(double floaty) {
	Serial.println(floaty);
	displayAdafruit::Doubbleln(floaty);
}

void logger::logFloat(float floaty) {
	Serial.print(floaty);
	displayAdafruit::Float(floaty);
}

void logger::logFloatln(float floaty) {
	Serial.println(floaty);
	displayAdafruit::Floatln(floaty);
}

void logger::logInt(int inty) {
	Serial.print(inty);
	displayAdafruit::Int(inty);
}

void logger::logIntln(int inty) {
	Serial.println(inty);
	displayAdafruit::Intln(inty);
}

void logger::logHex(unsigned char hexy, int i) {
	Serial.println(hexy, i);
	displayAdafruit::hex(hexy);
}

void logger::logHexln(unsigned char hexy, int i) {
	Serial.println(hexy, i);
	displayAdafruit::hexln(hexy);
}
