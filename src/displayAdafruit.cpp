//
// Created by mr on 11/19/2023.
//

#include "displayAdafruit.h"
#include <Wire.h>
#include "config.h"
#include "main_ra.h"
#include "logger.h"
#include <cstdint>
int displaySwitch = 1;
const int LOG_LINES = 11;
const int LINE_LENGTH = 41;
char logBuffer[LOG_LINES][LINE_LENGTH];
int currentLogIndex = 0;

Adafruit_SH1106G displayAdafruit::display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

static const unsigned char PROGMEM logo16_glcd_bmp[] =
		{ B00000000, B11000000,
		  B00000001, B11000000,
		  B00000001, B11000000,
		  B00000011, B11100000,
		  B11110011, B11100000,
		  B11111110, B11111000,
		  B01111110, B11111111,
		  B00110011, B10011111,
		  B00011111, B11111100,
		  B00001101, B01110000,
		  B00011011, B10100000,
		  B00111111, B11100000,
		  B00111111, B11110000,
		  B01111100, B11110000,
		  B01110000, B01110000,
		  B00000000, B00110000
		};


void displayAdafruit::setupAdafruit(){
	delay(250); // wait for the OLED to power up

	if (!display.begin(SCREEN_ADDRESS, true)){
		main::use_adafruit = false;
		main::Found_Display = false;
		Serial.println("Adafruit Display Failed");
	} else {
		main::Found_Display = true;
		display.clearDisplay();
		display.display();
		display.setFont(&TomThumb);
		logger::logln("Display Started");

		display.setTextWrap(false);
		display.setTextSize(1);
		display.setTextColor(SH110X_WHITE);
	}
}


void displayAdafruit::displayLoop(){
	displayAdafruit::animateScreen();
}

void displayAdafruit::updateDisplay() {
	display.clearDisplay();
	display.setCursor(0, 0);
	for (int i = 0; i < LOG_LINES; ++i) {
		int index = (currentLogIndex + i) % LOG_LINES;
		display.println(logBuffer[index]);
	}
	display.display();
}
/************************************************ Display Adafruit  *************************************************/
// section Display Adafruit
/***************************************************************************************************************/
void displayAdafruit::text(const char * text) {
	strncpy(logBuffer[currentLogIndex], text, LINE_LENGTH);
	logBuffer[currentLogIndex][LINE_LENGTH - 1] = '\0'; // Ensure null termination
	currentLogIndex = (currentLogIndex + 1) % LOG_LINES;
	updateDisplay();
}

void displayAdafruit::textln(const char * text) {
	strncpy(logBuffer[currentLogIndex], text, LINE_LENGTH);
	logBuffer[currentLogIndex][LINE_LENGTH - 1] = '\0'; // Ensure null termination
	currentLogIndex = (currentLogIndex + 1) % LOG_LINES;
	updateDisplay();
}

void displayAdafruit::Int(int inter) {
	display.print(inter);
	display.display();
}

void displayAdafruit::Intln(int inter) {
	display.println(inter);
	display.display();
}

void displayAdafruit::Float(float floaty) {
	// text display tests
	display.print(floaty);
	display.display();
}

void displayAdafruit::Floatln(float floaty) {
	// text display tests
	display.println(floaty);
	display.display();
}

void displayAdafruit::hex(unsigned char hexa) {
	display.print("0x"); display.print(hexa, HEX);
	display.display();
}

void displayAdafruit::hexln(unsigned char hexa) {
	display.print("0x"); display.println(hexa, HEX);
	display.display();
}

void displayAdafruit::bitmap(unsigned char bmp []) {
	display.drawBitmap(30, 16, bmp, 16, 16, 1);
	display.display();
	delay(1);
}


//u8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//8G2_SH1106_128X64_NONAME_1_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//// GLOBAL VARIABLES
const int framesPerSecond = 3;
int displayAdafruit::incoming;

// *** PICK THE ENVIRONMENT YOUR CREATURE LIVES IN ***
// 1 = Desert, 2 = Forest, 3 = Water
int displayAdafruit::environment = 1;
int displayAdafruit::petStatus = 1; // 0=HAPPY, 1=SAD
////

////////// HERE ARE WHERE THE INSTRUCTIONS FOR ANIMATION FRAMES GO
//NOTE: screen dimensions: 128x64
//NOTE: use the u8g2 library to write instructions for drawing images (for example using the shape and line functions)
//NOTE: only add your desired drawing functions, buffering/clearing/timing is handled for you :)
// https://github.com/olikraus/u8g2/wiki/u8g2reference#drawbox
//
void displayAdafruit::happyFrame1() {  //THE FIRST FRAME OF THE 'HAPPY' ANIMATION
	display.drawTriangle(46,4,46,16,55,16, SH110X_WHITE); // left ear
	display.drawTriangle(81,4,73,16,82,16, SH110X_WHITE); // right ear
	display.drawRect(46,15,36,33, SH110X_WHITE); //face
	drawEllipse(56.5, 25.5, 2, 2, SH110X_WHITE); // left eye
	drawEllipse(71.5, 25.5, 2, 2, SH110X_WHITE); // right eye
	display.drawTriangle(61.5,34, 67.5,34, 64.5, 36.67, SH110X_WHITE); // nose
	display.drawLine(64.5,36.5,64.5,40.5, SH110X_WHITE); // middle nose
	display.drawLine(59.5,35.5,52.5,33.5, SH110X_WHITE); // left top wisk
	display.drawLine(59.5,37.5,52.5,37.5, SH110X_WHITE); //left bottom wisk
	display.drawLine(69.5,35.5,76.5,33.5, SH110X_WHITE); // right top wisk
	display.drawLine(69.5,37.5,76.5,37.5, SH110X_WHITE); // right bottom wisk
	display.drawLine(59.5,40.5,62,43, SH110X_WHITE); // mouth 1
	display.drawLine(64.5,40.5,62,43, SH110X_WHITE); // mouth 2
	display.drawLine(64.5,40.5,67,43, SH110X_WHITE); // mouth 3
	display.drawLine(69.5,40.5,67,43, SH110X_WHITE); // mouth 4
}
void displayAdafruit::happyFrame2() {  //THE SECOND FRAME OF THE 'HAPPY' ANIMATION
	display.drawTriangle(46,4,46,16,55,16, SH110X_WHITE); // left ear
	display.drawTriangle(81,4,73,16,82,16, SH110X_WHITE); // right ear
	display.drawRect(46,15,36,33, SH110X_WHITE); //face
	drawEllipse(56.5, 25.5, 2, 2, SH110X_WHITE); // left eye
	drawEllipse(71.5, 25.5, 2, 2, SH110X_WHITE); // right eye
	display.drawTriangle(61.5,33, 67.5,34, 64.5, 36.67, SH110X_WHITE); // nose
	display.drawLine(64.5,35.5,64.5,39.5, SH110X_WHITE); // middle nose
	display.drawLine(59.5,35.5,52.5,31.5, SH110X_WHITE); // left top wisk
	display.drawLine(59.5,37.5,52.5,36.5, SH110X_WHITE); //left bottom wisk
	display.drawLine(69.5,35.5,76.5,31.5, SH110X_WHITE); // right top wisk
	display.drawLine(76.5,36.5,69.5,37.5, SH110X_WHITE); // right bottom wisk
	display.drawLine(59.5,39.5,62,42, SH110X_WHITE); // mouth 1
	display.drawLine(64.5,39.5,62,42, SH110X_WHITE); // mouth 2
	display.drawLine(64.5,39.5,67,42, SH110X_WHITE); // mouth 3
	display.drawLine(69.5,39.5,67,42, SH110X_WHITE); // mouth 4
}
void displayAdafruit::happyFrame3() {  //THE THIRD FRAME OF THE 'HAPPY' ANIMATION
	display.drawTriangle(46,4,46,16,55,16, SH110X_WHITE); // left ear
	display.drawTriangle(81,4,73,16,82,16, SH110X_WHITE); // right ear
	display.drawRect(46,15,36,33, SH110X_WHITE); //face
	drawEllipse(56.5, 25.5, 2, 2, SH110X_WHITE); // left eye
	drawEllipse(71.5, 25.5, 2, 2, SH110X_WHITE); // right eye
	display.drawTriangle(61.5,34, 67.5,34, 64.5, 36.67, SH110X_WHITE); // nose
	display.drawLine(64.5,36.5,64.5,40.5, SH110X_WHITE); // middle nose
	display.drawLine(59.5,35.5,52.5,33.5, SH110X_WHITE); // left top wisk
	display.drawLine(59.5,37.5,52.5,37.5, SH110X_WHITE); //left bottom wisk
	display.drawLine(69.5,35.5,76.5,33.5, SH110X_WHITE); // right top wisk
	display.drawLine(69.5,37.5,76.5,37.5, SH110X_WHITE); // right bottom wisk
	display.drawLine(59.5,40.5,62,43, SH110X_WHITE); // mouth 1
	display.drawLine(64.5,40.5,62,43, SH110X_WHITE); // mouth 2
	display.drawLine(64.5,40.5,67,43, SH110X_WHITE); // mouth 3
	display.drawLine(69.5,40.5,67,43, SH110X_WHITE); // mouth 4
}

void displayAdafruit::sadFrame1() {  //THE FIRST FRAME OF THE 'SAD' ANIMATION
	display.drawTriangle(46,4,46,16,55,16, SH110X_WHITE); // left ear
	display.drawTriangle(81,4,73,16,82,16, SH110X_WHITE); // right ear
	display.drawRect(46,15,36,33, SH110X_WHITE); //face
	drawEllipse(56.5, 25.5, 2, 2, SH110X_WHITE); // left eye
	drawEllipse(71.5, 25.5, 2, 2, SH110X_WHITE); // right eye
	display.drawTriangle(61.5,34, 67.5,34, 64.5, 36.67, SH110X_WHITE); // nose
	display.drawLine(64.5,36.5,64.5,40.5, SH110X_WHITE); // middle nose
	display.drawLine(59.5,35.5,52.5,33.5, SH110X_WHITE); // left top wisk
	display.drawLine(59.5,37.5,52.5,37.5, SH110X_WHITE); //left bottom wisk
	display.drawLine(69.5,35.5,76.5,33.5, SH110X_WHITE); // right top wisk
	display.drawLine(69.5,37.5,76.5,37.5, SH110X_WHITE); // right bottom wisk
	display.drawLine(59.5,40.5,62,43, SH110X_WHITE); // mouth 1
	display.drawLine(64.5,40.5,62,43, SH110X_WHITE); // mouth 2
	display.drawLine(64.5,40.5,67,43, SH110X_WHITE); // mouth 3
	display.drawLine(69.5,40.5,67,43, SH110X_WHITE); // mouth 4
}
void displayAdafruit::sadFrame2() {  //THE SECOND FRAME OF THE 'SAD' ANIMATION
	display.drawTriangle(46,4,46,16,55,16, SH110X_WHITE); // left ear
	display.drawTriangle(81,4,73,16,82,16, SH110X_WHITE); // right ear
	display.drawRect(46,15,36,33, SH110X_WHITE); //face
	display.drawLine(59.5,22.5,52.5,24.5, SH110X_WHITE); // low brow left
	display.drawLine(69.5,22.5,76.5,24.5, SH110X_WHITE); // low brow right
	drawEllipse(56.5, 25.5, 1, 1, SH110X_WHITE); // left eye
	drawEllipse(71.5, 25.5, 1, 1, SH110X_WHITE); // right eye
	display.drawTriangle(61.5,34, 67.5,34, 64.5, 36.67, SH110X_WHITE); // nose
	display.drawLine(64.5,36.5,64.5,40.5, SH110X_WHITE); // middle nose
	display.drawLine(59.5,35.5,52.5,35.5, SH110X_WHITE); // left top wisk
	display.drawLine(59.5,37.5,52.5,39.5, SH110X_WHITE); //left bottom wisk
	display.drawLine(69.5,35.5,76.5,35.5, SH110X_WHITE); // right top wisk
	display.drawLine(69.5,37.5,76.5,39.5, SH110X_WHITE); // right bottom wisk
	display.drawLine(58.5,41.5,62,43, SH110X_WHITE); // mouth 1
	display.drawLine(64.5,40.5,62,43, SH110X_WHITE); // mouth 2
	display.drawLine(64.5,40.5,67,43, SH110X_WHITE); // mouth 3
	display.drawLine(70.5,41.5,67,43, SH110X_WHITE); // mouth 4
}
void displayAdafruit::sadFrame3() {  //THE THIRD FRAME OF THE 'SAD' ANIMATION
	display.drawTriangle(46,4,46,16,55,16, SH110X_WHITE); // left ear
	display.drawTriangle(81,4,73,16,82,16, SH110X_WHITE); // right ear
	display.drawRect(46,15,36,33, SH110X_WHITE); //face
	drawEllipse(56.5, 25.5, 2, 2, SH110X_WHITE); // left eye
	drawEllipse(71.5, 25.5, 2, 2, SH110X_WHITE); // right eye
	display.drawTriangle(61.5,34, 67.5,34, 64.5, 36.67, SH110X_WHITE); // nose
	display.drawLine(64.5,36.5,64.5,40.5, SH110X_WHITE); // middle nose
	display.drawLine(59.5,35.5,52.5,33.5, SH110X_WHITE); // left top wisk
	display.drawLine(59.5,37.5,52.5,37.5, SH110X_WHITE); //left bottom wisk
	display.drawLine(69.5,35.5,76.5,33.5, SH110X_WHITE); // right top wisk
	display.drawLine(69.5,37.5,76.5,37.5, SH110X_WHITE); // right bottom wisk
	display.drawLine(59.5,40.5,62,43, SH110X_WHITE); // mouth 1
	display.drawLine(64.5,40.5,62,43, SH110X_WHITE); // mouth 2
	display.drawLine(64.5,40.5,67,43, SH110X_WHITE); // mouth 3
	display.drawLine(69.5,40.5,67,43, SH110X_WHITE); // mouth 4
}
//////////END OF ANIMATION FRAME INSTRUCTIONS


//// YOU DO NOT HAVE TO MODIFY THE REST OF THE CODE:
void displayAdafruit::animateScreen() {
	// the pet's status is checked every time animateScreen() is called
	if (displayAdafruit::petStatus == 0) { //IF PET IS HAPPY, DO THIS CODE:
		display.clearDisplay();
		displayAdafruit::happyFrame1();
		display.display();
		delay(1000 / framesPerSecond);

		display.clearDisplay();
		displayAdafruit::happyFrame2();
		display.display();
		delay(1000 / framesPerSecond);

		display.clearDisplay();
		displayAdafruit::happyFrame3();
		display.display();
		delay(1000 / framesPerSecond);

		display.clearDisplay();
		displayAdafruit::happyFrame2();
		display.display();
		delay(1000 / framesPerSecond);
	}
	else { //IF PET IS NOT HAPPY, IT'S SAD. DO THIS CODE:
		display.clearDisplay();
		displayAdafruit::sadFrame1();
		display.display();
		delay(1000 / framesPerSecond);

		display.clearDisplay();
		displayAdafruit::sadFrame2();
		display.display();
		delay(1000 / framesPerSecond);

		display.clearDisplay();
		displayAdafruit::sadFrame3();
		display.display();
		delay(1000 / framesPerSecond);

		display.clearDisplay();
		displayAdafruit::sadFrame2();
		display.display();
		delay(1000 / framesPerSecond);
	}
}

void displayAdafruit::drawEllipse(int x0, int y0, int a, int b, uint8_t color) {
	int x = 0, y = b;
	int a2 = a * a, b2 = b * b;
	int crit1 = -(a2/4 + a%2 + b2);
	int crit2 = -(b2/4 + b%2 + a2);
	int crit3 = -(b2/4 + b%2);
	int t = -a2*y;
	int dxt = 2*b2*x, dyt = -2*a2*y;
	int d2xt = 2*b2, d2yt = 2*a2;
	while (y >= 0 && x <= a) {
		display.drawLine(x0 + x, y0 + y, x0 - x, y0 + y, color);
		display.drawLine(x0 + x, y0 - y, x0 - x, y0 - y, color);
		if (2*t + x*d2yt < crit1 || 2*t + y*d2xt < crit3) {
			x++;
			dxt += d2xt;
			t += dxt;
		}
		else if (2*t - y*d2xt > crit2) {
			y--;
			dyt += d2yt;
			t += dyt;
		}
		else {
			x++;
			y--;
			dxt += d2xt;
			dyt += d2yt;
			t += dxt + dyt;
		}
	}
}


