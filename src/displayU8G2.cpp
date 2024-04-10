//
// Created by mr on 11/13/2023.
//

#include "displayU8G2.h"
#include "main_ra.h"

#if SMALL
U8G2_SSD1306_128X64_NONAME_1_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#else
    U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#endif

#if LOG_DEBUG
uint8_t u8log_buffer[U8LOG_WIDTH * U8LOG_HEIGHT]{};
U8G2LOG displayU8G2::u8g2log;
#endif
uint8_t displayU8G2::draw_state = 0;
int displayU8G2::t = 0;

void displayU8G2::U8G2setup() {
    if (!display.begin()) {
        Serial.println(F("U8g2 allocation failed"));
    } else {
        Serial.println(F("U8g2 allocation success"));
        main::Found_Display = true;
        display.clear();
        display.clearDisplay();
        display.begin();
#if LOG_DEBUG
        u8g2log.begin(U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer);
        u8g2log.setLineHeightOffset(0);    // set extra space between lines in pixel, this can be negative
        u8g2log.setRedrawMode(0);        // 0: Update screen with newline, 1: Update screen for every char
        u8g2log.println("U8g2 initialized");

        displayU8G2::U8G2printEnd();
#endif
    }
}

#if LOG_DEBUG
// print the output of millis() to the terminal every second
void displayU8G2::U8G2print(const char * log) {
    u8g2log.print(log);
    displayU8G2::U8G2printEnd();
}

void displayU8G2::U8G2println(const char * log) {
    u8g2log.println(log);
    displayU8G2::U8G2printEnd();
}

void displayU8G2::U8G2printEnd (){
    // print the log window together with a title
    display.firstPage();
    do {
        display.setFont(u8g2_font_5x7_tr);			// set the font for the terminal window
        display.drawLog(0, 7, u8g2log);			// draw the log content on the display
    } while ( display.nextPage() );
}
#endif

void displayU8G2::u8g2_prepare() {
    display.setFont(u8g2_font_6x10_tf);
    display.setFontRefHeightExtendedText();
    display.setDrawColor(1);
    display.setFontPosTop();
    display.setFontDirection(0);
}


//u8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//8G2_SH1106_128X64_NONAME_1_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//// GLOBAL VARIABLES
const int framesPerSecond = 3;
int displayU8G2::incoming;

// *** PICK THE ENVIRONMENT YOUR CREATURE LIVES IN ***
// 1 = Desert, 2 = Forest, 3 = Water
int displayU8G2::environment = 1;
int displayU8G2::petStatus = 1; // 0=HAPPY, 1=SAD
////

////////// HERE ARE WHERE THE INSTRUCTIONS FOR ANIMATION FRAMES GO
//NOTE: screen dimensions: 128x64
//NOTE: use the u8g2 library to write instructions for drawing images (for example using the shape and line functions)
//NOTE: only add your desired drawing functions, buffering/clearing/timing is handled for you :)
// https://github.com/olikraus/u8g2/wiki/u8g2reference#drawbox
//
void displayU8G2::happyFrame1() {  //THE FIRST FRAME OF THE 'HAPPY' ANIMATION
    display.drawTriangle(46,4,46,16,55,16); // left ear
    display.drawTriangle(81,4,73,16,82,16); // right ear
    display.drawFrame(46,15,36,33); //face
    display.drawFilledEllipse(56.5, 25.5, 2, 2, U8G2_DRAW_ALL); // left eye
    display.drawFilledEllipse(71.5, 25.5, 2, 2, U8G2_DRAW_ALL); // right eye
    display.drawTriangle(61.5,34, 67.5,34, 64.5, 36.67); // nose
    display.drawLine(64.5,36.5,64.5,40.5); // middle nose
    display.drawLine(59.5,35.5,52.5,33.5); // left top wisk
    display.drawLine(59.5,37.5,52.5,37.5); //left bottom wisk
    display.drawLine(69.5,35.5,76.5,33.5); // right top wisk
    display.drawLine(69.5,37.5,76.5,37.5); // right bottom wisk
    display.drawLine(59.5,40.5,62,43); // mouth 1
    display.drawLine(64.5,40.5,62,43); // mouth 2
    display.drawLine(64.5,40.5,67,43); // mouth 3
    display.drawLine(69.5,40.5,67,43); // mouth 4
}
void displayU8G2::happyFrame2() {  //THE SECOND FRAME OF THE 'HAPPY' ANIMATION
    display.drawTriangle(46,4,46,16,55,16); // left ear
    display.drawTriangle(81,4,73,16,82,16); // right ear
    display.drawFrame(46,15,36,33); //face
    display.drawFilledEllipse(56.5, 25.5, 2, 2, U8G2_DRAW_ALL); // left eye
    display.drawFilledEllipse(71.5, 25.5, 2, 2, U8G2_DRAW_ALL); // right eye
    display.drawTriangle(61.5,33, 67.5,34, 64.5, 36.67); // nose
    display.drawLine(64.5,35.5,64.5,39.5); // middle nose
    display.drawLine(59.5,35.5,52.5,31.5); // left top wisk
    display.drawLine(59.5,37.5,52.5,36.5); //left bottom wisk
    display.drawLine(69.5,35.5,76.5,31.5); // right top wisk
    display.drawLine(76.5,36.5,69.5,37.5); // right bottom wisk
    display.drawLine(59.5,39.5,62,42); // mouth 1
    display.drawLine(64.5,39.5,62,42); // mouth 2
    display.drawLine(64.5,39.5,67,42); // mouth 3
    display.drawLine(69.5,39.5,67,42); // mouth 4
}
void displayU8G2::happyFrame3() {  //THE THIRD FRAME OF THE 'HAPPY' ANIMATION
    display.drawTriangle(46,4,46,16,55,16); // left ear
    display.drawTriangle(81,4,73,16,82,16); // right ear
    display.drawFrame(46,15,36,33); //face
    display.drawFilledEllipse(56.5, 25.5, 2, 2, U8G2_DRAW_ALL); // left eye
    display.drawFilledEllipse(71.5, 25.5, 2, 2, U8G2_DRAW_ALL); // right eye
    display.drawTriangle(61.5,34, 67.5,34, 64.5, 36.67); // nose
    display.drawLine(64.5,36.5,64.5,40.5); // middle nose
    display.drawLine(59.5,35.5,52.5,33.5); // left top wisk
    display.drawLine(59.5,37.5,52.5,37.5); //left bottom wisk
    display.drawLine(69.5,35.5,76.5,33.5); // right top wisk
    display.drawLine(69.5,37.5,76.5,37.5); // right bottom wisk
    display.drawLine(59.5,40.5,62,43); // mouth 1
    display.drawLine(64.5,40.5,62,43); // mouth 2
    display.drawLine(64.5,40.5,67,43); // mouth 3
    display.drawLine(69.5,40.5,67,43); // mouth 4
}

void displayU8G2::sadFrame1() {  //THE FIRST FRAME OF THE 'SAD' ANIMATION
    display.drawTriangle(46,4,46,16,55,16); // left ear
    display.drawTriangle(81,4,73,16,82,16); // right ear
    display.drawFrame(46,15,36,33); //face
    display.drawFilledEllipse(56.5, 25.5, 2, 2, U8G2_DRAW_ALL); // left eye
    display.drawFilledEllipse(71.5, 25.5, 2, 2, U8G2_DRAW_ALL); // right eye
    display.drawTriangle(61.5,34, 67.5,34, 64.5, 36.67); // nose
    display.drawLine(64.5,36.5,64.5,40.5); // middle nose
    display.drawLine(59.5,35.5,52.5,33.5); // left top wisk
    display.drawLine(59.5,37.5,52.5,37.5); //left bottom wisk
    display.drawLine(69.5,35.5,76.5,33.5); // right top wisk
    display.drawLine(69.5,37.5,76.5,37.5); // right bottom wisk
    display.drawLine(59.5,40.5,62,43); // mouth 1
    display.drawLine(64.5,40.5,62,43); // mouth 2
    display.drawLine(64.5,40.5,67,43); // mouth 3
    display.drawLine(69.5,40.5,67,43); // mouth 4
}
void displayU8G2::sadFrame2() {  //THE SECOND FRAME OF THE 'SAD' ANIMATION
    display.drawTriangle(46,4,46,16,55,16); // left ear
    display.drawTriangle(81,4,73,16,82,16); // right ear
    display.drawFrame(46,15,36,33); //face
    display.drawLine(59.5,22.5,52.5,24.5); // low brow left
    display.drawLine(69.5,22.5,76.5,24.5); // low brow right
    display.drawFilledEllipse(56.5, 25.5, 1, 1, U8G2_DRAW_ALL); // left eye
    display.drawFilledEllipse(71.5, 25.5, 1, 1, U8G2_DRAW_ALL); // right eye
    display.drawTriangle(61.5,34, 67.5,34, 64.5, 36.67); // nose
    display.drawLine(64.5,36.5,64.5,40.5); // middle nose
    display.drawLine(59.5,35.5,52.5,35.5); // left top wisk
    display.drawLine(59.5,37.5,52.5,39.5); //left bottom wisk
    display.drawLine(69.5,35.5,76.5,35.5); // right top wisk
    display.drawLine(69.5,37.5,76.5,39.5); // right bottom wisk
    display.drawLine(58.5,41.5,62,43); // mouth 1
    display.drawLine(64.5,40.5,62,43); // mouth 2
    display.drawLine(64.5,40.5,67,43); // mouth 3
    display.drawLine(70.5,41.5,67,43); // mouth 4
}
void displayU8G2::sadFrame3() {  //THE THIRD FRAME OF THE 'SAD' ANIMATION
    display.drawTriangle(46,4,46,16,55,16); // left ear
    display.drawTriangle(81,4,73,16,82,16); // right ear
    display.drawFrame(46,15,36,33); //face
    display.drawFilledEllipse(56.5, 25.5, 2, 2, U8G2_DRAW_ALL); // left eye
    display.drawFilledEllipse(71.5, 25.5, 2, 2, U8G2_DRAW_ALL); // right eye
    display.drawTriangle(61.5,34, 67.5,34, 64.5, 36.67); // nose
    display.drawLine(64.5,36.5,64.5,40.5); // middle nose
    display.drawLine(59.5,35.5,52.5,33.5); // left top wisk
    display.drawLine(59.5,37.5,52.5,37.5); //left bottom wisk
    display.drawLine(69.5,35.5,76.5,33.5); // right top wisk
    display.drawLine(69.5,37.5,76.5,37.5); // right bottom wisk
    display.drawLine(59.5,40.5,62,43); // mouth 1
    display.drawLine(64.5,40.5,62,43); // mouth 2
    display.drawLine(64.5,40.5,67,43); // mouth 3
    display.drawLine(69.5,40.5,67,43); // mouth 4
}
//////////END OF ANIMATION FRAME INSTRUCTIONS


//// YOU DO NOT HAVE TO MODIFY THE REST OF THE CODE:
void displayU8G2::animateScreen(uint8_t a) {
    // the pet's status is checked every time animateScreen() is called
    if (displayU8G2::petStatus == 0) { //IF PET IS HAPPY, DO THIS CODE:
        display.clearBuffer();
        displayU8G2::happyFrame1();
        display.sendBuffer();
        delay(1000 / framesPerSecond);

        display.clearBuffer();
        displayU8G2::happyFrame2();
        display.sendBuffer();
        delay(1000 / framesPerSecond);

        display.clearBuffer();
        displayU8G2::happyFrame3();
        display.sendBuffer();
        delay(1000 / framesPerSecond);

        display.clearBuffer();
        displayU8G2::happyFrame2();
        display.sendBuffer();
        delay(1000 / framesPerSecond);
    }
    else { //IF PET IS NOT HAPPY, IT'S SAD. DO THIS CODE:
        display.clearBuffer();
        displayU8G2::sadFrame1();
        display.sendBuffer();
        delay(1000 / framesPerSecond);

        display.clearBuffer();
        displayU8G2::sadFrame2();
        display.sendBuffer();
        delay(1000 / framesPerSecond);

        display.clearBuffer();
        displayU8G2::sadFrame3();
        display.sendBuffer();
        delay(1000 / framesPerSecond);

        display.clearBuffer();
        displayU8G2::sadFrame2();
        display.sendBuffer();
        delay(1000 / framesPerSecond);
    }
}

void displayU8G2::animate(){
    // picture loop
    // picture loop

    display.firstPage();
    do {
        displayU8G2::draw();
    } while( display.nextPage() );


    displayU8G2::draw_state++;
    if ( displayU8G2::draw_state >= 1*8 )
        displayU8G2::draw_state = 0;

    // delay between each page
    delay(150);

}

void displayU8G2::draw() {
    displayU8G2::u8g2_prepare();
    switch(displayU8G2::draw_state >> 3) {
        case 0: displayU8G2::animateScreen(displayU8G2::draw_state & 7); break;

    }
}






