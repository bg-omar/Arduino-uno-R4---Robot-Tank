//
// Created by mr on 11/19/2023.
//

#include "displayAdafruit.h"
#include <Wire.h>
#include "config.h"
#include <cstdint>
int displaySwitch = 1;



#if SMALL
Adafruit_SSD1306 displayAdafruit::display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#else
Adafruit_SH1106G displayAdafruit::display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif
static const unsigned char PROGMEM logo_bmp[] =
    {0b00000000, 0b11000000,
    0b00000001, 0b11000000,
    0b00000001, 0b11000000,
    0b00000011, 0b11100000,
    0b11110011, 0b11100000,
    0b11111110, 0b11111000,
    0b01111110, 0b11111111,
    0b00110011, 0b10011111,
    0b00011111, 0b11111100,
    0b00001101, 0b01110000,
    0b00011011, 0b10100000,
    0b00111111, 0b11100000,
    0b00111111, 0b11110000,
    0b01111100, 0b11110000,
    0b01110000, 0b01110000,
    0b00000000, 0b00110000
};

/************************************************ Display Adafruit  *************************************************/
// section Display Adafruit
/***************************************************************************************************************/
void displayAdafruit::setupAdafruit(){
#if SMALL
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    }
#else
    display.begin(SCREEN_ADDRESS, true); // Address 0x3C default
            #if USE_ADAFRUIT
                display.display();
            #endif
            display.clearDisplay();
#endif
}


void displayAdafruit::testdrawroundrect() {
        display.clearDisplay();
        display.drawRoundRect(0, 0, display.width()-2, display.height()-2,
                              display.height()/8, PIXEL_WHITE);
        #if USE_ADAFRUIT
    display.display();
#endif
        delay(200);
}

void displayAdafruit::testfillroundrect() {
    display.clearDisplay();
    for(int16_t i=0; i<display.height()/2-2; i+=2) {
        // The INVERSE color is used so round-rects alternate white/black
        display.fillRoundRect(i, i, display.width()-2*i, display.height()-2*i,
                              display.height()/4, PIXEL_INVERSE);
        #if USE_ADAFRUIT
display.display();
#endif
        delay(1);
    }
}

void displayAdafruit::testdrawchar() {
    display.clearDisplay();

    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(PIXEL_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.cp437(true);         // Use full 256 char 'Code Page 437' font

    // Not all the characters will fit on the display. This is normal.
    // Library will draw what it can and the rest will be clipped.
    for(int16_t i=0; i<256; i++) {
        if(i == '\n') display.write(' ');
        else          display.write(i);
    }

    #if USE_ADAFRUIT
display.display();
#endif
    delay(1);
}

void displayAdafruit::testdrawstyles() {
    display.clearDisplay();

    display.setTextSize(2);             // Normal 1:1 pixel scale
    display.setTextColor(PIXEL_WHITE);        // Draw white text
    display.setCursor(0,0);             // Start at top-left corner
    display.println(F("Hellow, Pesto!"));

    display.setTextColor(PIXEL_BLACK, PIXEL_WHITE); // Draw 'inverse' text
    display.println(1337);

    display.setTextSize(2);             // Draw 2X-scale text
    display.setTextColor(PIXEL_WHITE);
    display.print(F("0x")); display.println(0xDEADBEEF, HEX);

    #if USE_ADAFRUIT
display.display();
#endif
}

void displayAdafruit::testscrolltext() {
        display.clearDisplay();

        display.setTextSize(2); // Draw 2X-scale text
        display.setTextColor(PIXEL_WHITE);
        display.setCursor(10, 0);
        display.println(F("PESTO?"));
        #if USE_ADAFRUIT
    display.display();
#endif      // Show initial text
        #if SMALL
            // Scroll in various directions, pausing in-between:
            display.startscrollright(0x00, 0x0F);
            display.stopscroll();
            delay(100);
            display.startscrollleft(0x00, 0x0F);
            display.stopscroll();
            delay(100);
            display.startscrolldiagright(0x00, 0x07);
            display.startscrolldiagleft(0x00, 0x07);
            display.stopscroll();
        #endif
}




void displayAdafruit::displayLoop(){
    switch (displaySwitch) {
            case 1:  testdrawroundrect();  break;
            case 2:  testfillroundrect();  break;
            case 3:  testdrawchar();       break;
            case 4:  testdrawstyles();     break;
            case 5:  testscrolltext();     break;
            default:  display.display();
        }
        displaySwitch == 5 ? displaySwitch = 1 : displaySwitch += 1;
}
