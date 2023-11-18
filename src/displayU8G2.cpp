//
// Created by mr on 11/13/2023.
//

#include "displayU8G2.h"

#if SMALL
U8G2_SSD1306_128X64_NONAME_1_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#else
U8G2_SH1106_128X64_NONAME_1_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#endif
uint8_t displayU8G2::draw_state = 0;
void displayU8G2::u8g2_prepare() {
    display.setFont(u8g2_font_6x10_tf);
    display.setFontRefHeightExtendedText();
    display.setDrawColor(1);
    display.setFontPosTop();
    display.setFontDirection(0);
}

void displayU8G2::u8g2_box_title(uint8_t a) {
    display.drawStr( 10+a*2, 5, "U8g2");
    display.drawStr( 10, 20, "GraphicsTest");

    display.drawFrame(0,0,display.getDisplayWidth(),display.getDisplayHeight() );
}

void displayU8G2::u8g2_box_frame(uint8_t a) {
    display.drawStr( 0, 0, "drawBox");
    display.drawBox(5,10,20,10);
    display.drawBox(10+a,15,30,7);
    display.drawStr( 0, 30, "drawFrame");
    display.drawFrame(5,10+30,20,10);
    display.drawFrame(10+a,15+30,30,7);
}

void displayU8G2::u8g2_disc_circle(uint8_t a) {
    display.drawStr( 0, 0, "drawDisc");
    display.drawDisc(10,18,9);
    display.drawDisc(24+a,16,7);
    display.drawStr( 0, 30, "drawCircle");
    display.drawCircle(10,18+30,9);
    display.drawCircle(24+a,16+30,7);
}

void displayU8G2::u8g2_r_frame(uint8_t a) {
    display.drawStr( 0, 0, "drawRFrame/Box");
    display.drawRFrame(5, 10,40,30, a+1);
    display.drawRBox(50, 10,25,40, a+1);
}
void displayU8G2::u8g2_string(uint8_t a) {
    display.setFontDirection(0);
    display.drawStr(30+a,31, " 0");
    display.setFontDirection(1);
    display.drawStr(30,31+a, " 90");
    display.setFontDirection(2);
    display.drawStr(30-a,31, " 180");
    display.setFontDirection(3);
    display.drawStr(30,31-a, " 270");
}

void displayU8G2::u8g2_line(uint8_t a) {
    display.drawStr( 0, 0, "drawLine");
    display.drawLine(7+a, 10, 40, 55);
    display.drawLine(7+a*2, 10, 60, 55);
    display.drawLine(7+a*3, 10, 80, 55);
    display.drawLine(7+a*4, 10, 100, 55);
}

void displayU8G2::u8g2_triangle(uint8_t a) {
    uint16_t offset = a;
    display.drawStr( 0, 0, "drawTriangle");
    display.drawTriangle(14,7, 45,30, 10,40);
    display.drawTriangle(14+offset,7-offset, 45+offset,30-offset, 57+offset,10-offset);
    display.drawTriangle(57+offset*2,10, 45+offset*2,30, 86+offset*2,53);
    display.drawTriangle(10+offset,40+offset, 45+offset,30+offset, 86+offset,53+offset);
}

void displayU8G2::u8g2_ascii_1() {
    char s[2] = " ";
    uint8_t x, y;
    display.drawStr( 0, 0, "ASCII page 1");
    for( y = 0; y < 6; y++ ) {
        for( x = 0; x < 16; x++ ) {
            s[0] = y*16 + x + 32;
            display.drawStr(x*7, y*10+10, s);
        }
    }
}

void displayU8G2::u8g2_ascii_2() {
    char s[2] = " ";
    uint8_t x, y;
    display.drawStr( 0, 0, "ASCII page 2");
    for( y = 0; y < 6; y++ ) {
        for( x = 0; x < 16; x++ ) {
            s[0] = y*16 + x + 160;
            display.drawStr(x*7, y*10+10, s);
        }
    }
}

void displayU8G2::u8g2_extra_page(uint8_t a)
{
    display.drawStr( 0, 0, "Unicode");
    display.setFont(u8g2_font_unifont_t_symbols);
    display.setFontPosTop();
    display.drawUTF8(0, 24, "☀ ☁");
    switch(a) {
        case 0:
        case 1:
        case 2:
        case 3:
            display.drawUTF8(a*3, 36, "☂");
            break;
        case 4:
        case 5:
        case 6:
        case 7:
            display.drawUTF8(a*3, 36, "☔");
            break;
    }
}

void displayU8G2::u8g2_xor(uint8_t a) {
    uint8_t i;
    display.drawStr( 0, 0, "XOR");
    display.setFontMode(1);
    display.setDrawColor(2);
    for( i = 0; i < 5; i++)
    {
        display.drawBox(10+i*16, 18 + (i&1)*4, 21,31);
    }
    display.drawStr( 5+a, 19, "XOR XOR XOR XOR");
    display.setDrawColor(0);
    display.drawStr( 5+a, 29, "CLR CLR CLR CLR");
    display.setDrawColor(1);
    display.drawStr( 5+a, 39, "SET SET SET SET");
    display.setFontMode(0);

}



void displayU8G2::u8g2_bitmap_overlay(uint8_t a) {
    uint8_t frame_size = 28;

    display.drawStr(0, 0, "Bitmap overlay");

    display.drawStr(0, frame_size + 12, "Solid / transparent");
    display.setBitmapMode(false);
    display.drawFrame(0, 10, frame_size, frame_size);
    display.drawXBMP(2, 12, cross_width, cross_height, cross_bits);
    if(a & 4)
        display.drawXBMP(7, 17, cross_block_width, cross_block_height, cross_block_bits);

    display.setBitmapMode(true);
    display.drawFrame(frame_size + 5, 10, frame_size, frame_size);
    display.drawXBMP(frame_size + 7, 12, cross_width, cross_height, cross_bits);
    if(a & 4)
        display.drawXBMP(frame_size + 12, 17, cross_block_width, cross_block_height, cross_block_bits);
}

void displayU8G2::u8g2_bitmap_modes(uint8_t transparent) {
    const uint8_t frame_size = 24;

    display.drawBox(0, frame_size * 0.5, frame_size * 5, frame_size);
    display.drawStr(frame_size * 0.5, 50, "Black");
    display.drawStr(frame_size * 2, 50, "White");
    display.drawStr(frame_size * 3.5, 50, "XOR");

    if(!transparent) {
        display.setBitmapMode(false );
        display.drawStr(0, 0, "Solid bitmap");
    } else {
        display.setBitmapMode(true );
        display.drawStr(0, 0, "Transparent bitmap");
    }
    display.setDrawColor(0);// Black
    display.drawXBMP(frame_size * 0.5, 24, cross_width, cross_height, cross_bits);
    display.setDrawColor(1); // White
    display.drawXBMP(frame_size * 2, 24, cross_width, cross_height, cross_bits);
    display.setDrawColor(2); // XOR
    display.drawXBMP(frame_size * 3.5, 24, cross_width, cross_height, cross_bits);
}


void displayU8G2::draw() {
    displayU8G2::u8g2_prepare();
    switch(draw_state >> 3) {
        case 0: displayU8G2::u8g2_box_title(draw_state & 7); break;
        case 1: displayU8G2::u8g2_box_frame(draw_state & 7); break;
        case 2: displayU8G2::u8g2_disc_circle(draw_state & 7); break;
        case 3: displayU8G2::u8g2_r_frame(draw_state & 7); break;
        case 4: displayU8G2::u8g2_string(draw_state & 7); break;
        case 5: displayU8G2::u8g2_line(draw_state & 7); break;
        case 6: displayU8G2::u8g2_triangle(draw_state & 7); break;
        case 7: displayU8G2::u8g2_ascii_1(); break;
        case 8: displayU8G2::u8g2_ascii_2(); break;
        case 9: displayU8G2::u8g2_extra_page(draw_state & 7); break;
        case 10: displayU8G2::u8g2_xor(draw_state & 7); break;
        case 11: displayU8G2::u8g2_bitmap_modes(0); break;
        case 12: displayU8G2::u8g2_bitmap_modes(1); break;
        case 13: displayU8G2::u8g2_bitmap_overlay(draw_state & 7); break;
    }
    // increase the state
    draw_state++;
    if ( draw_state >= 14*8 )
        draw_state = 0;
}




