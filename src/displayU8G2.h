//
// Created by mr on 11/13/2023.
//

#ifndef DISPLAYU8G2_H
#define DISPLAYU8G2_H

#include "config.h"
#include <U8g2lib.h>
#include <cstdint>

    /************************************************ Display u8g2  *************************************************/
    // section Display u8g2
    /***************************************************************************************************************/


    class displayU8G2 {

    public:
#define U8LOG_WIDTH 20
#define U8LOG_HEIGHT 6

        static void u8g2_prepare();
        static void u8g2_box_title(uint8_t a);
        static void u8g2_box_frame(uint8_t a) ;
        static void u8g2_disc_circle(uint8_t a) ;
        static void u8g2_r_frame(uint8_t a) ;
        static void u8g2_string(uint8_t a) ;
        static void u8g2_line(uint8_t a);
        static void u8g2_triangle(uint8_t a) ;
        static void u8g2_ascii_1() ;
        static void u8g2_ascii_2() ;
        static void u8g2_extra_page(uint8_t a);
        static void u8g2_xor(uint8_t a);
        static void u8g2_bitmap_overlay(uint8_t a) ;
        static void u8g2_bitmap_modes(uint8_t transparent) ;
        static void draw();

        static U8G2_SH1106_128X64_NONAME_F_HW_I2C display;

        static void U8G2setup();
#if LOG_DEBUG
        static void U8G2print(const char *log);
        static void U8G2println(const char * log);
        static void U8G2printEnd();
        static U8G2LOG u8g2log;
#endif
        static int t;



        static void animateScreen(uint8_t a);
        static void animate();
        static int incoming;
        static int environment;
        static int petStatus;
        static void happyFrame1();
        static void happyFrame2();
        static void happyFrame3();
        static void sadFrame1();
        static void sadFrame2();
        static void sadFrame3();

    private:

        static uint8_t draw_state;



    };






#endif //DISPLAYU8G2_H
