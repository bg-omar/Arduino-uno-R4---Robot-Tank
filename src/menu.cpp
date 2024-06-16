//
// Created by mr on 6/12/2024.
//
#include "U8g2lib.h"
#include "menu.h"
#include "main_ra.h"
#include "SD_card.h"

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0); // [full framebuffer, size = 1024 bytes]


const int NUM_ITEMS = 28; // number of items in the list and also the number of screenshots and screenshots with QR codes (other screens)
const int MAX_ITEM_LENGTH = 20; // maximum characters for the item name

char menu_items [NUM_ITEMS] [MAX_ITEM_LENGTH] = {  // array with item names
		{ "use_adafruit" },
		{ "use_u8g2" },
		{ "small" },
		{ "display_demo" },
		{ "use_round" },
		{ "use_menu" },
		{ "log_debug" },
		{ "use_ps4" },
		{ "use_sd_card" },
		{ "use_gyro" },
		{ "use_compass" },
		{ "use_barometer" },
		{ "use_distance" },
		{ "use_irremote" },
		{ "use_i2c_scanner" },
		{ "use_pwm_board" },
		{ "use_dot" },
		{ "use_mic" },
		{ "use_switch" },
		{ "use_analog" },
		{ "use_robot" },
		{ "use_timers" },
		{ "use_matrix" },
		{ "use_matrix_preview" },
		{ "read_esp32" },
		{ "use_lcd" },
		{ "use_hm_10_ble" }
};


// note - when changing the order of items above, make sure the other arrays referencing bitmaps
// also have the same order, for example array "bitmap_icons" for icons, and other arrays for screenshots and QR codes


int item_selected = 0; // which item in the menu is selected

int item_sel_previous; // previous item - used in the menu screen to draw the item before the selected one
int item_sel_next; // next item - used in the menu screen to draw next item after the selected one

int current_screen = 0;   // 0 = menu, 1 = screenshot, 2 = qr

int demo_mode_state = 0; // demo mode state = which screen and menu item to display
int demo_mode_delay = 0; // demo mode delay = used to slow down the screen switching


void menu::setupMenu() {
	u8g2.setColorIndex(1);  // set the color to white
	u8g2.begin();
	u8g2.setBitmapMode(1);
}



void menu::loopMenu() {
	if (main::display_demo) { // when demo mode is active, automatically switch between all the screens and menu items
		demo_mode_delay++; // increase demo mode delay
		if (demo_mode_delay > 15) { // after some time, switch to another screen - change this value to make it slower/faster
			demo_mode_delay = 0;
			demo_mode_state++; // increase counter
			if (demo_mode_state >= NUM_ITEMS*3) {demo_mode_state=0;} // jump back to the first screen
		}

		if (demo_mode_state % 3 == 0) {current_screen = 0; item_selected = demo_mode_state/3; } // menu screen
	} // end demo mode section



	// set correct values for the previous and next items
	item_sel_previous = item_selected - 1;
	if (item_sel_previous < 0) {item_sel_previous = NUM_ITEMS - 1;} // previous item would be below first = make it the last
	item_sel_next = item_selected + 1;
	if (item_sel_next >= NUM_ITEMS) {item_sel_next = 0;} // next item would be after last = make it the first



	u8g2.clearBuffer();  // clear buffer for storing display content in RAM

	if (current_screen == 0) { // MENU SCREEN

		// selected item background
		u8g2.drawXBMP(0, 22, 128, 21, bitmap_item_sel_outline);

		// draw previous item as icon + label
		u8g2.setFont(u8g_font_7x14);
		u8g2.drawStr(25, 15, menu_items[item_sel_previous]);
		u8g2.drawXBMP( 4, 2, 16, 16, bitmap_icons[item_sel_previous]);


		// draw selected item as icon + label in bold font
		u8g2.setFont(u8g_font_7x14B);
		u8g2.drawStr(25, 15+20+2, menu_items[item_selected]);
		u8g2.drawXBMP( 4, 24, 16, 16, bitmap_icons[item_selected]);

		// draw next item as icon + label
		u8g2.setFont(u8g_font_7x14);
		u8g2.drawStr(25, 15+20+20+2+2, menu_items[item_sel_next]);
		u8g2.drawXBMP( 4, 46, 16, 16, bitmap_icons[item_sel_next]);

		// draw scrollbar background
		u8g2.drawXBMP(128-8, 0, 8, 64, bitmap_scrollbar_background);

		// draw scrollbar handle
		u8g2.drawBox(125, 64/NUM_ITEMS * item_selected, 3, 64/NUM_ITEMS);

	}

	u8g2.sendBuffer(); // send buffer from RAM to display controller
}


void menu::down() {
	if (current_screen == 0) {
		item_selected = item_selected + 1; // select next item
		if (item_selected >= NUM_ITEMS) { // last item was selected, jump to first menu item
			item_selected = 0;
		}
	}
}

void menu::up() {
	if (current_screen == 0) {
		item_selected = item_selected - 1; // select previous item
		if (item_selected < 0) { // if first item was selected, jump to last item
			item_selected = NUM_ITEMS - 1;
		}
	}
}

void menu::select() {
	switch (item_selected) {
		case 0:
			main::use_adafruit = !main::use_adafruit;
			break;
		case 1:
			main::use_u8g2 = !main::use_u8g2;
			break;
		case 2:
			main::small = !main::small;
			break;
		case 3:
			main::display_demo = !main::display_demo;
			break;
		case 4:
			main::use_round = !main::use_round;
			break;
		case 5:
			main::use_menu = !main::use_menu;
			break;
		case 6:
			main::log_debug = !main::log_debug;
			break;
		case 7:
			main::use_ps4 = !main::use_ps4;
			break;
		case 8:
			main::use_sd_card = !main::use_sd_card;
			break;
		case 9:
			main::use_gyro = !main::use_gyro;
			break;
		case 10:
			main::use_compass = !main::use_compass;
			break;
		case 11:
			main::use_barometer = !main::use_barometer;
			break;
		case 12:
			main::use_distance = !main::use_distance;
			break;
		case 13:
			main::use_irremote = !main::use_irremote;
			break;
		case 14:
			main::use_i2c_scanner = !main::use_i2c_scanner;
			break;
		case 15:
			main::use_pwm_board = !main::use_pwm_board;
			break;
		case 16:
			main::use_dot = !main::use_dot;
			break;
		case 17:
			main::use_mic = !main::use_mic;
			break;
		case 18:
			main::use_switch = !main::use_switch;
			break;
		case 19:
			main::use_analog = !main::use_analog;
			break;
		case 20:
			main::use_robot = !main::use_robot;
			break;
		case 21:
			main::use_timers = !main::use_timers;
			break;
		case 22:
			main::use_matrix = !main::use_matrix;
			break;
		case 23:
			main::use_matrix_preview = !main::use_matrix_preview;
			break;
		case 24:
			main::read_esp32 = !main::read_esp32;
			break;
		case 25:
			main::use_lcd = !main::use_lcd;
			break;
		case 26:
			main::use_hm_10_ble = !main::use_hm_10_ble;
			break;
		default:
			break;
	}
	SD_card::configSaveSD();
}


