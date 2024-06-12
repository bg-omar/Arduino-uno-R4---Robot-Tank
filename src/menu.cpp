//
// Created by mr on 6/12/2024.
//

#include "menu.h"
#include "menu_imgs.h"

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0); // [full framebuffer, size = 1024 bytes]


const int NUM_ITEMS = 8; // number of items in the list and also the number of screenshots and screenshots with QR codes (other screens)
const int MAX_ITEM_LENGTH = 20; // maximum characters for the item name

char menu_items [NUM_ITEMS] [MAX_ITEM_LENGTH] = {  // array with item names
		{ "3D Cube" },
		{ "Battery" },
		{ "Dashboard" },
		{ "Fireworks" },
		{ "GPS Speed" },
		{ "Big Knob" },
		{ "Park Sensor" },
		{ "Turbo Gauge" }
};
// note - when changing the order of items above, make sure the other arrays referencing bitmaps
// also have the same order, for example array "bitmap_icons" for icons, and other arrays for screenshots and QR codes

#define BUTTON_UP_PIN 12 // pin for UP button
#define BUTTON_SELECT_PIN 8 // pin for SELECT button
#define BUTTON_DOWN_PIN 4 // pin for DOWN button

#define DEMO_PIN 13 // pin for demo mode, use switch or wire to enable or disable demo mode, see more details below


int button_up_clicked = 0; // only perform action when button is clicked, and wait until another press
int button_select_clicked = 0; // same as above
int button_down_clicked = 0; // same as above

int item_selected = 0; // which item in the menu is selected

int item_sel_previous; // previous item - used in the menu screen to draw the item before the selected one
int item_sel_next; // next item - used in the menu screen to draw next item after the selected one

int current_screen = 0;   // 0 = menu, 1 = screenshot, 2 = qr

int demo_mode = 0; // when demo mode is set to 1, it automatically goes over all the screens, 0 = control menu with buttons
int demo_mode_state = 0; // demo mode state = which screen and menu item to display
int demo_mode_delay = 0; // demo mode delay = used to slow down the screen switching


void menu::setupMenu() {
	u8g2.setColorIndex(1);  // set the color to white
	u8g2.begin();
	u8g2.setBitmapMode(1);

	// define pins for buttons
	// INPUT_PULLUP means the button is HIGH when not pressed, and LOW when pressed
	// since it´s connected between some pin and GND
	pinMode(BUTTON_UP_PIN, INPUT_PULLUP); // up button
	pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP); // select button
	pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP); // down button

	pinMode(DEMO_PIN, INPUT_PULLUP);

}



void menu::loopMenu() {
	// when pin 13 is LOW (DEMO_PIN), enable demo mode
	// this could be done either by using a switch
	// or simply by connecting the wire between pin 13 and GND
	// (those pins are next to each other)
	if (digitalRead(DEMO_PIN) == LOW) {
		demo_mode = 1; // enable demo mode
	}
	else  {
		demo_mode = 0; // disable demo mode
	}


	if (demo_mode == 1) { // when demo mode is active, automatically switch between all the screens and menu items
		demo_mode_delay++; // increase demo mode delay
		if (demo_mode_delay > 15) { // after some time, switch to another screen - change this value to make it slower/faster
			demo_mode_delay = 0;
			demo_mode_state++; // increase counter
			if (demo_mode_state >= NUM_ITEMS*3) {demo_mode_state=0;} // jump back to the first screen
		}

		if (demo_mode_state % 3 == 0) {current_screen = 0; item_selected = demo_mode_state/3; } // menu screen
		else if (demo_mode_state % 3 == 1) {current_screen = 1; item_selected = demo_mode_state/3;} // screenshots screen
		else if (demo_mode_state % 3 == 2) {current_screen = 2; item_selected = demo_mode_state/3;} // qr codes screen

	} // end demo mode section


	if (current_screen == 0) { // MENU SCREEN

		// up and down buttons only work for the menu screen
		if ((digitalRead(BUTTON_UP_PIN) == LOW) && (button_up_clicked == 0)) { // up button clicked - jump to previous menu item
			item_selected = item_selected - 1; // select previous item
			button_up_clicked = 1; // set button to clicked to only perform the action once
			if (item_selected < 0) { // if first item was selected, jump to last item
				item_selected = NUM_ITEMS-1;
			}
		}
		else if ((digitalRead(BUTTON_DOWN_PIN) == LOW) && (button_down_clicked == 0)) { // down button clicked - jump to next menu item
			item_selected = item_selected + 1; // select next item
			button_down_clicked = 1; // set button to clicked to only perform the action once
			if (item_selected >= NUM_ITEMS) { // last item was selected, jump to first menu item
				item_selected = 0;
			}
		}

		if ((digitalRead(BUTTON_UP_PIN) == HIGH) && (button_up_clicked == 1)) { // unclick
			button_up_clicked = 0;
		}
		if ((digitalRead(BUTTON_DOWN_PIN) == HIGH) && (button_down_clicked == 1)) { // unclick
			button_down_clicked = 0;
		}

	}


	if ((digitalRead(BUTTON_SELECT_PIN) == LOW) && (button_select_clicked == 0)) { // select button clicked, jump between screens
		button_select_clicked = 1; // set button to clicked to only perform the action once
		if (current_screen == 0) {current_screen = 1;} // menu items screen --> screenshots screen
		else if (current_screen == 1) {current_screen = 2;} // screenshots screen --> qr codes screen
		else {current_screen = 0;} // qr codes screen --> menu items screen
	}
	if ((digitalRead(BUTTON_SELECT_PIN) == HIGH) && (button_select_clicked == 1)) { // unclick
		button_select_clicked = 0;
	}

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

		// draw upir logo
		u8g2.drawXBMP(128-16-4, 64-4, 16, 4, upir_logo);

	}
	else if (current_screen == 1) { // SCREENSHOTS SCREEN
		u8g2.drawXBMP( 0, 0, 128, 64, bitmap_screenshots[item_selected]); // draw screenshot
	}
	else if (current_screen == 2) { // QR SCREEN
		u8g2.drawXBMP( 0, 0, 128, 64, bitmap_qr_codes[item_selected]); // draw qr code screenshot
	}


	u8g2.sendBuffer(); // send buffer from RAM to display controller


}


