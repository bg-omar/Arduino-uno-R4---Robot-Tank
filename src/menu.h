//
// Created by mr on 6/12/2024.
//

#ifndef ARDUINO_R4_UNO_WALL_Z_MENU_H
#define ARDUINO_R4_UNO_WALL_Z_MENU_H

class menu {

public:
	static void setupMenu();

	static void loopMenu();

	static void up();

	static void down();

	static void select();

private:

// 'icon_3dcube', 16x16px
	constexpr static const unsigned char bitmap_icon_3dcube [] PROGMEM = {
			0x00, 0x00, 0x80, 0x01, 0xE0, 0x06, 0x98, 0x18, 0x86, 0x60, 0x8A, 0x50,
			0xA2, 0x45, 0x82, 0x40, 0xA2, 0x44, 0x82, 0x40, 0xA2, 0x45, 0x8A, 0x50,
			0x86, 0x60, 0x98, 0x18, 0xE0, 0x06, 0x80, 0x01, };
// 'icon_battery', 16x16px
	constexpr static const unsigned char bitmap_icon_battery [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x1F, 0x02, 0x20,
			0xDA, 0x66, 0xDA, 0x66, 0xDA, 0x66, 0x02, 0x20, 0xFC, 0x1F, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };
// 'icon_dashboard', 16x16px
	constexpr static const unsigned char bitmap_icon_dashboard [] PROGMEM = {
			0xE0, 0x07, 0x18, 0x18, 0x84, 0x24, 0x0A, 0x40, 0x12, 0x50, 0x21, 0x80,
			0xC1, 0x81, 0x45, 0xA2, 0x41, 0x82, 0x81, 0x81, 0x05, 0xA0, 0x02, 0x40,
			0xD2, 0x4B, 0xC4, 0x23, 0x18, 0x18, 0xE0, 0x07, };
// 'icon_fireworks', 16x16px
	constexpr static const unsigned char bitmap_icon_fireworks [] PROGMEM = {
			0x00, 0x00, 0x00, 0x10, 0x00, 0x29, 0x08, 0x10, 0x08, 0x00, 0x36, 0x00,
			0x08, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x63, 0x00, 0x00, 0x00, 0x08,
			0x20, 0x08, 0x50, 0x00, 0x20, 0x00, 0x00, 0x00, };
// 'icon_gps_speed', 16x16px
	constexpr static const unsigned char bitmap_icon_gps_speed [] PROGMEM = {
			0x00, 0x00, 0xC0, 0x0F, 0x00, 0x10, 0x80, 0x27, 0x00, 0x48, 0x00, 0x53,
			0x60, 0x54, 0xE0, 0x54, 0xE0, 0x51, 0xE0, 0x43, 0xE0, 0x03, 0x50, 0x00,
			0xF8, 0x00, 0x04, 0x01, 0xFE, 0x03, 0x00, 0x00, };
// 'icon_knob_over_oled', 16x16px
	constexpr static const unsigned char bitmap_icon_knob_over_oled [] PROGMEM = {
			0x00, 0x00, 0xF8, 0x0F, 0xC8, 0x0A, 0xD8, 0x0D, 0x88, 0x0A, 0xF8, 0x0F,
			0xC0, 0x01, 0x80, 0x00, 0x00, 0x00, 0x90, 0x04, 0x92, 0x24, 0x04, 0x10,
			0x00, 0x80, 0x01, 0x40, 0x00, 0x00, 0x00, 0x00, };
// 'icon_parksensor', 16x16px
	constexpr static const unsigned char bitmap_icon_parksensor [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x44, 0x00, 0xA4, 0x00,
			0x9F, 0x00, 0x00, 0x81, 0x30, 0xA1, 0x48, 0xA9, 0x4B, 0xA9, 0x30, 0xA0,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };
// 'icon_turbo', 16x16px
	constexpr static const unsigned char bitmap_icon_turbo [] PROGMEM = {
			0x00, 0x70, 0xE0, 0x8F, 0x18, 0x80, 0x04, 0x80, 0x02, 0x80, 0xC2, 0x8F,
			0x21, 0x72, 0x51, 0x05, 0x91, 0x44, 0x51, 0x45, 0x21, 0x42, 0xC2, 0x21,
			0x02, 0x20, 0x04, 0x10, 0x18, 0x0C, 0xE0, 0x03, };

	// 'sentiment_very_dissatisfied', 16x16px
	constexpr static const unsigned char bitmap_icon_sentiment_very_dissatisfied [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0xe0, 0x07, 0x10, 0x08, 0x08, 0x10, 0x04, 0x20, 0x64, 0x26, 0x04, 0x20,
			0x04, 0x20, 0x84, 0x21, 0xe4, 0x27, 0x08, 0x10, 0x10, 0x08, 0xe0, 0x07, 0x00, 0x00, 0x00, 0x00
	};
// 'sentiment_neutral', 16x16px
	constexpr static const unsigned char bitmap_icon_sentiment_neutral [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0xe0, 0x07, 0x10, 0x08, 0x08, 0x10, 0x04, 0x20, 0x64, 0x26, 0x04, 0x20,
			0x04, 0x20, 0xc4, 0x23, 0x04, 0x20, 0x08, 0x10, 0x10, 0x08, 0xe0, 0x07, 0x00, 0x00, 0x00, 0x00
	};
// 'sentiment_dissatisfied', 16x16px
	constexpr static const unsigned char bitmap_icon_sentiment_dissatisfied [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0xe0, 0x07, 0x10, 0x08, 0x08, 0x10, 0x04, 0x20, 0x64, 0x26, 0x04, 0x20,
			0x04, 0x20, 0xc4, 0x23, 0x64, 0x26, 0x08, 0x10, 0x10, 0x08, 0xe0, 0x07, 0x00, 0x00, 0x00, 0x00
	};
// 'mood', 16x16px
	constexpr static const unsigned char bitmap_icon_mood [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0xe0, 0x07, 0x10, 0x08, 0x08, 0x10, 0x04, 0x20, 0x64, 0x26, 0x04, 0x20,
			0x04, 0x20, 0xe4, 0x27, 0xc4, 0x23, 0x08, 0x10, 0x10, 0x08, 0xe0, 0x07, 0x00, 0x00, 0x00, 0x00
	};
// 'sentiment_satisfied', 16x16px
	constexpr static const unsigned char bitmap_icon_sentiment_satisfied [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0xe0, 0x07, 0x10, 0x08, 0x08, 0x10, 0x04, 0x20, 0x64, 0x26, 0x04, 0x20,
			0x04, 0x20, 0x24, 0x24, 0xc4, 0x27, 0x08, 0x10, 0x10, 0x08, 0xe0, 0x07, 0x00, 0x00, 0x00, 0x00
	};
// 'timer', 16x16px
	constexpr static const unsigned char bitmap_icon_timer [] PROGMEM = {
			0x00, 0x00, 0xc0, 0x03, 0x00, 0x00, 0xc0, 0x03, 0x60, 0x1e, 0x10, 0x18, 0x88, 0x11, 0x88, 0x11,
			0x8c, 0x31, 0x08, 0x10, 0x08, 0x10, 0x08, 0x10, 0x30, 0x0c, 0xe0, 0x07, 0x00, 0x00, 0x00, 0x00
	};
// 'tune', 16x16px
	constexpr static const unsigned char bitmap_icon_tune [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xfc, 0x3d, 0x00, 0x04, 0x20, 0x00, 0x38, 0x1f,
			0x38, 0x1f, 0x20, 0x00, 0x00, 0x01, 0x3c, 0x3f, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
// 'remote_gen', 16x16px
	constexpr static const unsigned char bitmap_icon_remote_gen [] PROGMEM = {
			0x00, 0x00, 0xf0, 0x0f, 0x10, 0x08, 0x90, 0x09, 0x50, 0x0a, 0xd0, 0x0b, 0x90, 0x09, 0x10, 0x08,
			0x50, 0x0a, 0x10, 0x08, 0x50, 0x0a, 0x10, 0x08, 0x50, 0x0a, 0x10, 0x08, 0xf0, 0x0f, 0x00, 0x00
	};
// 'stadia_controller', 16x16px
	constexpr static const unsigned char bitmap_icon_stadia_controller [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x0f, 0x18, 0x18, 0x08, 0x14, 0x6c, 0x38, 0x04, 0x20,
			0x04, 0x20, 0xc4, 0x23, 0x24, 0x64, 0x34, 0x2c, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
// 'linked_camera', 16x16px
	constexpr static const unsigned char bitmap_icon_linked_camera [] PROGMEM = {
			0x00, 0x00, 0x00, 0x18, 0xc0, 0x21, 0xe0, 0x29, 0x3c, 0x10, 0x04, 0x00, 0x84, 0x21, 0x44, 0x22,
			0x24, 0x24, 0x64, 0x26, 0xc4, 0x23, 0x04, 0x20, 0xfc, 0x3f, 0xf8, 0x1f, 0x00, 0x00, 0x00, 0x00
	};
// '360', 16x16px
	constexpr static const unsigned char bitmap_icon_360 [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x0f, 0x1c, 0x38, 0x04, 0x20,
			0x44, 0x20, 0xdc, 0x38, 0xf0, 0x08, 0xc0, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
// 'speaker_phone', 16x16px
	constexpr static const unsigned char bitmap_icon_speaker_phone [] PROGMEM = {
			0x00, 0x00, 0xe0, 0x07, 0x30, 0x0c, 0x80, 0x01, 0x60, 0x06, 0x00, 0x00, 0xc0, 0x03, 0xe0, 0x07,
			0x20, 0x04, 0x20, 0x04, 0x20, 0x04, 0x20, 0x04, 0x20, 0x04, 0xe0, 0x07, 0x00, 0x00, 0x00, 0x00
	};
// 'piano', 16x16px
	constexpr static const unsigned char bitmap_icon_piano [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0xf8, 0x1f, 0xfc, 0x3f, 0x6c, 0x36, 0x6c, 0x36, 0x6c, 0x36, 0x6c, 0x36,
			0x6c, 0x36, 0x6c, 0x36, 0x0c, 0x30, 0x0c, 0x30, 0xfc, 0x3f, 0xf8, 0x1f, 0x00, 0x00, 0x00, 0x00
	};
// 'health_metrics', 16x16px
	constexpr static const unsigned char bitmap_icon_health_metrics [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0xe0, 0x07, 0x20, 0x04, 0x20, 0x04, 0x3c, 0x3c, 0x04, 0x23, 0x3c, 0x3f,
			0xfc, 0x3c, 0xc4, 0x20, 0x3c, 0x3c, 0x20, 0x04, 0x20, 0x04, 0xe0, 0x07, 0x00, 0x00, 0x00, 0x00
	};
// 'piano_off', 16x16px
	constexpr static const unsigned char bitmap_icon_piano_off [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0xe0, 0x1f, 0xe4, 0x3f, 0x4c, 0x36, 0x1c, 0x36, 0x2c, 0x36, 0x6c, 0x36,
			0xec, 0x34, 0x6c, 0x31, 0x0c, 0x22, 0x0c, 0x04, 0xfc, 0x0f, 0xf8, 0x1f, 0x00, 0x00, 0x00, 0x00
	};
// 'developer_board_off', 16x16px
	constexpr static const unsigned char bitmap_icon_developer_board_off [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0xe2, 0x0f, 0xc4, 0x1f, 0x0c, 0x10, 0x14, 0x33, 0x34, 0x10, 0x74, 0x32,
			0x84, 0x30, 0x74, 0x11, 0x74, 0x33, 0x04, 0x04, 0xfc, 0x0f, 0xf8, 0x1f, 0x00, 0x20, 0x00, 0x00
	};
// 'monitor', 16x16px
	constexpr static const unsigned char bitmap_icon_monitor [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0xf8, 0x1f, 0xfc, 0x3f, 0x04, 0x20, 0x04, 0x20, 0x04, 0x20, 0x04, 0x20,
			0x04, 0x20, 0x04, 0x20, 0x04, 0x20, 0xfc, 0x3f, 0xe0, 0x07, 0xe0, 0x07, 0x00, 0x00, 0x00, 0x00
	};
// 'desktop_access_disabled', 16x16px
	constexpr static const unsigned char bitmap_icon_desktop_access_disabled [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0xe2, 0x1f, 0xc4, 0x3f, 0x0c, 0x20, 0x14, 0x20, 0x24, 0x20, 0x44, 0x20,
			0x84, 0x20, 0x04, 0x21, 0x04, 0x22, 0xfc, 0x27, 0x80, 0x09, 0xc0, 0x13, 0x00, 0x20, 0x00, 0x00
	};
// 'restart_alt', 16x16px
	constexpr static const unsigned char bitmap_icon_restart_alt [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0xc0, 0x03, 0x80, 0x06, 0x10, 0x08, 0x18, 0x18,
			0x08, 0x10, 0x08, 0x10, 0x10, 0x08, 0x30, 0x0c, 0x60, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
// 'settings', 16x16px
	constexpr static const unsigned char bitmap_icon_settings [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0xc0, 0x03, 0x78, 0x1e, 0x0c, 0x30, 0xc8, 0x13, 0xc8, 0x13,
			0xc8, 0x13, 0xc8, 0x13, 0x0c, 0x30, 0x78, 0x1e, 0xc0, 0x03, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00
	};
// 'no_sim', 16x16px
	constexpr static const unsigned char bitmap_icon_no_sim [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0xc2, 0x1f, 0x24, 0x10, 0x08, 0x10, 0x18, 0x10, 0x38, 0x10, 0x48, 0x10,
			0x88, 0x10, 0x08, 0x11, 0x08, 0x02, 0x08, 0x04, 0x08, 0x08, 0xf8, 0x1f, 0x00, 0x20, 0x00, 0x00
	};
// 'storage', 16x16px
	constexpr static const unsigned char bitmap_icon_storage [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x3f, 0xf4, 0x3f, 0xfc, 0x3f, 0x00, 0x00, 0xf4, 0x3f,
			0xf4, 0x3f, 0x00, 0x00, 0xfc, 0x3f, 0xf4, 0x3f, 0xfc, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
// 'sim_card_download', 16x16px
	constexpr static const unsigned char bitmap_icon_sim_card_download [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0xc0, 0x1f, 0x20, 0x10, 0x10, 0x10, 0x08, 0x10, 0x08, 0x10, 0x88, 0x11,
			0xc8, 0x13, 0xc8, 0x13, 0x88, 0x11, 0x08, 0x10, 0x08, 0x10, 0xf8, 0x1f, 0x00, 0x00, 0x00, 0x00
	};
// 'sd_card', 16x16px
	constexpr static const unsigned char bitmap_icon_sd_card [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0xc0, 0x1f, 0x20, 0x10, 0x10, 0x10, 0x48, 0x1d, 0x48, 0x1d, 0x08, 0x10,
			0x08, 0x10, 0x08, 0x10, 0x08, 0x10, 0x08, 0x10, 0x08, 0x10, 0xf8, 0x1f, 0x00, 0x00, 0x00, 0x00
	};
// 'sd_card_alert', 16x16px
	constexpr static const unsigned char bitmap_icon_sd_card_alert [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0xc0, 0x1f, 0x20, 0x10, 0x10, 0x10, 0x08, 0x10, 0x88, 0x11, 0x88, 0x11,
			0x08, 0x10, 0x08, 0x10, 0x08, 0x10, 0x08, 0x10, 0x08, 0x10, 0xf8, 0x1f, 0x00, 0x00, 0x00, 0x00
	};
// 'joystick', 16x16px
	constexpr static const unsigned char bitmap_icon_joystick [] PROGMEM = {
			0x00, 0x00, 0x80, 0x01, 0x40, 0x02, 0x40, 0x02, 0x80, 0x01, 0x80, 0x01, 0xc0, 0x07, 0xb0, 0x0d,
			0x98, 0x19, 0x3c, 0x3c, 0x6c, 0x36, 0x98, 0x19, 0x70, 0x0e, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00
	};
// 'volume_off (1)', 16x16px
	constexpr static const unsigned char bitmap_icon_volume_off__1_ [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x06, 0x88, 0x08, 0x90, 0x10, 0x38, 0x12, 0x5c, 0x36,
			0x9c, 0x10, 0xf8, 0x11, 0xc0, 0x12, 0x80, 0x04, 0x00, 0x0e, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00
	};
// 'volume_down', 16x16px
	constexpr static const unsigned char bitmap_icon_volume_down [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x03, 0xf0, 0x0b, 0x70, 0x0b,
			0x70, 0x0b, 0xf0, 0x0b, 0x80, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
// 'volume_off', 16x16px
	constexpr static const unsigned char bitmap_icon_volume_off [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x06, 0x88, 0x08, 0x90, 0x10, 0x38, 0x12, 0x5c, 0x36,
			0x9c, 0x10, 0xf8, 0x11, 0xc0, 0x12, 0x80, 0x04, 0x00, 0x0e, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00
	};
// 'volume_up', 16x16px
	constexpr static const unsigned char bitmap_icon_volume_up [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x80, 0x08, 0xc0, 0x10, 0xf8, 0x12, 0x9c, 0x36,
			0x9c, 0x36, 0xf8, 0x12, 0xc0, 0x10, 0x80, 0x08, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
// 'developer_board', 16x16px
	constexpr static const unsigned char bitmap_icon_developer_board [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0xf8, 0x0f, 0xfc, 0x1f, 0x04, 0x10, 0x74, 0x33, 0x74, 0x10, 0x74, 0x33,
			0x04, 0x33, 0x74, 0x13, 0x74, 0x33, 0x04, 0x10, 0xfc, 0x1f, 0xf8, 0x0f, 0x00, 0x00, 0x00, 0x00
	};
// 'memory', 16x16px
	constexpr static const unsigned char bitmap_icon_memory [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x40, 0x02, 0x40, 0x02, 0xf0, 0x0f, 0x10, 0x08, 0x9c, 0x39, 0xd0, 0x0b,
			0xd0, 0x0b, 0x9c, 0x39, 0x10, 0x08, 0xf0, 0x0f, 0x40, 0x02, 0x40, 0x02, 0x00, 0x00, 0x00, 0x00
	};
// 'mic', 16x16px
	constexpr static const unsigned char bitmap_icon_mic [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0xc0, 0x03, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02,
			0xd0, 0x0b, 0x10, 0x08, 0x60, 0x06, 0xc0, 0x03, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
// 'stylus_laser_pointer', 16x16px
	constexpr static const unsigned char bitmap_icon_stylus_laser_pointer [] PROGMEM = {
			0x00, 0x00, 0x00, 0x18, 0x00, 0x0e, 0x80, 0x03, 0xc0, 0x01, 0x70, 0x00, 0xf8, 0x0f, 0xf0, 0x1f,
			0x00, 0x0f, 0x80, 0x07, 0x00, 0x02, 0xf0, 0x00, 0xf0, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00
	};
// 'wb', 16x16px
	constexpr static const unsigned char bitmap_icon_wb [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x08, 0x10, 0x08, 0x10, 0x00, 0x00, 0xf8, 0x1f, 0x18, 0x18,
			0x18, 0x18, 0xf8, 0x1f, 0x00, 0x00, 0x08, 0x10, 0x08, 0x10, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00
	};
// 'explore_off', 16x16px
	constexpr static const unsigned char bitmap_icon_explore_off [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0xe0, 0x07, 0x04, 0x0c, 0x08, 0x10, 0x1c, 0x26, 0x24, 0x27, 0x44, 0x22,
			0xc4, 0x20, 0xe4, 0x21, 0x24, 0x22, 0x08, 0x04, 0x30, 0x0c, 0xe0, 0x17, 0x00, 0x00, 0x00, 0x00
	};
// 'explore', 16x16px
	constexpr static const unsigned char bitmap_icon_explore [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0xe0, 0x07, 0x30, 0x0c, 0x08, 0x10, 0x04, 0x36, 0x84, 0x23, 0x44, 0x22,
			0x44, 0x22, 0xe4, 0x21, 0x24, 0x20, 0x08, 0x10, 0x30, 0x0c, 0xe0, 0x07, 0x00, 0x00, 0x00, 0x00
	};
// 'check_box', 16x16px
	constexpr static const unsigned char bitmap_icon_check_box [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x1f, 0x08, 0x10, 0x08, 0x10, 0x08, 0x16, 0x08, 0x13,
			0xe8, 0x11, 0xc8, 0x10, 0x08, 0x10, 0x08, 0x10, 0xf8, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
// 'check_box_outline_blank', 16x16px
	constexpr static const unsigned char bitmap_icon_check_box_outline_blank [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x1f, 0x08, 0x10, 0x08, 0x10, 0x08, 0x10, 0x08, 0x10,
			0x08, 0x10, 0x08, 0x10, 0x08, 0x10, 0x08, 0x10, 0xf8, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
// 'toggle_off', 16x16px
	constexpr static const unsigned char bitmap_icon_toggle_off [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x1f, 0x0c, 0x30, 0x36, 0x60, 0x7a, 0x40,
			0x7a, 0x40, 0x36, 0x60, 0x0c, 0x30, 0xf8, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
// 'toggle_on', 16x16px
	constexpr static const unsigned char bitmap_icon_toggle_on [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x1f, 0x0c, 0x30, 0x06, 0x6c, 0x02, 0x5e,
			0x02, 0x5e, 0x06, 0x6c, 0x0c, 0x30, 0xf8, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
// 'bluetooth_connected', 16x16px
	constexpr static const unsigned char bitmap_icon_bluetooth_connected [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x80, 0x03, 0x90, 0x06, 0xa0, 0x04, 0xc0, 0x03, 0x88, 0x11,
			0x88, 0x11, 0xc0, 0x03, 0xa0, 0x04, 0x90, 0x06, 0x80, 0x03, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00
	};
// 'bluetooth_disabled', 16x16px
	constexpr static const unsigned char bitmap_icon_bluetooth_disabled [] PROGMEM = {
			0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x8c, 0x03, 0x98, 0x06, 0x90, 0x04, 0x20, 0x02, 0x40, 0x00,
			0x80, 0x00, 0xc0, 0x01, 0xa0, 0x02, 0x90, 0x06, 0x80, 0x0b, 0x80, 0x11, 0x00, 0x00, 0x00, 0x00
	};

// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 2064)
	const int bitmap_icon_allArray_LEN = 43;
	constexpr static const unsigned char* bitmap_iconS[43] = {
			bitmap_icon_360,
			bitmap_icon_bluetooth_connected,
			bitmap_icon_bluetooth_disabled,
			bitmap_icon_check_box,
			bitmap_icon_check_box_outline_blank,
			bitmap_icon_desktop_access_disabled,
			bitmap_icon_developer_board,
			bitmap_icon_developer_board_off,
			bitmap_icon_explore,
			bitmap_icon_explore_off,
			bitmap_icon_health_metrics,
			bitmap_icon_joystick,
			bitmap_icon_linked_camera,
			bitmap_icon_memory,
			bitmap_icon_mic,
			bitmap_icon_monitor,
			bitmap_icon_mood,
			bitmap_icon_no_sim,
			bitmap_icon_piano,
			bitmap_icon_piano_off,
			bitmap_icon_remote_gen,
			bitmap_icon_restart_alt,
			bitmap_icon_sd_card,
			bitmap_icon_sd_card_alert,
			bitmap_icon_sentiment_dissatisfied,
			bitmap_icon_sentiment_neutral,
			bitmap_icon_sentiment_satisfied,
			bitmap_icon_sentiment_very_dissatisfied,
			bitmap_icon_settings,
			bitmap_icon_sim_card_download,
			bitmap_icon_speaker_phone,
			bitmap_icon_stadia_controller,
			bitmap_icon_storage,
			bitmap_icon_stylus_laser_pointer,
			bitmap_icon_timer,
			bitmap_icon_toggle_off,
			bitmap_icon_toggle_on,
			bitmap_icon_tune,
			bitmap_icon_volume_down,
			bitmap_icon_volume_off,
			bitmap_icon_volume_off__1_,
			bitmap_icon_volume_up,
			bitmap_icon_wb
	};


	// 'scrollbar_background', 8x64px
	constexpr static const unsigned char bitmap_scrollbar_background [] PROGMEM = {
			0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
			0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
			0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
			0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
			0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
			0x00, 0x40, 0x00, 0x00, };


// 'item_sel_outline', 128x21px
	constexpr static const unsigned char bitmap_item_sel_outline [] PROGMEM = {
			0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x02, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C,
			0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C,
			0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C,
			0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C,
			0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C,
			0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C,
			0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x0C, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0xF8, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03,
	};

	constexpr static const unsigned char siren_icon16x16[] =
			{
					0b10000000, 0b10000001, // #       #      #
					0b01000000, 0b10000010, //  #      #     #
					0b00100000, 0b10000100, //   #     #    #
					0b00010011, 0b11101000, //    #  ##### #
					0b00000111, 0b11110000, //      #######
					0b00001111, 0b11111000, //     #########
					0b00001111, 0b10111000, //     ##### ###
					0b00011111, 0b10011100, //    ######  ###
					0b00011111, 0b10011100, //    ######  ###
					0b00011111, 0b10001100, //    ######   ##
					0b00111111, 0b10001110, //   #######   ###
					0b00111111, 0b10001110, //   #######   ###
					0b01111111, 0b11111111, //  ###############
					0b01111111, 0b11111111, //  ###############
					0b01111111, 0b11111111, //  ###############
					0b00000000, 0b00000000, //
			};
	constexpr static const unsigned char warning_icon16x16[] =
			{
					0b00000000, 0b10000000, //         #
					0b00000001, 0b11000000, //        ###
					0b00000001, 0b11000000, //        ###
					0b00000011, 0b11100000, //       #####
					0b00000011, 0b01100000, //       ## ##
					0b00000111, 0b01110000, //      ### ###
					0b00000110, 0b00110000, //      ##   ##
					0b00001110, 0b10111000, //     ### # ###
					0b00001100, 0b10011000, //     ##  #  ##
					0b00011100, 0b10011100, //    ###  #  ###
					0b00011000, 0b10001100, //    ##   #   ##
					0b00111000, 0b00001110, //   ###       ###
					0b00110000, 0b10000110, //   ##    #    ##
					0b01111111, 0b11111111, //  ###############
					0b01111111, 0b11111111, //  ###############
					0b00000000, 0b00000000, //
			};
	constexpr static const unsigned char plus_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000001, 0b10000000, //        ##
					0b00000001, 0b10000000, //        ##
					0b00000001, 0b10000000, //        ##
					0b00000001, 0b10000000, //        ##
					0b00011111, 0b11111000, //    ##########
					0b00011111, 0b11111000, //    ##########
					0b00000001, 0b10000000, //        ##
					0b00000001, 0b10000000, //        ##
					0b00000001, 0b10000000, //        ##
					0b00000001, 0b10000000, //        ##
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
			};
	constexpr static const unsigned char minus_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00011111, 0b11111000, //    ##########
					0b00011111, 0b11111000, //    ##########
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
			};
	constexpr static const unsigned char mobile_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000000, 0b00110000, //           ##
					0b00000000, 0b00110000, //           ##
					0b00001111, 0b11111000, //     #########
					0b00001111, 0b11111000, //     #########
					0b00001100, 0b00011000, //     ##     ##
					0b00001100, 0b00011000, //     ##     ##
					0b00001100, 0b00011000, //     ##     ##
					0b00001100, 0b00011000, //     ##     ##
					0b00001111, 0b11111000, //     #########
					0b00001111, 0b11111000, //     #########
					0b00001111, 0b11111000, //     #########
					0b00001111, 0b11111000, //     #########
					0b00001111, 0b11111000, //     #########
					0b00001111, 0b11111000, //     #########
					0b00001111, 0b11111000, //     #########
			};
	constexpr static const unsigned char lock_closed_icon16x16[] =
			{
					0b00111111, 0b11111100, //   ############
					0b00111111, 0b11111100, //   ############
					0b00111000, 0b00011100, //   ###      ###
					0b00111000, 0b00011100, //   ###      ###
					0b00111000, 0b00011100, //   ###      ###
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b01111110, 0b01111110, //  ######  ######
					0b01111110, 0b01111110, //  ######  ######
					0b01111110, 0b01111110, //  ######  ######
					0b01111110, 0b01111110, //  ######  ######
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b00000000, 0b00000000, //
			};

	constexpr static const unsigned char lock_open_icon16x16[] =
			{
					0b00111111, 0b11111100, //   ############
					0b00111111, 0b11111100, //   ############
					0b00111000, 0b00011100, //   ###      ###
					0b00111000, 0b00000000, //   ###
					0b00111000, 0b00000000, //   ###
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b01111110, 0b01111110, //  ######  ######
					0b01111110, 0b01111110, //  ######  ######
					0b01111110, 0b01111110, //  ######  ######
					0b01111110, 0b01111110, //  ######  ######
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b00000000, 0b00000000, //
			};

	constexpr static const unsigned char person_icon16x16[] =
			{
					0b00000111, 0b11100000, //      ######
					0b00001111, 0b11110000, //     ########
					0b00001111, 0b11110000, //     ########
					0b00011111, 0b11111000, //    ##########
					0b00011111, 0b11111000, //    ##########
					0b00011111, 0b11111000, //    ##########
					0b00011111, 0b11111000, //    ##########
					0b00001111, 0b11110000, //     ########
					0b00001111, 0b11110000, //     ########
					0b00000111, 0b11100000, //      ######
					0b00000111, 0b11100000, //      ######
					0b00111111, 0b11111100, //   ############
					0b01111111, 0b11111110, //  ##############
					0b11111111, 0b11111111, // ################
					0b11111111, 0b11111111, // ################
					0b11111111, 0b11111111, // ################
			};
	constexpr static const unsigned char clock_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000011, 0b11100000, //       #####
					0b00000111, 0b11110000, //      #######
					0b00001100, 0b00011000, //     ##     ##
					0b00011000, 0b00001100, //    ##       ##
					0b00110000, 0b00000110, //   ##         ##
					0b00110000, 0b00000110, //   ##         ##
					0b00110000, 0b11111110, //   ##    #######
					0b00110000, 0b10000110, //   ##    #    ##
					0b00110000, 0b10000110, //   ##    #    ##
					0b00011000, 0b10001100, //    ##   #   ##
					0b00001100, 0b00011000, //     ##     ##
					0b00000111, 0b11110000, //      #######
					0b00000011, 0b11100000, //       #####
					0b00000000, 0b00000000, //
			};

	constexpr static const unsigned char timer_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000011, 0b11100000, //       #####
					0b00000111, 0b11110000, //      #######
					0b00001100, 0b10011000, //     ##  #  ##
					0b00011000, 0b00111100, //    ##     ####
					0b00110000, 0b01110110, //   ##     ### ##
					0b00110000, 0b11100110, //   ##    ###  ##
					0b00111001, 0b11001110, //   ###  ###  ###
					0b00110000, 0b10000110, //   ##    #    ##
					0b00110000, 0b00000110, //   ##         ##
					0b00011000, 0b00001100, //    ##       ##
					0b00001100, 0b10011000, //     ##  #  ##
					0b00000111, 0b11110000, //      #######
					0b00000011, 0b11100000, //       #####
					0b00000000, 0b00000000, //
			};

	constexpr static const unsigned char water_tap_icon16x16[] =
			{
					0b00000001, 0b10000000, //        ##
					0b00000111, 0b11100000, //      ######
					0b00000001, 0b10000000, //        ##
					0b00001111, 0b11110000, //     ########
					0b11111111, 0b11111110, // ###############
					0b11111111, 0b11111111, // ################
					0b11111111, 0b11111111, // ################
					0b11111111, 0b11111111, // ################
					0b00000000, 0b00001111, //             ####
					0b00000000, 0b00001111, //             ####
					0b00000000, 0b00000000, //
					0b00000000, 0b00001100, //             ##
					0b00000000, 0b00001100, //             ##
					0b00000000, 0b00000000, //
					0b00000000, 0b00001100, //             ##
					0b00000000, 0b00001100, //             ##
			};

	constexpr static const unsigned char humidity_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000001, 0b10000000, //        ##
					0b00000011, 0b11000000, //       ####
					0b00000111, 0b11100000, //      ######
					0b00001110, 0b01110000, //     ###  ###
					0b00001100, 0b00110000, //     ##    ##
					0b00011100, 0b00111000, //    ###    ###
					0b00011000, 0b00011000, //    ##      ##
					0b00111000, 0b00011100, //   ###      ###
					0b00111000, 0b00011100, //   ###      ###
					0b00111000, 0b00011100, //   ###      ###
					0b00011100, 0b00111000, //    ###    ###
					0b00011111, 0b11111000, //    ##########
					0b00001111, 0b11110000, //     ########
					0b00000011, 0b11000000, //       ####
					0b00000000, 0b00000000, //
			};

	constexpr static const unsigned char humidity2_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000001, 0b10000000, //        ##
					0b00000011, 0b11000000, //       ####
					0b00000111, 0b11100000, //      ######
					0b00001111, 0b11110000, //     ########
					0b00001111, 0b11110000, //     ########
					0b00011111, 0b11111000, //    ##########
					0b00011111, 0b11011000, //    ####### ##
					0b00111111, 0b10011100, //   #######  ###
					0b00111111, 0b10011100, //   #######  ###
					0b00111111, 0b00011100, //   ######   ###
					0b00011110, 0b00111000, //    ####   ###
					0b00011111, 0b11111000, //    ##########
					0b00001111, 0b11110000, //     ########
					0b00000011, 0b11000000, //       ####
					0b00000000, 0b00000000, //
			};

	constexpr static const unsigned char sun_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00100000, 0b10000010, //   #     #     #
					0b00010000, 0b10000100, //    #    #    #
					0b00001000, 0b00001000, //     #       #
					0b00000001, 0b11000000, //        ###
					0b00000111, 0b11110000, //      #######
					0b00000111, 0b11110000, //      #######
					0b00001111, 0b11111000, //     #########
					0b01101111, 0b11111011, //  ## ######### ##
					0b00001111, 0b11111000, //     #########
					0b00000111, 0b11110000, //      #######
					0b00000111, 0b11110000, //      #######
					0b00010001, 0b11000100, //    #   ###   #
					0b00100000, 0b00000010, //   #           #
					0b01000000, 0b10000001, //  #      #      #
					0b00000000, 0b10000000, //         #
			};

	constexpr static const unsigned char temperature_icon16x16[] =
			{
					0b00000001, 0b11000000, //        ###
					0b00000011, 0b11100000, //       #####
					0b00000111, 0b00100000, //      ###  #
					0b00000111, 0b11100000, //      ######
					0b00000111, 0b00100000, //      ###  #
					0b00000111, 0b11100000, //      ######
					0b00000111, 0b00100000, //      ###  #
					0b00000111, 0b11100000, //      ######
					0b00000111, 0b00100000, //      ###  #
					0b00001111, 0b11110000, //     ########
					0b00011111, 0b11111000, //    ##########
					0b00011111, 0b11111000, //    ##########
					0b00011111, 0b11111000, //    ##########
					0b00011111, 0b11111000, //    ##########
					0b00001111, 0b11110000, //     ########
					0b00000111, 0b11100000, //      ######
			};

	constexpr static const unsigned char heart_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00111100, 0b01111000, //   ####   ####
					0b01111110, 0b11111100, //  ###### ######
					0b11111111, 0b11111110, // ###############
					0b11111111, 0b11111110, // ###############
					0b11111111, 0b11111110, // ###############
					0b11111111, 0b11111110, // ###############
					0b01111111, 0b11111100, //  #############
					0b01111111, 0b11111100, //  #############
					0b00111111, 0b11111000, //   ###########
					0b00011111, 0b11110000, //    #########
					0b00001111, 0b11100000, //     #######
					0b00000111, 0b11000000, //      #####
					0b00000011, 0b10000000, //       ###
					0b00000001, 0b00000000, //    	  #
			};

	constexpr static const unsigned char nocon_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000011, 0b11100000, //       #####
					0b00001111, 0b11111000, //     #########
					0b00011111, 0b11111100, //    ###########
					0b00111110, 0b00111110, //   #####   #####
					0b00111000, 0b01111110, //   ###    ######
					0b01110000, 0b11111111, //  ###    ########
					0b01110001, 0b11110111, //  ###   ##### ###
					0b01110011, 0b11000111, //  ###  ####   ###
					0b01110111, 0b10000111, //  ### ####    ###
					0b00111111, 0b00001110, //   ######    ###
					0b00111110, 0b00011110, //   #####    ####
					0b00011111, 0b11111100, //    ###########
					0b00001111, 0b11111000, //     #########
					0b00000011, 0b11100000, //       #####
					0b00000000, 0b00000000, //
			};
	constexpr static const unsigned char tool_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000000, 0b01100000, //          ##
					0b00000000, 0b11100000, //         ###
					0b00000001, 0b11000000, //        ###
					0b00000001, 0b11000000, //        ###
					0b00000001, 0b11100110, //        ####  ##
					0b00000011, 0b11111110, //       #########
					0b00000111, 0b11111100, //      #########
					0b00001111, 0b11111000, //     #########
					0b00011111, 0b11000000, //    #######
					0b00111111, 0b10000000, //   #######
					0b01111111, 0b00000000, //  #######
					0b11111110, 0b00000000, // #######
					0b11111100, 0b00000000, // ######
					0b11111000, 0b00000000, // #####
					0b01110000, 0b00000000, //  ###
			};

	constexpr static const unsigned char plug_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000110, 0b01100000, //      ##  ##
					0b00000110, 0b01100000, //      ##  ##
					0b00000110, 0b01100000, //      ##  ##
					0b00000110, 0b01100000, //      ##  ##
					0b00111111, 0b11111100, //   ############
					0b00111111, 0b11111100, //   ############
					0b00111111, 0b11111100, //   ############
					0b00111111, 0b11111100, //   ############
					0b00011111, 0b11111000, //    ##########
					0b00001111, 0b11110000, //     ########
					0b00000111, 0b11100000, //      ######
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
			};
	constexpr static const unsigned char bluetooth_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000001, 0b10000000, //        ##
					0b00000001, 0b11000000, //        ###
					0b00000001, 0b01100000, //        # ##
					0b00001001, 0b00110000, //     #  #  ##
					0b00001101, 0b00110000, //     ## #  ##
					0b00000111, 0b01100000, //      ### ##
					0b00000011, 0b11000000, //       ####
					0b00000001, 0b10000000, //        ##
					0b00000011, 0b11000000, //       ####
					0b00000111, 0b01100000, //      ### ##
					0b00001101, 0b00110000, //     ## #  ##
					0b00001001, 0b00110000, //     #  #  ##
					0b00000001, 0b01100000, //        # ##
					0b00000001, 0b11000000, //        ###
					0b00000001, 0b10000000, //        ##
			};

	constexpr static const unsigned char bulb_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000011, 0b11100000, //       #####
					0b00000100, 0b00010000, //      #     #
					0b00001000, 0b00001000, //     #       #
					0b00010000, 0b00000100, //    #         #
					0b00010000, 0b00000100, //    #         #
					0b00010000, 0b00000100, //    #         #
					0b00010000, 0b00000100, //    #         #
					0b00010000, 0b00000100, //    #         #
					0b00001000, 0b00001000, //     #       #
					0b00000100, 0b00010000, //      #     #
					0b00000011, 0b11100000, //       #####
					0b00000010, 0b00100000, //       #   #
					0b00000011, 0b11100000, //       #####
					0b00000010, 0b00100000, //       #   #
					0b00000011, 0b11100000, //       #####
			};

	constexpr static const unsigned char bulb_on_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00100011, 0b11100010, //   #   #####   #
					0b00010100, 0b00010100, //    # #     # #
					0b00001000, 0b00001000, //     #       #
					0b00010000, 0b00000100, //    #         #
					0b00010000, 0b00000100, //    #         #
					0b00010000, 0b00000100, //    #         #
					0b00010000, 0b00000100, //    #         #
					0b00010000, 0b00000100, //    #         #
					0b00001000, 0b00001000, //     #       #
					0b00010100, 0b00010100, //    # #     # #
					0b00100011, 0b11100010, //   #   #####   #
					0b00000010, 0b00100000, //       #   #
					0b00000011, 0b11100000, //       #####
					0b00000010, 0b00100000, //       #   #
					0b00000011, 0b11100000, //       #####
			};

	constexpr static const unsigned char bulb_off_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000011, 0b11100000, //       #####
					0b00000111, 0b11110000, //      #######
					0b00001111, 0b11111000, //     #########
					0b00011111, 0b11111100, //    ###########
					0b00011111, 0b11111100, //    ###########
					0b00011111, 0b11111100, //    ###########
					0b00011111, 0b11111100, //    ###########
					0b00011111, 0b11111100, //    ###########
					0b00001111, 0b11111000, //     #########
					0b00000100, 0b00010000, //      #     #
					0b00000011, 0b11100000, //       #####
					0b00000010, 0b00100000, //       #   #
					0b00000011, 0b11100000, //       #####
					0b00000010, 0b00100000, //       #   #
					0b00000011, 0b11100000, //       #####
			};

	constexpr static const unsigned char bullet_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000011, 0b10000000, //       ###
					0b00001111, 0b11100000, //     #######
					0b00001111, 0b11100000, //     #######
					0b00011111, 0b11110000, //    #########
					0b00011111, 0b11110000, //    #########
					0b00011111, 0b11110000, //    #########
					0b00001111, 0b11100000, //     #######
					0b00001111, 0b11100000, //     #######
					0b00000011, 0b10000000, //       ###
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
			};

	constexpr static const unsigned char cancel_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00111000, 0b00001110, //   ###       ###
					0b00111100, 0b00011110, //   ####     ####
					0b00111110, 0b00111110, //   #####   #####
					0b00011111, 0b01111100, //    ##### #####
					0b00001111, 0b11111000, //     #########
					0b00000111, 0b11110000, //      #######
					0b00000011, 0b11100000, //       #####
					0b00000111, 0b11110000, //      #######
					0b00001111, 0b11111000, //     #########
					0b00011111, 0b01111100, //    ##### #####
					0b00111110, 0b00111110, //   #####   #####
					0b00111100, 0b00011110, //   ####     ####
					0b00111000, 0b00001110, //   ###       ###
					0b00000000, 0b00000000, //
			};

	constexpr static const unsigned char check_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000111, //              ###
					0b00000000, 0b00001111, //             ####
					0b00000000, 0b00011111, //            #####
					0b01110000, 0b00111110, //  ###      #####
					0b01111000, 0b01111100, //  ####    #####
					0b01111100, 0b11111000, //  #####  #####
					0b00011111, 0b11110000, //    #########
					0b00001111, 0b11100000, //     #######
					0b00000111, 0b11000000, //      #####
					0b00000011, 0b10000000, //       ###
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
			};

	constexpr static const unsigned char fillstate1_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
			};

	constexpr static const unsigned char fillstate2_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b00000000, 0b00000000, //
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
			};

	constexpr static const unsigned char fillstate3_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b00000000, 0b00000000, //
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b00000000, 0b00000000, //
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b00000000, 0b00000000, //
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
			};

	constexpr static const unsigned char fillstate4_icon16x16[] =
			{
					0b00000000, 0b00000000, //
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b00000000, 0b00000000, //
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b00000000, 0b00000000, //
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b00000000, 0b00000000, //
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
					0b01111111, 0b11111110, //  ##############
			};

	// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 384)
	constexpr static const unsigned char* bitmap_icons[28] = {
			bitmap_icon_monitor,
			bitmap_icon_monitor,
			bitmap_icon_monitor,
			bitmap_icon_monitor,
			bitmap_icon_monitor,
			bitmap_icon_storage,
			bitmap_icon_check_box_outline_blank,
			bitmap_icon_joystick,
			bitmap_icon_sd_card,
			bitmap_icon_360,
			bitmap_icon_explore,
			bitmap_icon_check_box_outline_blank,
			bitmap_icon_speaker_phone,
			bitmap_icon_remote_gen,
			bitmap_icon_health_metrics,
			bitmap_icon_developer_board,
			bitmap_icon_wb,
			bitmap_icon_mic,
			bitmap_icon_toggle_on,
			check_icon16x16,
			person_icon16x16,
			bitmap_icon_timer,
			bullet_icon16x16,
			bullet_icon16x16,
			bullet_icon16x16,
			bitmap_icon_wb,
			bitmap_icon_bluetooth_connected,
	};
};


#endif //ARDUINO_R4_UNO_WALL_Z_MENU_H
