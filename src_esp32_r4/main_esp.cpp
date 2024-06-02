/*
 * C:\Users\mr\Desktop\KS0428 Mini Tank Robot V2.0\unor4wifi-update-windows>update.bat
Start flashing firmware
version of espflash is: v2.0.1
Chip type:         esp32s3 (revision v0.1)
Crystal frequency: 40MHz
Flash size:        8MB
Features:          WiFi, BLE
MAC address:       dc:54:75:c3:d9:ec

UNO BLE -->     DC:54:75:C3:D9:EC   -
PC USB Dongel   00:1F:E2:C8:82:BA
ESP 1           66:CB:3E:E9:02:8A
ESP small cam   3C:E9:0E:88:65:16
PS4 Controller: A4:AE:11:E1:8B:B3 (SONYWA) GooglyEyes
PS5 Controller: 88:03:4C:B5:00:66
*/

#include <ArduinoBLE.h>

BLEService carService("180A"); // create service: "Device Information"

// create direction control characteristic and allow remote device to read and write
BLEByteCharacteristic carControlCharacteristic("2A57", BLERead | BLEWrite); // 2A57 is "Digital Output"

#include <esp_uno_r4.h>
#include <Arduino.h>

#include "real_time.h"


// ===========================

unsigned long lastTimeStamp = 0;
const uint32_t message_interval = 5000;

const char* emojis[8] = {
  "🌟🌟🌟", "😎👌🔥", "✅", "🇺🇦🛡️🇺🇦🛡️🇺🇦",
  "🤓", "¯\\_(ツ)_/¯", "🚀🌘", "🤯"
};


/*
UNO BLE -->     DC:54:75:C3:D9:EC   -
PC USB Dongel   00:1F:E2:C8:82:BA
ESP 1           66:CB:3E:E9:02:8A
ESP             3c:e9:0e:89:80:84
ESP small cam   3C:E9:0E:88:65:16
PS4 Controller: A4:AE:11:E1:8B:B3 (SONYWA) GooglyEyes
PS5 Controller: 88:03:4C:B5:00:66
*/
/********************************************** Setup booting the arduino **************************************/
// section Defines
/***************************************************************************************************************/

/********************************************** Setup booting the arduino **************************************/
// section Includes
/***************************************************************************************************************/



/********************************************** Setup booting the arduino **************************************/
// section Functions
/***************************************************************************************************************/


int r = 255;
int g = 0;
int b = 0;

// Calculates the next value in a rainbow sequence
void nextRainbowColor() {
    if (r > 0 && b == 0) { r--;  g++; }
    if (g > 0 && r == 0) { g--;  b++; }
    if (b > 0 && g == 0) { r++;  b--; }
}


void onConnect() {
    Serial.println("Connected!");
}

void onDisConnect() {
    Serial.println("Disconnected!");
}

void send(char32_t texting) {
    Serial.println(texting);
}


/********************************************** Setup booting the arduino **************************************/
// section BLE
/***************************************************************************************************************/



/*
  Bluetooth controlled car (that's the eventual goal here)

  My code is shared under the MIT license.
    In a nutshell, use it for anything but you take full responsibility.

Starting with the built-in ArduinoBLE example "Peripheral-ButtonBLE"
Also see:
https://docs.arduino.cc/tutorials/nano-33-ble/bluetooth

  This example creates a Bluetooth® Low Energy peripheral with service that contains a
  characteristic to control an LED

  You can use a generic Bluetooth® Low Energy central app, like LightBlue (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.

  See: https://www.arduino.cc/reference/en/libraries/arduinoble/

  Random UUID Generator: https://www.uuidgenerator.net/version4
  example: ea943a1a-2206-4235-970f-ad8127fff9bb

  Characteristics can have a random/custom UUID, or they can use a pre-defined value from the BlueTooth Assigned Numbers list:
  https://btprodspecificationrefs.blob.core.windows.net/assigned-numbers/Assigned%20Number%20Types/Assigned_Numbers.pdf

Examples:
// Bluetooth® Low Energy Battery Level Characteristic
BLEUnsignedCharCharacteristic batteryLevelChar("2A19",  // standard 16-bit characteristic UUID (see assigned numbers document)
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

According to the BLE Assigned Numbers document:
  joystick is 0x03C3
  boolean is 0x2AE2

*/


void BLEsetup() {
	Serial.begin(9600);
	while (!Serial);

	pinMode(LED_BUILTIN, OUTPUT); // use the LED as an output

	// begin initialization
	if (!BLE.begin()) {
		Serial.println("starting Bluetooth® Low Energy module failed!");
		while (1) { // blink the built-in LED fast to indicate an issue
			digitalWrite(LED_BUILTIN, HIGH);
			delay(100);
			digitalWrite(LED_BUILTIN, LOW);
			delay(100);
		}
	}

	BLE.setLocalName("UnoR4 BLE Car");
	BLE.setAdvertisedService(carService);

	// add the characteristics to the service
	carService.addCharacteristic(carControlCharacteristic);

	// add the service
	BLE.addService(carService);

	carControlCharacteristic.writeValue(0);

	// start advertising
	BLE.advertise();

	Serial.println("Bluetooth® device active, waiting for connections...");
}

void BLEloop() {
	// listen for BLE peripherals to connect:
	BLEDevice controller = BLE.central();

	// if a central is connected to peripheral:
	if (controller) {
		Serial.print("Connected to controller: ");
		// print the controller's MAC address:
		Serial.println(controller.address());
		digitalWrite(LED_BUILTIN, HIGH);  // turn on the LED to indicate the connection

		// while the controller is still connected to peripheral:
		while (controller.connected()) {

			if (carControlCharacteristic.written()) {

				switch (carControlCharacteristic.value()) {
					case 01:
						Serial.println("LEFT");
						break;
					case 02:
						Serial.println("RIGHT");
						break;
					case 03:
						Serial.println("UP");
						break;
					case 04:
						Serial.println("DOWN");
						break;
					default:  // 0 or invalid control
						Serial.println("STOP");
						break;
				}
			}
		}

		// when the central disconnects, print it out:
		Serial.print(F("Disconnected from controller: "));
		Serial.println(controller.address());
		digitalWrite(LED_BUILTIN, LOW);         // when the central disconnects, turn off the LED

	}
}

/********************************************** Setup booting the arduino **************************************/
// section Setup
/***************************************************************************************************************/


void setup()
{
  // Initialize CDC Bridge and CMSIS-DAP
    esp_uno_r4_setup();
    // Initialize the Serial communication for debugging
    Serial.begin(9600);

	BLEsetup();
}

void loop() {
	BLEloop();

	const uint32_t now = millis();
	static uint32_t last_message = 0;
	if (now - last_message > message_interval) {
		last_message += message_interval;

		// Send a message to Renesas chip
		SERIAL_AT.printf(" --> ESP32-S3 --> Arduino --> Pesto ¯\\_(ツ)_/¯ %s\r\n", emojis[random(8)]);
	}
}

