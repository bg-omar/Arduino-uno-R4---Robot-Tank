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


#include <esp_uno_r4.h>
#include <Arduino.h>

#include "real_time.h"
#include <ArduinoBLE.h>

// Define the name of the Bluetooth peripheral
const char* peripheralName = "Wall-Z ESP32";

// Create a BLE Service
BLEService fileTransferService("19B10000-E8F2-537E-4F6C-D104768A1214");

// Create a BLE Characteristic for receiving data
const char* dataCharacteristicUUID = "19B10001-E8F2-537E-4F6C-D104768A1214";
BLECharacteristic dataCharacteristic(dataCharacteristicUUID, BLEWrite | BLERead, "");

// ===========================

unsigned long lastTimeStamp = 0;
const uint32_t message_interval = 5000;

const char* emojis[8] = {
  "🌟🌟🌟", "😎👌🔥", "✅", "🇺🇦🛡️🇺🇦🛡️🇺🇦",
  "🤓", "¯\\_(ツ)_/¯", "🚀🌘", "🤯"
};

void setup()
{
  // Initialize CDC Bridge and CMSIS-DAP
    esp_uno_r4_setup();
    // Initialize the Serial communication for debugging
    Serial.begin(9600);

    // Initialize the BLE library
    if (!BLE.begin()) {
        SERIAL_AT.print("Starting BLE failed!");
    }

    // Set the local name of the Bluetooth peripheral
    BLE.setLocalName(peripheralName);
    BLE.setAdvertisedService(fileTransferService);

    // Add the characteristics to the service
    fileTransferService.addCharacteristic(dataCharacteristic);

    // Start advertising
    BLE.advertise();

    SERIAL_AT.print("Bluetooth peripheral advertising...");
}

void loop()
{
    BLEDevice central = BLE.central();
    if (central) {
        SERIAL_AT.print("Connected to central: ");
        SERIAL_AT.println(central.address());
        char receivedData [200] ;

        while (central.connected()) {
            if (dataCharacteristic.written()) {
                strcpy (receivedData, (const char*) dataCharacteristic.value());
                // Process the received data (e.g., parse JSON)
                SERIAL_AT.print("Received data: ");
                SERIAL_AT.println(receivedData);
            }
        }
    }

  const uint32_t now = millis();
  static uint32_t last_message = 0;
  if (now - last_message > message_interval) {
    last_message += message_interval;

    // Send a message to Renesas chip
    SERIAL_AT.printf(" --> ESP32-S3 --> Arduino --> Pesto ¯\\_(ツ)_/¯ %s\r\n", emojis[random(8)]);
  }
}
