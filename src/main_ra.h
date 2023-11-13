//
// Created by mr on 11/13/2023.
//

#ifndef MAIN_RA_H
#define MAIN_RA_H

/********************************************** Declare all the functions***************************************/
// section declaration
/***************************************************************************************************************/

void setup();
void loop();

#if USE_I2C_SCANNER
    void I2CScanner();
#endif

void compass();

double checkDistance();



#if USE_ROBOT
void dance();
void avoid();
void light_track();
#endif




#if USE_GYRO
void gyroCalibrate_sensor();
void gyroDetectMovement();
void gyroFunc();
float ax, ay, az, gx, gy, gz, baseAx, baseAy, baseAz, baseGx, baseGy, baseGz, temperature;
Adafruit_MPU6050 mpu; // Set the gyroscope
#endif

#if USE_COMPASS
void compass();
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
#endif


/***************************************************** the Sensor Assigns **************************************/
// section Sensor Assigns
/***************************************************************************************************************/

#if USE_LCD
LiquidCrystal_I2C lcd(0x27,16,2);
#endif

#if USE_ADAFRUIT
#if SMALL
        Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
    #else
        Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
    #endif
#endif



#if USE_BAROMETER
Adafruit_BME280 bme;
#endif



/********************************************** Make DotMatric Images*******************************************/
// section DotMatrix Images
/***************************************************************************************************************/

#if USE_ADAFRUIT
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
		 0b00000000, 0b00110000 };
#endif

#if USE_MATRIX
const uint32_t animation[][4] = {
            {
                    0x0,
                    0x200500,
                    0x20000000,
                    66
            },
            {
                    0x5,
                    0xa80500,
                    0x20000000,
                    66
            },
            {
                    0xd812,
                    0x40880500,
                    0x20000000,
                    66
            },
            {
                    0xd812,
                    0x40880500,
                    0x20000000,
                    66
            },
            {
                    0x5,
                    0xa80500,
                    0x20000000,
                    66
            },
            {
                    0x0,
                    0x200500,
                    0x20000000,
                    66
            },
            {
                    0x0,
                    0x200500,
                    0x20000000,
                    66
            }
    };
#endif


/*************************************************** the Global Variables **************************************/
// section Global Variables
/***************************************************************************************************************/

int previousXY, previousZ;
int displaySwitch = 1;

long random2, randomXY, randomZ;
double distanceF, distanceR, distanceL;



long baseSound;
int r,g,b;
int  outputValueR, outputValueL;  // inverse input
double calcValue;



#endif //MAIN_RA_H
