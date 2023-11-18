/*
UNO BLE -->     DC:54:75:C3:D9:ED   -
PC USB Dongel   00:1F:E2:C8:82:BA
ESP 1           66:CB:3E:E9:02:8A
ESP small cam   3C:E9:0E:88:65:16
PS4 Controller: A4:AE:11:E1:8B:B3 (SONYWA) GooglyEyes
PS5 Controller: 88:03:4C:B5:00:66
*/


/***************************************************************************************************************/
// section include
/***************************************************************************************************************/

#include "index.h"
#include "secrets.h"

#include "displayU8G2.h"
#include "pesto_matrix.h"
#include "follow_light.h"
#include "motor.h"

#include "main_ra.h"
#include "PS4.h"
#include "pwm_board.h"
#include "timers.h"
#include "compass.h"
#include "gyroscope.h"
#include "IRremote.h.h"


U8G2_SH1106_128X64_NONAME_1_HW_I2C displayU8G2::display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

/************************************************** the I2CScanner *********************************************/
// section I2CScanner
/***************************************************************************************************************/
#if USE_I2C_SCANNER
	void I2CScanner() {
		byte error, address;
		int nDevices;

        displayU8G2::display.println("I2C Scanning...");
        #if USE_ADAFRUIT
            display.display();
        #endif
		nDevices = 0;
		
		delay(200);
		for(address = 1; address < 127; address++ ) {
			Wire.beginTransmission(address);
			error = Wire.endTransmission();

			if (error == 0) {
				displayU8G2::display.print(" 0x");
				if (address<16) {
					displayU8G2::display.print("0");
				}
				displayU8G2::display.println(address, HEX);
                #if USE_ADAFRUIT
                    display.display();
                #endif
				nDevices++;
				delay(200);
			}
			else if (error==4) {
				displayU8G2::display.print("Unknow error at address 0x");
				if (address<16) {
					
					displayU8G2::display.print("0");
				}
				displayU8G2::display.println(address, HEX);
                #if USE_ADAFRUIT
                    display.display();
                #endif
            }
		}
		delay(20);
		displayU8G2::display.print(nDevices);
		delay(20);
		displayU8G2::display.println(" devices");
        #if USE_ADAFRUIT
            display.display();
        #endif
		delay(100);
		if (nDevices == 0) {
			displayU8G2::display.println("No I2C devices found");
            #if USE_ADAFRUIT
                display.display();
            #endif
		}
	}
#endif
/************************************************** the gyroscope **********************************************/
// section gyroRead
/***************************************************************************************************************/
#if USE_GYRO

#endif
/*************************************************** the Compass ***********************************************/
// section Compass
/***************************************************************************************************************/
#if USE_COMPASS

#endif
/********************************************** control ultrasonic sensor***************************************/
// section BaroMeter
/***************************************************************************************************************/
#if USE_BAROMETER
    void baroSetup() {
        display.clearDisplay();
        /* Initialise the sensor */
        if (!bme.begin(0x76)) {
            
            displayU8G2::display.print("BME280,not found!");
            delay(500);
        } else {
            
            displayU8G2::display.print("BME280 Found!     ");
            delay(500);
        }
    }

    void baroMeter() {
        display.clearDisplay();
        
        displayU8G2::display.print("Temp= ");
        displayU8G2::display.print(bme.readTemperature());
        displayU8G2::display.print("*C ");

        displayU8G2::display.print("P= ");
        displayU8G2::display.print(bme.readPressure() / 100.0F);
        displayU8G2::display.print("hPa");

        
        displayU8G2::display.print("Alt= ");
        displayU8G2::display.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
        displayU8G2::display.print("m ");

        displayU8G2::display.print("H= ");
        displayU8G2::display.print(bme.readHumidity());
        displayU8G2::display.print("%");
        delay(500);
    }
#endif
/********************************************** control ultrasonic sensor***************************************/
// section UltraSonic
/***************************************************************************************************************/

double checkDistance() {
    digitalWrite(Trig_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig_PIN, LOW);
    double checkDistance = pulseIn(Echo_PIN, HIGH) / 58.00;  //58.20, that is, 2*29.1=58.2
    delay(10);
    return checkDistance;
}



/********************************************** arbitrary sequence *********************************************/
// section Dance
/***************************************************************************************************************/
#if USE_ROBOT
	void dance() {
        int flag = 0;
		randomXY = random(1, 180);
		randomZ = random(1, 160);
		pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(randomXY));
		delay(500);
		pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(randomZ));
		delay(500);

		// One pixel, row by row
		randomXY = random(1, 180);
		randomZ = random(1, 160);
		pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(randomXY));
		delay(500);
		pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(randomZ));
		delay(500);


		randomXY = random(1, 180);
		randomZ = random(1, 160);
		pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(randomXY));
		delay(500);
		pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(randomZ));
		delay(500);
        flag = PS4::exitLoop();
	}
#endif
/********************************************** Obstacle Avoidance Function*************************************/
// section Avoid
/***************************************************************************************************************/
#if USE_ROBOT
	void avoid() {
        int flag = 0; ///the design that enter obstacle avoidance function
		while (flag == 0) {
            random2 = random(1, 100);
            distanceF = checkDistance();
            if (distanceF < 25) {
                analogWrite(LED_PIN, 255);
                Motor::Car_Stop(); /// robot stops
                pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(115));
                delay(10); ///delay in 200ms
                pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(90));
                delay(10); ///delay in 200ms
                analogWrite(LED_PIN, 0);
                pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(160)); /// look left
                for (int j = 1;
                     j <= 10; j = j + (1)) { ///  the data will be more accurate if sensor detect a few times.
                    distanceL = checkDistance();
                }
                delay(200);
                pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(20)); /// look right
                for (int k = 1; k <= 10; k = k + (1)) {
                    distanceR = checkDistance();
                }
                if (distanceL < 50 || distanceR < 50) {
                    if (distanceL > distanceR) {
                        pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(90));
                        Motor::Car_left();
                        delay(500); ///turn left 500ms
                        Motor::Car_front();
                    } else {
                        pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(90));
                        Motor::Car_right();
                        delay(500);
                        Motor::Car_front();
                    }
                } else {  /// not (distanceL < 50 || distanceR < 50)
                    if ((long) (random2) % (long) (2) == 0) ///when the random number is even
                    {
                        pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(90));
                        Motor::Car_left(); ///robot turns left
                        delay(500);
                        Motor::Car_front(); ///go forward
                    } else {
                        pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(90));
                        Motor::Car_right(); ///robot turns right
                        delay(500);
                        Motor::Car_front(); ///go forward
                    }
                }
            } else /// if (distanceF < 25) { If the front distance is greater than or equal, robot car will go forward
            {
                Motor::Car_front();
            }
            flag = PS4::exitLoop();
        }
	}
#endif

/************************************************ Display Adafruit  *************************************************/
// section Display Adafruit
/***************************************************************************************************************/

#if USE_ADAFRUIT
    void testdrawroundrect() {
        display.clearDisplay();
        display.drawRoundRect(0, 0, display.width()-2, display.height()-2,
                              display.height()/8, PIXEL_WHITE);
        #if USE_ADAFRUIT
    display.display();
#endif
        delay(200);
    }

    void testfillroundrect() {
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

    void testdrawchar() {
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

    void testdrawstyles() {
        display.clearDisplay();

        display.setTextSize(2);             // Normal 1:1 pixel scale
        display.setTextColor(PIXEL_WHITE);        // Draw white text
        display.setCursor(0,0);             // Start at top-left corner
        displayU8G2::display.println(F("Hellow, Pesto!"));

        display.setTextColor(PIXEL_BLACK, PIXEL_WHITE); // Draw 'inverse' text
        displayU8G2::display.println(1337);

        display.setTextSize(2);             // Draw 2X-scale text
        display.setTextColor(PIXEL_WHITE);
        displayU8G2::display.print(F("0x")); displayU8G2::display.println(0xDEADBEEF, HEX);

        #if USE_ADAFRUIT
    display.display();
#endif
    }

    void testscrolltext() {
        display.clearDisplay();

        display.setTextSize(2); // Draw 2X-scale text
        display.setTextColor(PIXEL_WHITE);
        display.setCursor(10, 0);
        displayU8G2::display.println(F("PESTO?"));
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

#endif


void displayLoop(){
    #if USE_ADAFRUIT
        switch (displaySwitch) {
            case 1:  testdrawroundrect();  break;
            case 2:  testfillroundrect();  break;
            case 3:  testdrawchar();       break;
            case 4:  testdrawstyles();     break;
            case 5:  testscrolltext();     break;
            default:  display.display();
        }
        displaySwitch == 5 ? displaySwitch = 1 : displaySwitch += 1;
    #endif
}

/***************************** IrReceiver **********************************/
// section Loop IrReceiver
/***************************************************************************/

#if USE_IRREMOTE

#endif







void defaultLCD(){
    displayU8G2::display.print("Hello Pesto!");
       //Set cursor to line 1
    #if USE_IRREMOTE
        //displayU8G2::display.print(previousIR, HEX);
    #endif
}

int16_t lastY;


void printLog(const char *text, int size = 1, int16_t x = 0, int16_t y = lastY){
#if USE_ADAFRUIT
    display.setTextWrap(false);
    display.setCursor(x,y);             // Start at top-left corner
    display.setTextSize(size);             // Normal 1:1 pixel scale
    display.setTextColor(PIXEL_WHITE);        // Draw white text
    displayU8G2::display.println(text);
    lastY = y + 8;
    #if USE_ADAFRUIT
    display.display();
#endif
#else
    U8G2_SH1106_128X64_NONAME_1_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
    display.drawStr(x, y, (const char *) text);
    display.drawFrame(0,0,display.getDisplayWidth(),display.getDisplayHeight() );
    lastY = y + 8;
#endif


}


unsigned char front[] =  {0x00,0x00,0x00,0x00,0x00,0x24,0x12,0x09,0x12,0x24,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char back[] =   {0x00,0x00,0x00,0x00,0x00,0x24,0x48,0x90,0x48,0x24,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char left[] =   {0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x28,0x10,0x44,0x28,0x10,0x44,0x28,0x10,0x00};
unsigned char right[] =  {0x00,0x10,0x28,0x44,0x10,0x28,0x44,0x10,0x28,0x44,0x00,0x00,0x00,0x00,0x00,0x00};

unsigned char clear[] =  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};


/********************************************** Setup booting the arduino **************************************/
// section Setup
/***************************************************************************************************************/
void setup(){
    Wire.begin();

    #if USE_ADAFRUIT
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
    #else
        U8G2_display::display.begin();
        U8G2_display::u8g2_prepare();
        U8G2_display::display.firstPage();
    #endif

    delay(1);
    Serial.begin(115200); // Initialize the hardware serial port for debugging
    printLog("Serial started");
    delay(1);
    Serial1.begin(115200);
    printLog("Serial1 started");
    delay(1);
    SERIAL_AT.begin(115200);
    printLog("Serial_AT started");
    delay(1);

    #if USE_MATRIX
        matrix.loadSequence(animation);
        matrix.begin();
        //matrix.autoscroll(300);
        matrix.play(true);
    #endif
        
    pinMode(Trig_PIN, OUTPUT);    /***** 6 ******/
    pinMode(Echo_PIN, INPUT);     /***** 7 ******/

    pinMode(R_ROT, OUTPUT);     /***** 13 ******/
    pinMode(R_PWM, OUTPUT);      /***** 11 ******/
    pinMode(L_ROT, OUTPUT);     /***** 12 ******/
    pinMode(L_PWM, OUTPUT);      /***** 3 ******/
    digitalWrite(R_ROT, HIGH);
    digitalWrite(L_ROT, LOW);

    pinMode(LED_PIN, OUTPUT);     /***** 2 ******/
    pinMode(MIC_PIN, INPUT);

    pinMode(DotClockPIN,OUTPUT);/***** 5 ******/
    pinMode(DotDataPIN,OUTPUT); /***** 4 ******/
	digitalWrite(DotClockPIN,LOW);
	digitalWrite(DotDataPIN,LOW);

    baseSound = map(analogRead(MIC_PIN), 0, 1023, 0, 255); /***** A3 ******/

	#if USE_DOT
        Pesto::matrix_display(clear);
        Pesto::pestoMatrix();
	#endif

	#if USE_I2C_SCANNER
		I2CScanner();
		delay(500);
	#endif

    #if USE_GYRO
        #if USE_ADAFRUIT
            display.clearDisplay();
        #endif
        
        gyroscope::gyroSetup();
    #endif

    #if USE_COMPASS
        #if USE_ADAFRUIT
            display.clearDisplay();
        #endif
        compass::compassSetup();
    #endif

    #if USE_BAROMETER
        #if USE_ADAFRUIT
        display.clearDisplay();
    #endif
        baroSetup();
    #endif

    #if USE_IRREMOTE
        // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
        IRremote::IrReceiver.begin(IR_Pin, ENABLE_LED_FEEDBACK);
        timers::timerButton = PSHOME;
        #if USE_ADAFRUIT
            display.clearDisplay();
        #endif
        
        displayU8G2::display.print("InfraRed remote");
	#endif

	#if USE_PWM
        pwm_board::pwm.begin();
		pwm_board::pwm.setPWMFreq(FREQUENCY);  // Analog servos run at ~50 Hz updates
		pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(90));
		pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(45));
		delay(500);
	#endif

    #if USE_TIMERS
        timers::initTimers();
    #endif
    #if USE_ADAFRUIT
        display.clearDisplay();
    #endif
}

/*********************************** Loop **********************************/
// section Loop
/***************************************************************************/
unsigned long t = 0;

void loop(){
    #if USE_U8G2
        #if DISPLAY_DEMO
            U8G2_display::display.firstPage();
            do {
                U8G2_display::draw();
            } while( U8G2_display::display.nextPage() );
        #endif
    #endif
    #if READ_ESP32
        // Read messages from Arduino R4 ESP32
        if (SERIAL_AT.available()) {
            displayU8G2::display.print("ESP32 says: ");
            while (SERIAL_AT.available()) {
                Serial.write(SERIAL_AT.read());
                displayU8G2::display.println(SERIAL_AT.read());
            }
        }
    #endif

    while (Serial1.available() > 0) {
        PS4::controller();
    }

    
    // React on messages from ESP32-CAM AI-Thinker
    while(Serial1.available()) {
        int c = Serial1.read();
        Serial.write(c);
    }

    #if USE_IRREMOTE
        irRemote();
	#endif

    //Motor::Car_Stop();
    distanceF = checkDistance();  /// assign the front distance detected by ultrasonic sensor to variable a
    if (distanceF < 35) {
        #if USE_PWM
        pwm_board::RGBled(230, 0, 0);
        #endif
        #if USE_DOT
            Pesto::pestoMatrix();
        #endif
        double deltime = distanceF*3;
        delay(deltime);
    } else {
        int micStatus = analogRead(MIC_PIN);
        int mic255 = map(micStatus, 0, 1023, 0, 255);

        if (mic255 > baseSound) {
	#if USE_PWM
            pwm_board::RGBled(mic255, 0, mic255);
	#endif
		} else {
		#if USE_PWM
            pwm_board::RGBled(0, mic255, 0);
		#endif
		}
        //defaultLCD();
		#if USE_PWM
        pwm_board::RGBled(0, 0, Follow_light::lightSensor());
		#endif
	}
#if USE_GYRO
    gyroscope::gyroDetectMovement();
#endif

#if USE_TIMERS
    timers::update();
#endif
}


