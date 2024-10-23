//
// Created by mr on 11/13/2023.
//

#include "PS4.h"

#include "Arduino.h"
#include <U8g2lib.h>
#include "motor.h"
#include "pwm_board.h"
#include "timers.h"

#include "dancing.h"
#include "avoid_objects.h"
#include "follow_light.h"
#include "main_ra.h"
#include "compass.h"
#include "gyroscope.h"
#include "barometer.h"
#include "menu.h"
#include "displayAdafruit.h"

#include <string>
#include <vector>
#include <cctype>
using namespace std;

int PS4::exitLoop() {
    if (Serial1.available()) {
        static char message[MAX_MESSAGE_LENGTH]; // Create char for serial1 message
        static unsigned int message_pos = 0;
        char inByte = Serial1.read();
        if (inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1)) { // Add the incoming byte to our message
            message[message_pos] = inByte;
            message_pos++;
        } else { // Full message received...
            message[message_pos] = '\0'; // Add null character to string to end string
            int PS4input = atoi(message);
            if (PS4input == PSHOME){
                return 1;
            }
        }
    }
    #if USE_IRREMOTE
        if (IrReceiver.decode()) {
                IRRawDataType ir_rec = IrReceiver.decodedIRData.decodedRawData;
                IrReceiver.resume();
                if (ir_rec == Rem_OK) {
                    flag = 1;
                }
            }
    #endif
    return 0;
}

void PS4::joystick(int Xinput, int Yinput) {
    int R_velocity, L_velocity;
    int L2, R2, LX, LY;
    // Safety for if only Yinput is received as Xinput
    if (Xinput >= 9000 && Xinput <= 9255 ||
        Xinput >= 5000 && Xinput <= 5255 ||
        Xinput >= 7000 && Xinput <= 7255
        ) Yinput = Xinput;
    //    Serial.print("Xinput: "); Serial.println(Xinput); Serial.print("Yinput: "); Serial.println(Yinput);

    //---------------------------------------------- RIGHT THUMBSTICK
    if ((USE_PWM_BOARD && !main::use_pwm_board) || main::use_pwm_board) {
		if (Xinput >= 8000 && Xinput <= 8255 || Yinput >= 9000 && Yinput <= 9255) {
			if (Xinput >= 8000 && Xinput <= 8128) {
				int xMapped = map(Xinput - 8000, 0, 128, 1, 10); //        Serial.println(xMapped);
				if (pwm_board::posXY < 170) pwm_board::posXY += xMapped;
			} else if (Xinput > 8128 && Xinput <= 8255) {
				int xMapped = map(Xinput - 8128, 0, 128, 1, 10); //        Serial.println(xMapped);
				if (pwm_board::posXY > 10) pwm_board::posXY -= xMapped;
			}
			if (Yinput >= 9000 && Yinput < 9128) {
				int yMapped = map(Yinput - 9000, 0, 128, 1, 10); //        Serial.println(yMapped);
				if (pwm_board::posZ < 150)pwm_board::posZ += yMapped;

			} else if (Yinput >= 9128 && Yinput <= 9255) {
				int yMapped = map(Yinput - 9128, 0, 128, 1, 10); //        Serial.println(yMapped);
				if (pwm_board::posZ > 5) pwm_board::posZ -= yMapped;

			}

			if (pwm_board::posXY < 0) pwm_board::posXY = 0;
			if (pwm_board::posXY > 180) pwm_board::posXY = 180;
			if (pwm_board::posZ < 0) pwm_board::posZ = 0;
			if (pwm_board::posZ > 150) pwm_board::posZ = 150;
			pwm_board::pwm.setPWM(PWM_1, 0, pwm_board::pulseWidth(pwm_board::posZ));
			pwm_board::pwm.setPWM(PWM_0, 0, pwm_board::pulseWidth(pwm_board::posXY));
		}
	}

    // Change L2/R2 Lx/Ly inputs back to -128 / +128 values
    if (Xinput >= 4000 && Xinput <= 4255 || Yinput >= 5000 && Yinput <= 5255) {
        L2 = Xinput - 4000;
        R2 = Yinput - 5000;
        L_velocity = R2 - L2;//        Serial.print("L2: "); Serial.print(L2);
        R_velocity = R2 - L2;//        Serial.print("------R2: "); Serial.println(R2);
    } else if (Xinput >= 6000 && Xinput <= 6255 || Yinput >= 7000 && Yinput <= 7255) {
        LX = Xinput - 6128;//        Serial.print("X: "); Serial.print(LX);
        LY = Yinput - 7128;//        Serial.print("------Y: "); Serial.println(LY);
        // Calculate motor speed for Left Thumb stick
        L_velocity = LY-LX;//        Serial.print("L: "); Serial.print(L_velocity);
        R_velocity = LY+LX;//        Serial.print("------R: "); Serial.println(R_velocity);
    }

    // Determine rotation for the motors and change neg to pos pwm value
    if (L_velocity < 0) { digitalWrite(L_ROT, HIGH); L_velocity *= -2; /*Serial.print("_");*/} else{  digitalWrite(L_ROT, LOW); L_velocity *= 2;}
    if (L_velocity > 255) { L_velocity = 255; } //    Serial.print(L_velocity);

    if (R_velocity < 0)  { digitalWrite(R_ROT, LOW); R_velocity *= -2; /*Serial.print("_");*/} else{  digitalWrite(R_ROT, HIGH); R_velocity *= 2;}
    if (R_velocity > 255) { R_velocity = 255; } //    Serial.println(R_velocity);

    if (Xinput > 4000 && Xinput < 6999) {
        analogWrite(L_PWM, L_velocity);
        analogWrite(R_PWM, R_velocity);
    } else {
            Motor::Car_Stop();
    }
};

size_t find_operator(const std::string& expression, size_t start_index = 0) {
    string supported_operators = "+-_";
    for (char ch : supported_operators) {
        size_t op_index = expression.find(ch, start_index);
        if (op_index != std::string::npos) return op_index;
    }
    return std::string::npos;
}

vector<int> extract_integers(const string& expression){
    vector<int> integers;
    size_t prev_search_end = 0;
    for (size_t i = 0; i < expression.length(); i++) {
        size_t op_index = find_operator(expression, prev_search_end);
        if (op_index != std::string::npos)  {
            // operator found - produce substring of term
            string substr = expression.substr(prev_search_end, op_index - prev_search_end);

            // if there's a non-number at the end, remove it, e.g. 3x --> 3
            if (!isdigit(substr.back())) {
                substr.pop_back();
            }
            integers.push_back(stoi(substr));
            prev_search_end = op_index + 1;
        }
    }
    {   // account for last remaining term
        string substr = expression.substr(prev_search_end);

        // if there's a non-number at the end, remove it, e.g. 3x --> 3
        if (!isdigit(substr.back())) {
            substr.pop_back();
        }
        integers.push_back(stoi(substr));
    }
    return integers;
}
bool is_digit(char c) { return c >= '0' && c <= '9';}
bool is_alpha(char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');}

bool contains_letters(const std::string& s) {
    for (char c : s) {
        if (is_alpha(c)) { return true; }
    }
    return false;
}

void PS4::controller() {
    while (Serial1.available() > 0) {
        static char message[MAX_MESSAGE_LENGTH]; // Create char for serial1 message
        static unsigned int message_pos = 0;

        char inByte = Serial1.read();
        if (inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1)) { // Add the incoming byte to our message
            message[message_pos] = inByte;
            message_pos++;
        } else { // Full message received...
            message[message_pos] = '\0'; // Add null character to string to end string

            // Use the message
            vector<int> PS4input;
            if (!contains_letters(message)){
                PS4input = extract_integers(message);
            } else {
                // message contains text
                PS4input = { 0, 0 };
            }
//            main::logln(message);

            if (PS4input[0] >= 4000) { //for double input X-Y
                PS4::joystick(PS4input[0], PS4input[1]);
            } else {
                switch (PS4input[0]) {
                    case SQUARE:
						//compass::displayCompass();
						delay(250);
						break;
                    case TRIANG:
						displayAdafruit::animateScreen();
						delay(250);
                        break;
                    case xCROSS:
						menu::select();
						delay(250);
						break;
                    case CIRCLE:
						displayAdafruit::petStatus = !displayAdafruit::petStatus;
                        break;


					case DPAD_U:
						if ((USE_PWM_BOARD && !main::use_pwm_board) || main::use_pwm_board) {
							menu::up();
							delay(250);
						}
						break;
					case DPAD_R:
						if ((USE_PWM_BOARD && !main::use_pwm_board) || main::use_pwm_board) {
							if (pwm_board::posXY > 10) pwm_board::posXY -= 10;
							Serial.println(pwm_board::posXY);
						}
						break;
					case DPAD_D:
						if ((USE_PWM_BOARD && !main::use_pwm_board) || main::use_pwm_board) {
							menu::down();
							delay(250);
						}
					break;
					case DPAD_L:
						if ((USE_PWM_BOARD && !main::use_pwm_board) || main::use_pwm_board) {
							if (pwm_board::posXY < 170) pwm_board::posXY += 10;
							Serial.println(pwm_board::posXY);
						}
					break;

                    case 3101:
                    case 3401:
                    case 3201:
                    case 3301:
                        //Motor::Car_Stop();
                        break;

                    case xSHARE: if (main::use_compass) compass::showCompass();break;
                    case OPTION: if (main::use_gyro)  gyroscope::gyroFunc();break;
                    case PSHOME: if (main::use_barometer) barometer::baroMeter();break;
                    case L1:
                        #if USE_TIMERS
                          timers::timerTwoActive = !timers::timerTwoActive;
                          timers::timerTreeActive = false;
                          timers::timerButton = L1;
                        #endif
                        delay(100);
                        break;
                    case TOUCHPD:
						if(main::use_barometer) {
							barometer::baroMeter();
						}
                    case R1:
                        #if USE_TIMERS
                          timers::timerTwoActive = !timers::timerTwoActive;
                          timers::timerTreeActive = false;
                          timers::timerButton = R1;
                        #endif
                        delay(100);
                        break;
                    case L3:
                        #if USE_ROBOT
                          avoid_objects::avoid(); break;
                        #endif
						#if USE_ROBOT
							//dancing::dance(); break;
						#endif
                    case R3:
                      #if USE_ROBOT
                          Follow_light::light_track(); break;
                      #endif
                        //                    CHARGE  3500
                        //                    XAUDIO  3600
                        //                    MIC     3700
                        //                    PS4_Battery        3900 + Battery

                    default:
                        break;
                }
            }
            message_pos = 0; //Reset next message
            delay(50);
        }

    }
}
