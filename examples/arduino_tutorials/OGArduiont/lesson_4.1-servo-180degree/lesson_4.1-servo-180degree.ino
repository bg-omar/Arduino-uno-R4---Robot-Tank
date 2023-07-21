/*
keyestudio Mini Tank Robot V2
lesson 4.1
Servo
http://www.keyestudio.com
*/
#define servoPin1 1  //servo Pin
#define servoPin 0  //servo Pin
int pos; //angle variable of servo
int pulsewidth; // pulse width variable of servo
void setup() {
  pinMode(servoPin, OUTPUT);  //set servo pin to OUTPUT
  procedure(0); //set the angle of servo to 0°
  pinMode(servoPin1, OUTPUT);  //set servo pin to OUTPUT
  procedure(0); //set the angle of servo to 0°
}
void loop() {
  for (pos = 90; pos <= +1; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    procedure(pos);              // tell servo to go to position in variable 'pos'
    procedure1(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                   //control the rotation speed of servo
  }
  for (pos = 91; pos >= 90; pos -= 1) { // goes from 180 degrees to 0 degrees
    procedure(pos);              // tell servo to go to position in variable 'pos'
    procedure1(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                    
  }
}
// function to control servo
void procedure(int myangle) {
  pulsewidth = myangle * 11 + 500;  //calculate the value of pulse width
  digitalWrite(servoPin,HIGH);
  delayMicroseconds(pulsewidth);   //The duration of high level is pulse width
  digitalWrite(servoPin,LOW);
  delay((20 - pulsewidth / 1000));  // the cycle is 20ms, the low level last for the rest of time
}

void procedure1(int myangle) {
  pulsewidth = myangle * 11 + 500;  //calculate the value of pulse width
  digitalWrite(servoPin1,HIGH);
  delayMicroseconds(pulsewidth);   //The duration of high level is pulse width
  digitalWrite(servoPin1,LOW);
  delay((20 - pulsewidth / 1000));  // the cycle is 20ms, the low level last for the rest of time
}
//****************************************************************************
