/*
keyestudio Mini Tank Robot V2
lesson 4.1
Servo
http://www.keyestudio.com
*/
#define servoPin 9  //servo Pin
int pos; //angle variable of servo
int pulsewidth; // pulse width variable of servo
int trigPin = 5;    // Trigger
int echoPin = 4;    // Echo
long duration, cm;

void setup() {
    //Serial Port begin
  Serial.begin (9600);
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(servoPin, OUTPUT);  //set servo pin to OUTPUT
  procedure(0); //set the angle of servo to 0°
}
void loop() {
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    procedure(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                   //control the rotation speed of servo
      // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Read the signal from the sensor: a HIGH pulse whose
    // duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    duration = pulseIn(echoPin, HIGH);
    // Convert the time into a distance
    cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
    Serial.print(cm);
    Serial.print("cm");
    Serial.println();
    delay(25);
    if (cm>=2 && cm<=10)
      digitalWrite(10, HIGH);
      delay(100);
      digitalWrite(10, LOW);
      delay(100);
    }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    procedure(pos);              // tell servo to go to position in variable 'pos'
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
//****************************************************************************
