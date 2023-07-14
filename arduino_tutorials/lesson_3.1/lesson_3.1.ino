/*
 keyestudio Mini Tank Robot V2
 lesson 3.1
 photocell
 http://www.keyestudio.com
*/
int sensorPin = A1;    // select the input pin for the photocell
int sensorValue = 0;  // variable to store the value coming from the sensor
void setup() {
Serial.begin(9600);
}
void loop() {
sensorValue = analogRead(sensorPin);  // read the value from the sensor:
Serial.println(sensorValue);  //Serial port prints the resistance value
delay(500);
}
//******************************************************
