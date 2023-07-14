/*keyestudio Mini Tank Robot V2
lesson 3.2
photocell-analog output
http://www.keyestudio.com
*/
int analogInPin = A1;  // Analog input pin that the photocell is attached to
int analogOutPin = 10; // Analog output pin that the LED is attached to
int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
void setup() {
Serial.begin(9600);
 }
void loop() {
  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 1023, 0, 255);
  // change the analog out value:
  analogWrite(analogOutPin, outputValue);
  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
 Serial.println(sensorValue);  //serial port prints the value of photoresistor
delay(2);
}
//***************************************************************
