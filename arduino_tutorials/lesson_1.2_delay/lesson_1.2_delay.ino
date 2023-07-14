/*
 keyestudio Mini Tank Robot V2
 lesson 1.2
 delay
 http://www.keyestudio.com
*/
void setup() {  // initialize digital pin 10 as an output.
   pinMode(10, OUTPUT);
}
// the loop function runs over and over again forever
void loop() {
   digitalWrite(10, HIGH); // turn the LED on (HIGH is the voltage level)
   delay(100); // wait for 0.1 second
   digitalWrite(10, LOW); // turn the LED off by making the voltage LOW
   delay(100); // wait for 0.1 second
}
