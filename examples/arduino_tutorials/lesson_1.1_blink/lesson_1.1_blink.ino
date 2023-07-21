 
/*
 keyestudio Mini Tank Robot V2
 lesson 1.1
 Blink
 http://www.keyestudio.com
*/
void setup()
 { 
    pinMode(2, OUTPUT);// initialize digital pin 2 as an output.
}
void loop() // the loop function runs over and over again forever

{
   digitalWrite(2, HIGH); // turn the LED on (HIGH is the voltage level)
   delay(1000); // wait for a second
   digitalWrite(2, LOW); // turn the LED off by making the voltage LOW
   delay(1000); // wait for a second
}
