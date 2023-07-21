/*
keyestudio Mini Tank Robot V2
lesson 2.2
pwm-slow
http://www.keyestudio.com
*/
int ledPin = 10; // Define the LED pin at D10
int value;
void setup ()
{
pinMode (ledPin, OUTPUT); // initialize ledpin as an output.
}
void loop ()
{
for (value = 0; value <255; value = value + 1)
{
analogWrite (ledPin, value); // LED lights gradually light up
delay (30); // delay 30MS
}
for (value = 255; value> 0; value = value-1)
{
analogWrite (ledPin, value); // LED gradually goes out
delay (30); // delay 30MS
}}//**********************************************************
