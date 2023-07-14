/*
 keyestudio Mini Tank Robot v2.0
 lesson 7.1
 bluetooth 
http://www.keyestudio.com
*/

char ble_val; //character variable: save the value of Bluetooth reception

void setup() {
  Serial.begin(9600);
}
void loop() {
  if(Serial.available() > 0)  //make sure if there is data in serial buffer
  {
    ble_val = Serial.read();  //Read data from serial buffer
    Serial.println(ble_val);  //Print
  }}
//*******************************************
