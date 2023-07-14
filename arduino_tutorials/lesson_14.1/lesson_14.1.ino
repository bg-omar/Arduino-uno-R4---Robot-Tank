/*
 keyestudio Mini Tank Robot v2.0
 lesson 14.1
 bluetooth test
 http://www.keyestudio.com
*/
char ble_val; //character variables, used to save the value of Bluetooth reception
void setup() {
  Serial.begin(9600);
}
void loop() {
  if(Serial.available() > 0)  //judge if there is data in buffer area
  { ble_val = Serial.read();  //read the data from serial buffer
    Serial.println(ble_val);  //print out
  }}//**************************************************************
