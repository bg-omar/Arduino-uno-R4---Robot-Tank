/*
 keyestudio Mini Tank Robot v2.0
 lesson 7.2
 Bluetooth 
 http://www.keyestudio.com
*/ 
int ledpin=11;
void setup()
{Serial.begin(9600);
 pinMode(ledpin,OUTPUT);
}
void loop()
{ int i;
  if (Serial.available())
  {i=Serial.read();
    Serial.println("DATA RECEIVED:");
    if(i=='1')
    { digitalWrite(ledpin,1);
      Serial.println("led on");
    }
    if(i=='0')
    { digitalWrite(ledpin,0);
      Serial.println("led off");
}}}//*******************************************
