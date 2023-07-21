/* keyestudio Mini Tank Robot V2
lesson 6.2
IRremote
http://www.keyestudio.com
*/ 
#include <IRremoteTank.h>
int RECV_PIN = A0;//define the pin of IR receiver as A0
int LED_PIN=10;//define the pin of LED
int a=0;

int ledPin    = 10;    //define the ledPin
int sensorPin = 8;   //define the sensorPin



IRrecv irrecv(RECV_PIN);
decode_results results;
void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); // Initialize the IR receiver 
  pinMode(LED_PIN,OUTPUT);//set the pin of LED to 4
  pinMode(ledPin, OUTPUT); 
  pinMode(sensorPin, INPUT);
  
}
void loop() {
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);//Wrap word in 16 HEX to output and receive code
    if(results.value==0xFF02FD &a==0) // according to the above key value, press “OK” on remote control , LED will be controlled
    {
    a=1;
    }
    else if(results.value==0xFF02FD &a==1) //press again
    {
    digitalWrite(LED_PIN,LOW);//LED will go off
    a=0;
    }
    irrecv.resume(); //receive the next value
  }
   if(a==1){
      if(digitalRead(sensorPin) == HIGH){ //if read sensor for high level
        digitalWrite(ledPin, HIGH);   //led on
        Serial.println("led on...\n");
      }
      if(digitalRead(sensorPin) == LOW){        
        digitalWrite(ledPin, LOW);   //led off
        Serial.println("...led off\n");
      }
 }

}
//*******************************************************
