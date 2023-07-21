#include <TimerEvent.h>

#include <EveryTimer.h>
#include <OneShotTimer.h>

/*
 keyestudio Mini Tank Robot v2.0
 lesson 10
 Light-following tank
 http://www.keyestudio.com
*/ 

const unsigned int timerOnePeriod = 1000;
const unsigned int timerTwoPeriod = 2000;
const unsigned int timerThreePeriod = 3000;
const unsigned int timerFourPeriod = 4000;
const unsigned int timerFivePeriod = 5000;
const unsigned int timerSixPeriod = 1000;
TimerEvent timerOne;
TimerEvent timerTwo;
TimerEvent timerThree;
TimerEvent timerFour;
TimerEvent timerFive;
TimerEvent timerSix;
//Array, used to store the data of the pattern, can be calculated by yourself or obtained from the modulus tool
unsigned char STOP01[] = 
{0x2E,0x2A,0x3A,0x00,0x02,0x3E,0x02,0x00,0x3E,0x22,0x3E,0x00,0x3E,0x0A,0x0E,0x00};
unsigned char hou[] = 
{0x00, 0x7f, 0x08, 0x08, 0x7f, 0x00, 0x3c, 0x42, 0x42, 0x3c, 0x00, 0x3e, 0x40, 0x40, 0x3e, 0x00
};
unsigned char op[] = 
{0x00, 0x00, 0x3c, 0x42, 0x42, 0x3c, 0x00, 0x7e, 0x12, 0x12, 0x0c, 0x00, 0x00, 0x5e, 0x00, 0x00
};
unsigned char met[] = 
{0xf8, 0x0c, 0xf8, 0x0c, 0xf8, 0x00, 0x78, 0xa8, 0xa8, 0xb8, 0x00, 0x08, 0x08, 0xf8, 0x08, 0x08
};
unsigned char pesto[] = 
{
0xfe, 0x12, 0x12, 0x7c, 0xb0, 0xb0, 0x80, 0xb8, 0xa8, 0xe8, 0x08, 0xf8, 0x08, 0xe8, 0x90, 0xe0
};
unsigned char bleh[] =
{0x00,0x11,0x0a,0x04,0x8a,0x51,0x40,0x40,0x40,0x40,0x51,0x8a,0x04,0x0a,0x11,0x00};
unsigned char left[] = 
{0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x28,0x10,0x44,0x28,0x10,0x44,0x28,0x10,0x00};
unsigned char right[] = 
{0x00,0x10,0x28,0x44,0x10,0x28,0x44,0x10,0x28,0x44,0x00,0x00,0x00,0x00,0x00,0x00};

unsigned char clear[] = 
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
#define SCL_Pin  A5  //Set clock pin to A5
#define SDA_Pin  A4  //Set data pin to A4
#define light_L_Pin A1   //define the pin of left photo resistor
#define light_R_Pin A2   //define the pin of right photo resistor
#define ML_Ctrl 13  //define the direction control pin of left motor
#define ML_PWM 11   //define the PWM control pin of left motor
#define MR_Ctrl 12  //define the direction control pin of right motor
#define MR_PWM 3   //define the PWM control pin of right motor
int left_light; 
int right_light;
int i = 0;
int screen = 1;


void setup(){
  Serial.begin(9600);
  pinMode(light_L_Pin, INPUT);
  pinMode(light_R_Pin, INPUT);
  pinMode(ML_Ctrl, OUTPUT);
  pinMode(ML_PWM, OUTPUT);
  pinMode(MR_Ctrl, OUTPUT);
  pinMode(MR_PWM, OUTPUT);
  pinMode(SCL_Pin,OUTPUT);
  pinMode(SDA_Pin,OUTPUT);

  timerOne.set(timerOnePeriod, timerOneFunc);
  timerTwo.set(timerTwoPeriod, timerTwoFunc);
  timerThree.set(timerThreePeriod, timerThreeFunc);
  timerFour.set(timerFourPeriod, timerFourFunc);
  timerFive.set(timerFivePeriod, timerFiveFunc);
  timerSix.set(timerSixPeriod, timerSixFunc);

}
void loop(){
  timerOne.update();
  timerTwo.update();
  timerThree.update();
  timerFour.update();
  timerFive.update();
  timerSix.update();

  left_light = analogRead(light_L_Pin);
  right_light = analogRead(light_R_Pin);
  Serial.print("left_light_value = ");
  Serial.println(left_light);
  Serial.print("right_light_value = ");
  Serial.println(right_light);
  if (left_light > 650 && right_light > 650) //the value detected photo resistor，go front
  {  
    Car_front();
  } 
  else if (left_light > 650 && right_light <= 650)  //the value detected photo resistor，turn left
  {
    Car_left();
  } 
  else if (left_light <= 650 && right_light > 650) //the value detected photo resistor，turn right
  {
    Car_right();
  } 
  else  //other situations, stop
  {
    Car_Stop();
  }
}

void Car_front()
{
  digitalWrite(MR_Ctrl,HIGH);
  analogWrite(MR_PWM,200);
  digitalWrite(ML_Ctrl,HIGH);
  analogWrite(ML_PWM,200);
}
void Car_left()
{
  digitalWrite(MR_Ctrl,LOW);
  analogWrite(MR_PWM,200);
  digitalWrite(ML_Ctrl,HIGH);
  analogWrite(ML_PWM,200);
}
void Car_right()
{
  digitalWrite(MR_Ctrl,HIGH);
  analogWrite(MR_PWM,200);
  digitalWrite(ML_Ctrl,LOW);
  analogWrite(ML_PWM,200);
}
void Car_Stop()
{
  digitalWrite(MR_Ctrl,LOW);
  analogWrite(MR_PWM,0);
  digitalWrite(ML_Ctrl,LOW);
  analogWrite(ML_PWM,0);
}
//****************************************************************
// This function will be called every timerOnePeriod
void timerOneFunc(){
  matrix_display(STOP01);
}
void timerTwoFunc(){
  matrix_display(hou);
}
void timerThreeFunc(){
  matrix_display(op);
}
void timerFourFunc(){
  matrix_display(met);
}
void timerFiveFunc(){
  matrix_display(pesto);
}
void timerSixFunc(){
  matrix_display(bleh);
}
       
// This function is used to display of dot matrix
void matrix_display(unsigned char matrix_value[])
{
  IIC_start();  //call the function that data transmission start  
  IIC_send(0xc0);  //Choose address
  
  for(int i = 0;i < 16;i++) //pattern data has 16 bits
  {
     IIC_send(matrix_value[i]); //data to convey patterns 
  }
  IIC_end();   //end the transmission of pattern dataEnd
  IIC_start();
  IIC_send(0x8A);  //display control, set pulse width to 4/16
  IIC_end();
}
//The condition starting to transmit data
void IIC_start()
{
  digitalWrite(SCL_Pin,HIGH);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin,HIGH);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin,LOW);
  delayMicroseconds(3);
}
//Convey data
void IIC_send(unsigned char send_data)
{
  for(char i = 0;i < 8;i++)  //Each byte has 8 bits
  {
      digitalWrite(SCL_Pin,LOW);  //pull down clock pin SCL Pin to change the signals of SDA      
delayMicroseconds(3);
      if(send_data & 0x01)  //set high and low level of SDA_Pin according to 1 or 0 of every bit
      {
        digitalWrite(SDA_Pin,HIGH);
      }
      else
      {
        digitalWrite(SDA_Pin,LOW);
      }
      delayMicroseconds(3);
      digitalWrite(SCL_Pin,HIGH); //pull up clock pin SCL_Pin to stop transmitting data
      delayMicroseconds(3);
      send_data = send_data >> 1;  //detect bit by bit, so shift the data right by one
  }}
//The sign that data transmission ends
void IIC_end()
{
  digitalWrite(SCL_Pin,LOW);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin,LOW);
  delayMicroseconds(3);
  digitalWrite(SCL_Pin,HIGH);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin,HIGH);
  delayMicroseconds(3);} 
