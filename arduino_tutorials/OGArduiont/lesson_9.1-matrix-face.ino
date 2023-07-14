/*
 keyestudio Mini Tank Robot v2.0
 lesson 9.1
 Matrix  face
 http://www.keyestudio.com
*/ 
//the data of smiley from modulus tool
unsigned char smile[] = {0x00, 0x00, 0x1c, 0x02, 0x02, 0x02, 0x5c, 0x40, 0x40, 0x5c, 0x02, 0x02, 0x02, 0x1c, 0x00, 0x00};

#define SCL_Pin  A5  //Set clock pin to A5
#define SDA_Pin  A4  //Set data pin to A4

void setup(){
  //Set pin to output
  pinMode(SCL_Pin,OUTPUT);
  pinMode(SDA_Pin,OUTPUT);
  //clear the display
  //matrix_display(clear);
}
void loop(){
  matrix_display(smile);  // display smile face
}
// the function for dot matrix display
void matrix_display(unsigned char matrix_value[])
{
  IIC_start();  // use the function of the data transmission start condition
  IIC_send(0xc0);  //select address
  
  for(int i = 0;i < 16;i++) //pattern data has 16 bits
  {
     IIC_send(matrix_value[i]); //convey the pattern data
  }

  IIC_end();   //end the transmission of pattern data  
  IIC_start();
  IIC_send(0x8A);  //display control, set pulse width to 4/16 s
  IIC_end();
}

//the condition to start conveying data
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
  for(char i = 0;i < 8;i++)  //Each byte has 8 bits 8bit for every character
  {
      digitalWrite(SCL_Pin,LOW);  // pull down clock pin SCL_Pin to change the signal of SDA
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
      digitalWrite(SCL_Pin,HIGH); //pull up the clock pin SCL_Pin to stop transmission
      delayMicroseconds(3);
      send_data = send_data >> 1;  // detect bit by bit, shift the data to the right by one
  }
}

//The sign of ending data transmission
void IIC_end()
{
  digitalWrite(SCL_Pin,LOW);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin,LOW);
  delayMicroseconds(3);
  digitalWrite(SCL_Pin,HIGH);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin,HIGH);
  delayMicroseconds(3);
}
//******************************************************
