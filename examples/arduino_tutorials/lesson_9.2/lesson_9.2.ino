
/* keyestudio Mini Tank Robot v2.0
 lesson 9.2
 Matrix loop
 http://www.keyestudio.com
*/ 
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
#define SCL_Pin  6  //Set clock pin to A5
#define SDA_Pin  7  //Set data pin to A4
void setup(){
  //Set pins to output
  pinMode(SCL_Pin,OUTPUT);
  pinMode(SDA_Pin,OUTPUT);
  //Clear the display
  matrix_display(clear);
}
void loop(){
  matrix_display(STOP01);  // Display start pattern
  delay(2000);
  matrix_display(hou);    //Front pattern
  delay(2000);
  matrix_display(op);    //Front pattern
  delay(2000);
  matrix_display(met);    //Front pattern
  delay(2000);
  matrix_display(pesto);    //Front pattern
  delay(2000);
  matrix_display(bleh);  
  delay(2000);
  matrix_display(clear);    //Clear the display Clear the screen
  delay(2000);
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
//*****************************************************
