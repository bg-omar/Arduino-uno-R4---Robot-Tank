

// Include the libraries:
#include <TimerEvent.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x3F for a 16 chars and 2 line display
byte Heart[8] = {
0b00000,
0b01010,
0b11111,
0b11111,
0b01110,
0b00100,
0b00000,
0b00000
};

byte Bell[8] = {
0b00100,
0b01110,
0b01110,
0b01110,
0b11111,
0b00000,
0b00100,
0b00000
};


byte Alien[8] = {
0b11111,
0b10101,
0b11111,
0b11111,
0b01110,
0b01010,
0b11011,
0b00000
};

byte Check[8] = {
0b00000,
0b00001,
0b00011,
0b10110,
0b11100,
0b01000,
0b00000,
0b00000
};

byte Speaker[8] = {
0b00001,
0b00011,
0b01111,
0b01111,
0b01111,
0b00011,
0b00001,
0b00000
};


byte Sound[8] = {
0b00001,
0b00011,
0b00101,
0b01001,
0b01001,
0b01011,
0b11011,
0b11000
};


byte Skull[8] = {
0b00000,
0b01110,
0b10101,
0b11011,
0b01110,
0b01110,
0b00000,
0b00000
};

byte Lock[8] = {
0b01110,
0b10001,
0b10001,
0b11111,
0b11011,
0b11011,
0b11111,
0b00000
};

//Array, used to store the data of the pattern, can be calculated by yourself or obtained from the modulus tool
unsigned char data_line = 0;
unsigned char delay_count = 0;
unsigned char STOP01[] = {0x2E,0x2A,0x3A,0x00,0x02,0x3E,0x02,0x00,0x3E,0x22,0x3E,0x00,0x3E,0x0A,0x0E,0x00};
unsigned char hou[] = {0x00, 0x7f, 0x08, 0x08, 0x7f, 0x00, 0x3c, 0x42, 0x42, 0x3c, 0x00, 0x3e, 0x40, 0x40, 0x3e, 0x00};
unsigned char op[] = {0x00, 0x00, 0x3c, 0x42, 0x42, 0x3c, 0x00, 0x7e, 0x12, 0x12, 0x0c, 0x00, 0x00, 0x5e, 0x00, 0x00};
unsigned char met[] = {0xf8, 0x0c, 0xf8, 0x0c, 0xf8, 0x00, 0x78, 0xa8, 0xa8, 0xb8, 0x00, 0x08, 0x08, 0xf8, 0x08, 0x08};
unsigned char pesto[] = {0xfe, 0x12, 0x12, 0x7c, 0xb0, 0xb0, 0x80, 0xb8, 0xa8, 0xe8, 0x08, 0xf8, 0x08, 0xe8, 0x90, 0xe0};
unsigned char bleh[] ={0x00,0x11,0x0a,0x04,0x8a,0x51,0x40,0x40,0x40,0x40,0x51,0x8a,0x04,0x0a,0x11,0x00};
unsigned char clear[] = 
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
#define SCL_Pin  6  //Set clock pin to A5
#define SDA_Pin  7  //Set data pin to A4    
void IIC_start();
void IIC_send(unsigned char send_data);
void IIC_end();
void matrix_display(unsigned char matrix_value[]);

#define ML_Ctrl 13  //define the direction control pin of left motor
#define ML_PWM 11   //define PWM control pin of left motor
#define MR_Ctrl 12  //define the direction control pin of right motor
#define MR_PWM 3   //define PWM control pin of right motor
#define Trig 5  //ultrasonic trig Pin
#define Echo 4  //ultrasonic echo Pin
#define servoPin 9  //servo Pin
#define Led 10
// Initialize DHT sensor for normal 16mhz Arduino:
#define DHTPIN 8
#define DHTTYPE DHT11    
DHT dht = DHT(DHTPIN, DHTTYPE);

Adafruit_MPU6050 mpu;

const unsigned int timerOnePeriod = 1000;
const unsigned int timerTwoPeriod = 1250;
const unsigned int timerThreePeriod = 1666;
const unsigned int timerFourPeriod = 2200;
const unsigned int timerFivePeriod = 2500;
const unsigned int timerSixPeriod = 2700;
const unsigned int timerDHTPeriod = 1440;
const unsigned int timerGyroPeriod = 1000;

TimerEvent timerOne;
TimerEvent timerTwo;
TimerEvent timerThree;
TimerEvent timerFour;
TimerEvent timerFive;
TimerEvent timerSix;
TimerEvent timerDHT;
TimerEvent timerGyro;
    
int random2;
int distancef;
int distance1;
int distance2;
int distance;
int analogInPinR = A1;  // Analog input pin that the photocell is attached to
int analogInPinL = A0;  // Analog input pin that the photocell is attached to
int sensorValueR = 0;        // value read from the pot
int sensorValueL = 0;        // value read from the pot
int outputValueR = 0;        // value output to the PWM (analog out) 
int outputValueL = 0;        // value output to the PWM (analog out) 
int calcValue = 255;      // inverse input
int pulsewidth;

/************the function to run motor**************/
void Car_front()
{
  analogWrite(Led, calcValue);
  digitalWrite(MR_Ctrl,HIGH);
  analogWrite(MR_PWM,255);
  digitalWrite(ML_Ctrl,HIGH);
  analogWrite(ML_PWM,255);
}
void Car_back()
{
  digitalWrite(MR_Ctrl,LOW);
  analogWrite(MR_PWM,200);
  digitalWrite(ML_Ctrl,LOW);
  analogWrite(ML_PWM,200);
}
void Car_left()
{
  digitalWrite(MR_Ctrl,LOW);
  analogWrite(MR_PWM,255);
  digitalWrite(ML_Ctrl,HIGH);
  analogWrite(ML_PWM,255);
}
void Car_right()
{
  digitalWrite(MR_Ctrl,HIGH);
  analogWrite(MR_PWM,255);
  digitalWrite(ML_Ctrl,LOW);
  analogWrite(ML_PWM,255);
}
void Car_Stop()
{
  digitalWrite(MR_Ctrl,LOW);
  analogWrite(MR_PWM,0);
  digitalWrite(ML_Ctrl,LOW);
  analogWrite(ML_PWM,0);
}

// the function for dot matrix display
void matrix_display(unsigned char matrix_value[])
{
    /**************set the address plus 1***************/
    IIC_start();
    IIC_send(0x40);// set the address plus 1 automatically
    IIC_end();
    /************end the process of address plus 1 *****************/
    /************set the data display*****************/ 
    IIC_start();
    IIC_send(0xc0);// set the initial address as 0
    for(int i = 0;i < 16;i++) //pattern data has 16 bits
    {
        IIC_send(matrix_value[i]); //convey the pattern data
    }
    if(++delay_count >= 10)
    {
      delay_count = 0;
      data_line++;
      if(data_line >= 4)
      {
        data_line = 0;
      }
    }
    
    IIC_end();
    /************end the data display*****************/
    /*************set the brightness display***************/ 
    IIC_start();
    IIC_send(0x8A);// set the brightness display
    IIC_end(); 
    /*************end the brightness display***************/ 
    delay(100);
}

//the condition to start conveying data
void IIC_start()
{
    digitalWrite(SCL_Pin,HIGH);
    delayMicroseconds(3);
    digitalWrite(SDA_Pin,HIGH);
    delayMicroseconds(3);
    digitalWrite(SCL_Pin,LOW);
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

void timerGyroFunc(){
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  lcd.setCursor(0,0); // Sets the location at which subsequent text written to the LCD will be displayed
  lcd.print("A:"); // m/s2
  lcd.print(a.acceleration.x);
  lcd.print(" ");
  lcd.print(a.acceleration.y);
  lcd.print(" ");
  lcd.print(a.acceleration.z);
  lcd.setCursor(0,1);
  lcd.print("R:");
  lcd.print(g.gyro.x);
  lcd.print(" ");
  lcd.print(g.gyro.y);
  lcd.print(" ");
  lcd.print(g.gyro.z);
}
void timerDHTFunc(){
 // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)

  // Read the humidity in %:
  float h = dht.readHumidity();
  // Read the temperature as Celsius:
  float t = dht.readTemperature();

  // Check if any reads failed and exit early (to try again):
  if (isnan(h) || isnan(t) ) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Serial.print("Humidity: ");
  Serial.println(h);
  Serial.print("Temperature: ");
  Serial.println(t);
  lcd.setCursor(0,0); // Sets the location at which subsequent text written to the LCD will be displayed
  lcd.print("T:"); // Prints string "Temp." on the LCD
  lcd.print(t); // Prints the temperature value from the sensor
  lcd.print(" H:");
  lcd.print(h);
  lcd.print("%");
}
  
//The function to control servo
void procedure(int myangle) {
  for (int i = 0; i <= 50; i = i + (1)) {
    pulsewidth = myangle * 11 + 500;
    digitalWrite(servoPin,HIGH);
    delayMicroseconds(pulsewidth);
    digitalWrite(servoPin,LOW);
    delay((20 - pulsewidth / 1000));
  }
}
//The function to control ultrasonic sensor
float checkdistance() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  float distance = pulseIn(Echo, HIGH) / 58.00;  //58.20, that is, 2*29.1=58.2
  delay(10);
  return distance;
}
  //****************************************************************

void setup(){
  Serial.begin(115200);

  pinMode(servoPin, OUTPUT);
  procedure(85); //set servo to 90°
  
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(ML_Ctrl, OUTPUT);
  pinMode(ML_PWM, OUTPUT);
  pinMode(MR_Ctrl, OUTPUT);
  pinMode(MR_PWM, OUTPUT);
  pinMode(Led, OUTPUT);
  
  pinMode(SCL_Pin,OUTPUT);
  pinMode(SDA_Pin,OUTPUT);
  digitalWrite(SCL_Pin,LOW);
  digitalWrite(SDA_Pin,LOW);
  //Clear the display
  matrix_display(clear);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // 2, 4, 8, 16G
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);      // 250, 500, 1000, 2000 deg
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);    // 5, 10, 21, 44, 94, 164, 260hz
  timerOne.set(timerOnePeriod, timerOneFunc);
  timerTwo.set(timerTwoPeriod, timerTwoFunc);
  timerThree.set(timerThreePeriod, timerThreeFunc);
  timerFour.set(timerFourPeriod, timerFourFunc);
  timerFive.set(timerFivePeriod, timerFiveFunc);
  timerSix.set(timerSixPeriod, timerSixFunc);
  timerDHT.set(timerDHTPeriod, timerDHTFunc);
  timerGyro.set(timerGyroPeriod, timerGyroFunc);   
  dht.begin();
  lcd.init();
  lcd.clear();         
  lcd.backlight();      // Make sure backlight is on
  // create a new characters
  lcd.createChar(0, Heart);
  lcd.createChar(1, Bell);
  lcd.createChar(2, Alien);
  lcd.createChar(3, Check);
  lcd.createChar(4, Speaker);
  lcd.createChar(5, Sound);
  lcd.createChar(6, Skull);
  lcd.createChar(7, Lock);
  
  // Print a message on both lines of the LCD.
  lcd.setCursor(2,0);   //Set cursor to character 2 on line 0
  lcd.print("Hello Pesto!");
  lcd.setCursor(3, 1);
  lcd.write(0);
  lcd.setCursor(5, 1);
  lcd.print("Love u");
  lcd.setCursor(12, 1);
  lcd.write(0);
}


void loop(){

  timerOne.update();
  timerTwo.update();
  timerThree.update();
  timerFour.update();
  timerFive.update();
  timerSix.update();
  timerDHT.update();
  timerGyro.update();
  

  random2 = random(1, 100);
  sensorValueR = analogRead(analogInPinR);
  sensorValueL = analogRead(analogInPinL);
  outputValueR = map(sensorValueR, 0, 1023, 0, 255);
  outputValueL = map(sensorValueL, 0, 1023, 0, 255);
  calcValue = 255 - ((outputValueR + outputValueL) * 1.5);
  calcValue = (calcValue < 0) ? 0 : calcValue;
  distancef = checkdistance();  //assign the front distance detected by ultrasonic sensor to variable a

  if (distancef < 30) //when the front distance detected is less than 20 
  {
      Car_Stop();  //robot stops
      analogWrite (Led, 250);
      delay(500); //delay in 500ms
      
      procedure(160);  //Ultrasonic platform turns left
      //for statement, the data will be more accurate if ultrasonic sensor detect a few times.
      for (int j = 1; j <= 10; j = j + (1)) { 
        //assign the left distance detected by ultrasonic sensor to variable a1
        distance1 = checkdistance();  
      }
      delay(300);
      
      procedure(20); //Ultrasonic platform turns right
      for (int k = 1; k <= 10; k = k + (1)) {
        //assign the right distance detected by ultrasonic sensor to variable a2
        distance2 = checkdistance(); 
      }
      analogWrite (Led, 0);
      if (distance1 < 50 || distance2 < 50)  //robot will turn to the longer distance side when left or right distance is less than 50cm. 
      {
        if (distance1 > distance2) //left distance is greater than right side      
        {
          procedure(85);  //Ultrasonic platform turns back to right ahead         
          Car_left();  //robot turns left
          delay(500);  //turn left for 500ms
          Car_front(); //go front
        } 
        else 
        {
          procedure(85);
          Car_right(); //robot turns right
          delay(500);
          Car_front();  //go front
        }
      } 
      else  //If both side is greater than or equal to 50cm, turn left or right randomly
      {
        if ((long) (random2) % (long) (2) == 0)  //When the random number is even
        {
          procedure(85);
          Car_left(); //tank robot turns left
          delay(500);
          Car_front(); //go front
        } 
        else 
        {
          procedure(85);
          Car_right(); //robot turns right
          delay(500);
          Car_front(); //go front
       }
     }
  } 
  else  //If the front distance is greater than or equal to 20cm, robot car will go front
  {
      Car_front(); //go front
  }
}
