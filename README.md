This is my source code for my Arduino uno R4 Wifi robot

|Env| Folder|
|------|-----|
|src|./src|
|esp32_cam|./src_esp32_cam|
|esp32_ps4|./src_esp32_ps4|
|esp32_r4|./src_esp32_r4|


The Arduino's onboard ESP32-S3 is programmed seperatly,
We soldered a ESP32-cam to the MotorShield to receive PS4-controller commands
We have an other ESP32-Cam on the front to have a video feed
 

|Lib-Module| Type|
|------|-----|
|ADS1X15 | Analog|
|BMP280| Barometer|
|HMC5883| compas|
|SH1106| Oled Display|
|MPU6050|Gyroscope|
|Sonar | Distance calculation|
|DotMatrix| for showing emotions|
|PS4| Using ESP32 Cam TX-RX via Serial2 |
|PWMServoDriver x 16| front servos, laser,  rgb-led|
|SdFat| for save and loading setup of module booleans |
|Menu| for setup of SD and logging|
|Mic Module| left and right|
|Light Sensors| Left and right|
