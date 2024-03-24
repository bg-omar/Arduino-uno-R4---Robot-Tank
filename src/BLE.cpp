//
// Created by mr on 3/24/2024.
//

#include "BLE.h"


if (Serial.available())
{
bluetooth_val = Serial.read();
Serial.println(bluetooth_val);
}
switch (bluetooth_val)
{
case 'F': ///Forward instruction
Car_front();
break;
case 'B': ///Back instruction
Car_back();
break;
case 'L': ///left-turning instruction
Car_left();
break;
case 'R': ///right-turning instruction
Car_right();
break;
case 'S': ///stop instruction
Car_Stop();
break;
case 'Y':
follow();
break;
case 'U':
avoid();
break;
case 'X':
light_track();
break;
}
}
