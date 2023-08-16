void bluetooth(){
    if (Serial.available())
    {
        bluetooth_val = Serial.read();
        Serial.println(bluetooth_val);
    }
    switch (bluetooth_val)
    {
        case 'F': //Forward instruction
            Car_front();
            matrix_display(front); //display forward pattern
            break;
        case 'B': //Back instruction
            Car_Back();
            matrix_display(back); // display back pattern
            break;
        case 'L': //left-turning instruction
            Car_left();
            matrix_display(left); //show left-turning pattern
            break;
        case 'R': //right-turning instruction
            Car_right();
            matrix_display(right); //show right-turning pattern
            break;
        case 'S': //stop instruction
            Car_Stop();
            matrix_display(STOP01); //display stop pattern
            break;
        case 'Y':
            matrix_display(pesto); //show start pattern
            dance();
            break;
        case 'U':
            matrix_display(bleh); //show start pattern
            avoid();
            break;
        case 'X':
            matrix_display(bleh); //show start pattern
            light_track();
            break;
        case 'C':
            lcd.clear();
            timerTwoActive = !timerTwoActive;
            timerTreeActive = false;
            timerButton = Rem_9;
            delay(100);
            break;
        case 'A':
            lcd.clear();
            timerTwoActive = !timerTwoActive;
            timerTreeActive = false;
            timerButton = Rem_7;
            delay(100);
            break;
        case 'a': posXY = min(180, posXY + speedXY); break;
        case 'w': posZ = min(160, posZ + speedZ); break;
        case 'd': posXY = max(0, posXY - speedXY); break;
        case 'q': posXY = 90; posZ = 45; break;
        case 's': posZ = max(0, posZ - speedZ); break;
        case 'e': posXY = 90; posZ = 15; break;
    }
}