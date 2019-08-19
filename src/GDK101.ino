#include <Wire.h>

#define DELAY_TIME_MAIN_LOOP 60000


int addr = 0x18;
int day, hour, minute, sec = 0;
byte buffer[2] = { 0, 0 };
int status = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    Serial.println("GDK101 start\n\n");

    delay(50);

    Gamma_Mod_Read(0xB4);
    Gamma_Mod_Read(0xA0);
    Serial.println("=========================================================");
    Gamma_Mod_Read_Value();
}

void loop() {
    //Gamma_Mod_Read_Value();
    Gamma_Mod_Read(0xB3); // Read Measuring Value (1 min avg / 1 min update)
    Serial.println("=========================================================");
    delay(DELAY_TIME_MAIN_LOOP);
}

void Gamma_Mod_Read_Value() {
    Gamma_Mod_Read(0xB0); // Read Status
    Gamma_Mod_Read(0xB1); // Read Measuring Time
    Gamma_Mod_Read(0xB2); // Read Measuring Value (10 min avg / 1 min update)
    Gamma_Mod_Read(0xB3); // Read Measuring Value (1 min avg / 1 min update)
}

void Gamma_Mod_Read(int cmd) {
    /* Begin writing sequence */
    Wire.beginTransmission(addr);
    Wire.write(cmd);
    Wire.endTransmission();
    /* End writing sequence */

    delay(10);

    /* Begin reading sequence */
    Wire.requestFrom(addr, 2);
    byte i = 0;

    while (Wire.available()) {
        buffer[i] = Wire.read();
        ++i;
    }
    /* Begin reading sequence */

    //Print out the result
    Print_Result(cmd);
}

void Call_Measuring_Time() {
    if (sec == 60) {
        sec = 0;
        ++minute;
    }
    if (minute == 60) {
        minute = 0;
        ++hour;
    }
    if (sec == 24) {
        sec = 0;
        ++day;
    }

    Serial.print("Measuring time: ");
    Serial.print(day);
    Serial.print("d ");
    if (hour < 10) Serial.print("0");
    Serial.print(hour);
    Serial.print(":");
    if (minute < 10) Serial.print("0");
    Serial.print(minute);
    Serial.print(":");
    if (sec < 10) Serial.print("0");
    Serial.println(sec);
}

void Print_Result(int cmd) {
    float value = 0.0f;

    switch (cmd) {
        case 0xA0 :
            if (buffer[0] != 1) Serial.println("Reset command failed!!");
            else Serial.println("Reset command success!!");
            break;
        case 0xB0 :
            switch (buffer[0]) {
                case 0:
                    Serial.println("Status Ready");
                    break;
                case 1:
                    Serial.println("10min waiting");
                    break;
                case 2:
                    Serial.println("Normal");
            }
            break;
        case 0xB1 :
            if (status > 0) {
                ++sec;
                Call_Measuring_Time();
                break;
            }
            break;
        case 0xB2 :
            Serial.print("Measuring value (10min avg) : ");
            value = buffer[0] + (float) buffer[1] / 100;
            Serial.print(value);
            Serial.println(" uSv/hour");
            break;
        case 0xB3 :
            Serial.print("Measuring value (1min avg) : ");
            value = buffer[0] + (float) buffer[1] / 100;
            Serial.print(value);
            Serial.println(" uSv/hour");
            break;
        case 0xB4 :
            Serial.print("FW Version :  V");
            Serial.print(buffer[0]);
            Serial.print(".");
            Serial.println(buffer[1]);
    }
}
