#include <Bluepad32.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LPS.h>
#include <Helpers.h>
#include <InputManager.h>

InputManager IM;

void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22);
}

// Arduino loop function. Runs in CPU 1.
void loop()
{
}
