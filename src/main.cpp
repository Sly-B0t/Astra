#include <Bluepad32.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include "Input.h"
#include "Helpers.h"
#include "Motor.h"

float batteryVoltage = 12.0f;
float batteryPercent = 100.0f;

bool deadBattery = false;
bool armed = false;

void controllerInput(void *pvParameters);
void control(void *pvParameters);
void telemetry(void *pvParameters);

Motors motors;
Inputs inputs;

void setup()
{
    Serial.begin(115200);
    delay(1500);
    Serial.println("Drone controller starting...");
    // begin Input
    inputs.begin();

    // Begin Motors
    motors.begin();

    Serial.println("If ESCs are powered, wait for arming beeps.");
    delay(4000);
    armed = false;

    // Setup some Threads
    xTaskCreate(controllerInput, "Controller Input", 4096, NULL, 2, NULL);
    xTaskCreate(control, "Control", 4096, NULL, 1, NULL);
    xTaskCreate(telemetry, "Telemetry", 4096, NULL, 1, NULL);
}

void loop()
{
}

// ===================== TASKS =====================

void controllerInput(void *pvParameters)
{
    while (true)
    {
        BP32.update();
        InputStruct data = inputs.GetInput();
        if (controller->a() && data.throttle < 0.05f && !deadBattery)
        {
            motors.resetPID();
            armed = true;
        }

        if (controller->b())
        {
            motors.resetPID();
            armed = false;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void control(void *pvParameters)
{
    unsigned long lastTime = millis();
    unsigned long lastPrint = 0;

    while (true)
    {
        // Get Player Input
        InputStruct InputGoal = inputs.GetGoal();
        // Get Current Drone Orientation
        InputStruct InputNow = inputs.GetIMU();
        // Act on to get Orientation = Goal
        motors.PID(InputNow, InputGoal);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void telemetry(void *pvParameters)
{
    while (true)
    {
        uint32_t mv = analogReadMilliVolts(BATTERYADCINPUT);

        batteryVoltage = (mv / 1000.0f) * 4.0f;
        batteryPercent = batteryPercentFromVoltage(batteryVoltage);

        if (batteryVoltage <= BATTERY_CUTOFF_VOLTAGE)
        {
            deadBattery = true;
            armed = false;
        }
        else
        {
            deadBattery = false;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
