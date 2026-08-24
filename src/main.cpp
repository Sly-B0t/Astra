#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LPS.h>
#include <Helpers.h>
#include <InputManager.h>
#include <PID.h>

InputManager *IM = nullptr;
PID pid;

QueueHandle_t pidQueue;
QueueHandle_t motorQueue;
volatile bool armed = false;

void IOCore(void *parameter);
void PIDCore(void *parameter);
void LEDCore(void *parameter);

uint8_t CONTROLLER_MAC[] = {0xA4, 0xCB, 0x8F, 0x20, 0x55, 0xD8};

void setup()
{
    Serial.begin(115200);
    delay(300);
    String firmwareversion = FIRMWAREVER;
    Serial.println("Astra Revision" + firmwareversion + "starting...");
    Serial.println("Starting I2C.");
    blinkStartupCode(1);
    Wire.begin(SERIALDATAPIN, SERIALCLOCKPIN);
    Wire.setClock(400000);
    Serial.println("I2C started at 400kHz.");
    Serial.println("Creating InputManager...");
    IM = new InputManager();
    Serial.println("Startup LED code 2: sensors and motors initialized.");
    blinkStartupCode(2);

    Serial.println("Creating FreeRTOS queues...");
    pidQueue = xQueueCreate(1, sizeof(PIDData));
    motorQueue = xQueueCreate(1, sizeof(Motors));

    if (pidQueue == nullptr || motorQueue == nullptr)
    {
        Serial.println("Failed to create queues");
        while (true)
        {
            delay(1000);
        }
    }
    Serial.println("Queues ready.");

    Serial.println("Starting tasks...");
    xTaskCreatePinnedToCore(IOCore, "IO Core", 4096, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(PIDCore, "PID Core", 4096, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(LEDCore, "LED Core", 2048, nullptr, 1, nullptr, 0);
    Serial.println("Startup LED code 3: tasks started.");
    blinkStartupCode(3);
    Serial.println("=== Astra startup complete ===");
}

void loop()
{
    vTaskDelay(portMAX_DELAY);
}

void IOCore(void *parameter)
{
}

void PIDCore(void *parameter)
{
}

void LEDCore(void *parameter)
{
    while (true)
    {
        if (armed)
        {
            setStatusLed(true);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        else
        {
            blinkStatusLed(80, 920);
        }
    }
}
