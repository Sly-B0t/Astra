#include <Bluepad32.h>
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
    PIDData newData;
    Motors idleSpeed = {1000, 1000, 1000, 1000};
    Motors currentSpeed = idleSpeed;
    Motors newSpeed;
    bool previousArmButton = false;
    bool previousDisarmButton = false;
    bool previousConnected = false;
    unsigned long lastStatusPrintMs = 0;

    while (true)
    {
        IM->updateControllers();
        bool connected = IM->isControllerConnected();
        Control controls = IM->getController();
        bool armPressed = connected && controls.a;
        bool disarmPressed = connected && controls.b;

        if (connected != previousConnected)
        {
            Serial.println(connected ? "IO: controller connected." : "IO: controller not connected.");
            previousConnected = connected;
        }

        if (!connected && armed)
        {
            armed = false;
            xQueueReset(pidQueue);
            xQueueReset(motorQueue);
            currentSpeed = idleSpeed;
            IM->setMotors(idleSpeed);
            Serial.println("DISARMED: controller disconnected.");
        }

        if (armPressed && !previousArmButton && !armed && controls.r2 < 0.05f && controls.l2 < 0.05f)
        {
            xQueueReset(pidQueue);
            xQueueReset(motorQueue);
            armed = true;
            Serial.println("ARMED.");
        }

        if (disarmPressed && !previousDisarmButton && armed)
        {
            armed = false;
            xQueueReset(pidQueue);
            xQueueReset(motorQueue);
            currentSpeed = idleSpeed;
            IM->setMotors(idleSpeed);
            Serial.println("DISARMED.");
        }

        // Get fresh values before sending them.
        newData.actual = IM->getOrientation();
        newData.goal = armed ? IM->getGoal() : Orientation{};

        if (armed)
        {
            // Send the newest sensor/controller data to the PID core.
            xQueueOverwrite(pidQueue, &newData);
        }
        else
        {
            currentSpeed = idleSpeed;
        }

        // Check whether the PID core produced new motor speeds.
        // A timeout of 0 means do not wait.
        if (xQueueReceive(motorQueue, &newSpeed, 0) == pdTRUE)
        {
            currentSpeed = armed ? newSpeed : idleSpeed;
        }

        // Apply either the new speeds or the previous valid speeds.
        IM->setMotors(currentSpeed);

        unsigned long now = millis();
        if (now - lastStatusPrintMs >= STATUS_PRINT_INTERVAL_MS)
        {
            lastStatusPrintMs = now;
            Serial.printf("STATUS armed=%d connected=%d goal[p=%.1f r=%.1f y=%.1f thr=%.2f] actual[p=%.1f r=%.1f y=%.1f alt=%.2f] motors[%.0f %.0f %.0f %.0f]\n",
                          armed,
                          connected,
                          newData.goal.pitch,
                          newData.goal.roll,
                          newData.goal.yaw,
                          newData.goal.altitude,
                          newData.actual.pitch,
                          newData.actual.roll,
                          newData.actual.yaw,
                          newData.actual.altitude,
                          currentSpeed.m1,
                          currentSpeed.m2,
                          currentSpeed.m3,
                          currentSpeed.m4);
        }

        previousArmButton = armPressed;
        previousDisarmButton = disarmPressed;

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void PIDCore(void *parameter)
{
    PIDData receivedData;
    Motors calculatedSpeed;
    unsigned long lastPidPrintMs = 0;

    while (true)
    {
        if (xQueueReceive(pidQueue, &receivedData, portMAX_DELAY) == pdTRUE)
        {
            pid.setOrientation(receivedData.goal, receivedData.actual);
            pid.updatePID();
            calculatedSpeed = pid.getMotorThrust();
            // Replace any uncollected result with the newest result.
            xQueueOverwrite(motorQueue, &calculatedSpeed);

            unsigned long now = millis();
            if (now - lastPidPrintMs >= STATUS_PRINT_INTERVAL_MS)
            {
                lastPidPrintMs = now;
                Serial.printf("PID: goal[p=%.1f r=%.1f y=%.1f a=%.2f] actual[p=%.1f r=%.1f y=%.1f a=%.2f] out[%.0f %.0f %.0f %.0f]\n",
                              receivedData.goal.pitch,
                              receivedData.goal.roll,
                              receivedData.goal.yaw,
                              receivedData.goal.altitude,
                              receivedData.actual.pitch,
                              receivedData.actual.roll,
                              receivedData.actual.yaw,
                              receivedData.actual.altitude,
                              calculatedSpeed.m1,
                              calculatedSpeed.m2,
                              calculatedSpeed.m3,
                              calculatedSpeed.m4);
            }
        }
    }
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
