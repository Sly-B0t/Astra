#pragma once

#include <Arduino.h>

struct ICM
{
    float gx, gy, gz;
    float ay, ax, az;
    float mx, my, mz;
    float temp;
};

struct Orientation
{
    float yaw, roll, pitch;
    float altitude;
};

struct Control
{
    int a, b, r1, l1; // X, Circle, R1, and L1
    float r2, l2;     // R2 and L2
    float ry;         // Right analog stick Y only
    // r1 and l1 for yaw
    // r2 and l2 for altitude
    // ry for angling down or up
    // x and circle for arming and disarming
};

struct PIDData
{
    Orientation goal, actual;
};

// IO Pins
#define SERIALCLOCKPIN 9
#define SERIALDATAPIN 8

#define MOTOR1PIN 11
#define MOTOR2PIN 12
#define MOTOR3PIN 13
#define MOTOR4PIN 14
#define BATTERYADCINPUT 10
#define BAR_ADDR 0x5D
#define MPU_ADDR 0x68
#define MPU_ADDR_ALT 0x69
#define LED_18 18
#define LED_ACTIVE_LEVEL HIGH

#define STATUS_PRINT_INTERVAL_MS 1000

#define LPS_ERROR 1
#define IMU_ERROR 2
#define I2C_ERROR 3

inline void setStatusLed(bool on)
{
    pinMode(LED_18, OUTPUT);
    digitalWrite(LED_18, on ? LED_ACTIVE_LEVEL : !LED_ACTIVE_LEVEL);
}

inline void blinkStatusLed(unsigned int onMs, unsigned int offMs)
{
    setStatusLed(true);
    delay(onMs);
    setStatusLed(false);
    delay(offMs);
}

inline void blinkStartupCode(int code)
{
    for (int repeat = 0; repeat < 2; repeat++)
    {
        for (int i = 0; i < code; i++)
        {
            blinkStatusLed(90, 120);
        }
        blinkStatusLed(420, 650);
    }
}

inline void callError(int code)
{
    Serial.print("Fatal startup error code: ");
    Serial.println(code);

    while (1)
    {
        blinkStartupCode(code);
        delay(1200);
    }
}

struct Motors
{
    float m1, m2, m3, m4;
};
