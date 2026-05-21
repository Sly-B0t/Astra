#pragma once
#include <ESP32Servo.h>
#include "Helpers.h"
#include <Arduino.h>

struct Motor
{
    float power = 0;
};

class Motors
{
public:
    Motors();
    void begin();
    void writeAllMotors(int us1, int us2, int us3, int us4);
    void PID(InputStruct InputNow, InputStruct InputGoal);
    void setConstants(float kp, float ki, float kd);
    void resetPID();

private:
    Servo motor1;
    Servo motor2;
    Servo motor3;
    Servo motor4;
    Motor motors[4];
    Motor goalMotor[4];
    float kp = PROPORTIONAL;
    float ki = INTEGRAL;
    float kd = DERIVITAVE;
    unsigned long prevTime = 0;
    float prevRollError = 0;
    float prevPitchError = 0;
    float prevYawError = 0;
    float yawIntegral = 0;
    float pitchIntegral = 0;
    float rollIntegral = 0;
};