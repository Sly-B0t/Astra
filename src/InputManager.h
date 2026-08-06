#pragma once

#include <LPS.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Bluepad32.h>
#include <Helpers.h>
#include <ESP32Servo.h>

class InputManager
{
private:
    static LPS ps;
    static Adafruit_ICM20948 icm;
    static ControllerPtr controller;
    ICM icm_struct = {};
    Orientation orientation = {};
    unsigned long previousFusionTime = 0;
    float gyroXOffset = 0.0f;
    float gyroYOffset = 0.0f;
    float gyroZOffset = 0.0f;
    Servo m1;
    Servo m2;
    Servo m3;
    Servo m4;

public:
    InputManager();
    ~InputManager();
    static void onConnectedController(ControllerPtr ctl);
    static void onDisconnectedController(ControllerPtr ctl);
    void dumpGamepad(ControllerPtr ctl);
    void processGamepad(ControllerPtr ctl);
    void updateControllers();
    bool isControllerConnected() const;
    float getAltitude();
    float getAverageTemperature();
    float getTemperatureBar();
    float getTemperatureICM();
    Control getController();
    Orientation getOrientation();
    Orientation sensorFusion();
    void setMotor(float speed, Servo motor);
    void setMotors(Motors motors);
    void updateICM();
    Orientation getGoal();
};
