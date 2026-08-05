#include <LPS.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Bluepad32.h>
#include <Helpers.h>
class InputManager
{
private:
    static LPS ps;
    static Adafruit_ICM20948 icm;
    static ControllerPtr controller;

public:
    InputManager();
    ~InputManager();
    static void onConnectedController(ControllerPtr ctl);
    static void onDisconnectedController(ControllerPtr ctl);
    void dumpGamepad(ControllerPtr ctl);
    void processGamepad(ControllerPtr ctl);
    float getAltitude();
    float getTemperature();
    float getTemperatureBar();
    float getTemperatureICM();
    Control getController();
    Orientation getOrientation();
    Orientation sensorFusion(ICM icm);
    void setMotor(float speed, int motor);
    void setMotors(float m1, float m2, float m3, float m4);
};