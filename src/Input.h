#include <Bluepad32.h>
#include <Arduino.h>
#include "Helpers.h"
#include <MPU9250.h>

extern float yaw;
extern Attitude attitude;
extern float armed;
ControllerPtr controller;
class Inputs
{
public:
    Inputs();
    void begin();
    float applyDeadzone(float x);
    InputStruct GetInput();
    InputStruct GetIMU();
    InputStruct GetGoal();

private:
    InputStruct InputGoal;
    InputStruct InputNow;
    MPU9250 mpu;
};
void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);