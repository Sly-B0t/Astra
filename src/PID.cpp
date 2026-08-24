#include <PID.h>
#include <Arduino.h>

namespace
{
    const PIDGains rollGains = {8.0f, 0.0f, 0.35f};
    const PIDGains pitchGains = {8.0f, 0.0f, 0.35f};
    const PIDGains yawGains = {4.0f, 0.0f, 0.10f};
    const PIDGains altitudeGains = {90.0f, 4.0f, 35.0f};
}

PID::PID()
{
    goalOrientation = {};
    actualOrientation = {};
    motorThrust = {idleThrottleUs, idleThrottleUs, idleThrottleUs, idleThrottleUs};
    previousUpdateMicros = 0;
    rollState = {};
    pitchState = {};
    yawState = {};
    altitudeState = {};
}

PID::~PID()
{
}

Motors PID::getMotorThrust()
{
    return motorThrust;
}

void PID::setOrientation(Orientation goal, Orientation actual)
{
    goalOrientation = goal;
    actualOrientation = actual;
}

void PID::updatePID()
{
}
