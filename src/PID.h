#pragma once

#include <Helpers.h>

struct PIDGains
{
    float kp;
    float ki;
    float kd;
};

struct PIDState
{
    float integral;
    float previousError;
};

class PID
{
private:
    Orientation goalOrientation;
    Orientation actualOrientation;
    Motors motorThrust;
    unsigned long previousUpdateMicros;

    PIDState rollState;
    PIDState pitchState;
    PIDState yawState;
    PIDState altitudeState;

    static constexpr float escMinUs = 1000.0f;
    static constexpr float escMaxUs = 2000.0f;
    static constexpr float idleThrottleUs = 1000.0f;
    static constexpr float hoverThrottleUs = 1350.0f;

    static constexpr float attitudeCorrectionLimitUs = 420.0f;
    static constexpr float yawCorrectionLimitUs = 320.0f;
    static constexpr float altitudeCorrectionLimitUs = 250.0f;
    static constexpr float integralLimit = 120.0f;

    float runAxisPID(float error, float dt, const PIDGains &gains, PIDState &state, float outputLimit);
    float shortestAngleError(float target, float current);
    float getCollectiveThrottle(float dt);
    void writeMixedMotors(float collectiveUs, float rollCorrection, float pitchCorrection, float yawCorrection);
    void writeIdle();
    float clampMotor(float value);

public:
    PID();
    ~PID();
    Motors getMotorThrust();
    void setOrientation(Orientation goal, Orientation Actual);
    void updatePID();
};
