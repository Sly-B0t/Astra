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
    unsigned long now = micros();
    if (previousUpdateMicros == 0)
    {
        previousUpdateMicros = now;
        writeIdle();
        return;
    }

    float dt = (now - previousUpdateMicros) / 1000000.0f;
    previousUpdateMicros = now;

    if (dt <= 0.0f || dt > 0.1f)
    {
        writeIdle();
        rollState = {};
        pitchState = {};
        yawState = {};
        altitudeState = {};
        return;
    }

    float collectiveUs = getCollectiveThrottle(dt);
    if (collectiveUs <= idleThrottleUs + 5.0f)
    {
        writeIdle();
        rollState = {};
        pitchState = {};
        yawState = {};
        altitudeState = {};
        return;
    }

    float rollError = goalOrientation.roll - actualOrientation.roll;
    float pitchError = goalOrientation.pitch - actualOrientation.pitch;
    float yawError = shortestAngleError(goalOrientation.yaw, actualOrientation.yaw);

    float rollCorrection = runAxisPID(rollError, dt, rollGains, rollState, attitudeCorrectionLimitUs);
    float pitchCorrection = runAxisPID(pitchError, dt, pitchGains, pitchState, attitudeCorrectionLimitUs);
    float yawCorrection = runAxisPID(yawError, dt, yawGains, yawState, yawCorrectionLimitUs);

    writeMixedMotors(collectiveUs, rollCorrection, pitchCorrection, yawCorrection);
}

float PID::runAxisPID(float error, float dt, const PIDGains &gains, PIDState &state, float outputLimit)
{
    state.integral += error * dt;
    state.integral = constrain(state.integral, -integralLimit, integralLimit);

    float derivative = (error - state.previousError) / dt;
    state.previousError = error;

    float output = gains.kp * error + gains.ki * state.integral + gains.kd * derivative;
    return constrain(output, -outputLimit, outputLimit);
}

float PID::shortestAngleError(float target, float current)
{
    float error = target - current;
    while (error > 180.0f)
    {
        error -= 360.0f;
    }
    while (error < -180.0f)
    {
        error += 360.0f;
    }
    return error;
}

float PID::getCollectiveThrottle(float dt)
{
    // If goal.altitude is 0..1, treat it as a direct normalized throttle command.
    if (goalOrientation.altitude >= 0.0f && goalOrientation.altitude <= 1.0f)
    {
        return escMinUs + goalOrientation.altitude * (escMaxUs - escMinUs);
    }

    // Otherwise treat goal.altitude and actual.altitude as meters for altitude hold.
    float altitudeError = goalOrientation.altitude - actualOrientation.altitude;
    float altitudeCorrection = runAxisPID(altitudeError,
                                          dt,
                                          altitudeGains,
                                          altitudeState,
                                          altitudeCorrectionLimitUs);
    return constrain(hoverThrottleUs + altitudeCorrection, escMinUs, escMaxUs);
}

void PID::writeMixedMotors(float collectiveUs, float rollCorrection, float pitchCorrection, float yawCorrection)
{
    // M1 rear-left, M2 front-left, M3 front-right, M4 rear-right.
    // Yaw assumes M1/M3 spin CCW and M2/M4 spin CW when viewed from above.
    motorThrust.m1 = clampMotor(collectiveUs + rollCorrection + pitchCorrection + yawCorrection);
    motorThrust.m2 = clampMotor(collectiveUs + rollCorrection - pitchCorrection - yawCorrection);
    motorThrust.m3 = clampMotor(collectiveUs - rollCorrection - pitchCorrection + yawCorrection);
    motorThrust.m4 = clampMotor(collectiveUs - rollCorrection + pitchCorrection - yawCorrection);
}

void PID::writeIdle()
{
    motorThrust = {idleThrottleUs, idleThrottleUs, idleThrottleUs, idleThrottleUs};
}

float PID::clampMotor(float value)
{
    return constrain(value, escMinUs, escMaxUs);
}
