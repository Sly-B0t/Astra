#include "Motor.h"
#include <Arduino.h>
#include <ESP32Servo.h>

Motors::Motors()
{
}

void Motors::begin()
{
    for (int i = 0; i < 4; i++)
    {
        motors[i].power = 0;
        goalMotor[i].power = 0;
    }
    Serial.println("Attaching motors...");

    motor1.attach(MOTORFR, ESC_MIN_US, ESC_MAX_US);
    motor2.attach(MOTORFL, ESC_MIN_US, ESC_MAX_US);
    motor3.attach(MOTORBR, ESC_MIN_US, ESC_MAX_US);
    motor4.attach(MOTORBL, ESC_MIN_US, ESC_MAX_US);

    writeAllMotors(ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US);

    Serial.println("Motors set to 1000us idle.");
}

void Motors::writeAllMotors(int us1, int us2, int us3, int us4)
{
    motor1.writeMicroseconds(us1);
    motor2.writeMicroseconds(us2);
    motor3.writeMicroseconds(us3);
    motor4.writeMicroseconds(us4);
}

void Motors::setConstants(float kp, float ki, float kd)
{
    Motors::kp = kp;
    Motors::ki = ki;
    Motors::kd = kd;
}

void Motors::resetPID()
{
    prevRollError = 0;
    prevPitchError = 0;
    prevYawError = 0;
    yawIntegral = 0;
    pitchIntegral = 0;
    rollIntegral = 0;
}

void Motors::PID(InputStruct InputNow, InputStruct InputGoal)
{
    unsigned long now = millis();
    float dt = (now - prevTime) / 1000.0f;
    if (prevTime == 0)
    {
        prevTime = now;
        return;
    }
    prevTime = now;
    float throttle = InputGoal.throttle;

    float rollError = InputGoal.roll - InputNow.roll;
    float pitchError = InputGoal.pitch - InputNow.pitch;
    float yawError = InputGoal.yaw - InputNow.yaw;

    rollIntegral += rollError * dt;
    pitchIntegral += pitchError * dt;
    yawIntegral += yawError * dt;

    float rollDer = (rollError - prevRollError) / dt;
    float pitchDer = (pitchError - prevPitchError) / dt;
    float yawDer = (yawError - prevYawError) / dt;

    float rollOutput = kp * rollError + kd * rollDer + ki * rollIntegral;
    float pitchOutput = kp * pitchError + kd * pitchDer + ki * pitchIntegral;
    float yawOutput = kp * yawError + kd * yawDer + ki * yawIntegral;

    prevRollError = rollError;
    prevYawError = yawError;
    prevPitchError = pitchError;

    rollOutput = (rollOutput < -ROLL_CORRECTION_STRENGTH) ? -ROLL_CORRECTION_STRENGTH : rollOutput;
    rollOutput = (rollOutput > ROLL_CORRECTION_STRENGTH) ? ROLL_CORRECTION_STRENGTH : rollOutput;
    pitchOutput = (pitchOutput < -PITCH_CORRECTION_STRENGTH) ? -PITCH_CORRECTION_STRENGTH : pitchOutput;
    pitchOutput = (pitchOutput > PITCH_CORRECTION_STRENGTH) ? PITCH_CORRECTION_STRENGTH : pitchOutput;
    yawOutput = (yawOutput < -YAW_CORRECTION_STRENGTH) ? -YAW_CORRECTION_STRENGTH : yawOutput;
    yawOutput = (yawOutput > YAW_CORRECTION_STRENGTH) ? YAW_CORRECTION_STRENGTH : yawOutput;

    int m1 = constrain(throttle + pitchOutput + rollOutput - yawOutput, ESC_MIN_US, ESC_MAX_US);
    int m2 = constrain(throttle + pitchOutput - rollOutput + yawOutput, ESC_MIN_US, ESC_MAX_US);
    int m3 = constrain(throttle - pitchOutput + rollOutput + yawOutput, ESC_MIN_US, ESC_MAX_US);
    int m4 = constrain(throttle - pitchOutput - rollOutput - yawOutput, ESC_MIN_US, ESC_MAX_US);

    writeAllMotors(m1, m2, m3, m4);
}