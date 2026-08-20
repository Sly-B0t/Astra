
#include <LPS.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Bluepad32.h>
#include <InputManager.h>
#include <Helpers.h>
#include <ESP32Servo.h>

LPS InputManager::ps;
Adafruit_ICM20948 InputManager::icm;

InputManager::InputManager()
{
    Serial.println("InputManager: begin.");
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    Serial.println("InputManager: Bluepad32 ready, pairing keys cleared. Put controller in pairing mode.");

    Serial.println("InputManager: initializing LPS barometer...");
    if (!ps.init())
    {
        Serial.println("InputManager: LPS barometer init failed.");
        callError(LPS_ERROR);
    }
    ps.enableDefault();
    Serial.println("InputManager: LPS barometer ready.");

    Serial.println("InputManager: initializing ICM-20948...");
    bool imuReady = false;
    const uint8_t imuAddresses[] = {MPU_ADDR, MPU_ADDR_ALT};
    for (int attempt = 0; attempt < 5 && !imuReady; attempt++)
    {
        for (uint8_t address : imuAddresses)
        {
            Serial.printf("InputManager: ICM-20948 init attempt %d at 0x%02X...\n", attempt + 1, address);
            if (icm.begin_I2C(address, &Wire))
            {
                Serial.printf("InputManager: ICM-20948 ready at 0x%02X.\n", address);
                imuReady = true;
                break;
            }
            delay(100);
        }
        delay(250);
    }

    if (!imuReady)
    {
        Serial.println("InputManager: ICM-20948 init failed.");
        callError(IMU_ERROR);
    }

    Serial.println("InputManager: attaching ESC outputs...");
    m1.setPeriodHertz(50);
    m2.setPeriodHertz(50);
    m3.setPeriodHertz(50);
    m4.setPeriodHertz(50);

    m1.attach(MOTOR1PIN, 1000, 2000);
    m2.attach(MOTOR2PIN, 1000, 2000);
    m3.attach(MOTOR3PIN, 1000, 2000);
    m4.attach(MOTOR4PIN, 1000, 2000);

    setMotors({1000, 1000, 1000, 1000});
    Serial.println("InputManager: motors idle at 1000us.");

    delay(3000);
    Serial.println("InputManager: complete.");
}

InputManager::~InputManager()
{
}

float InputManager::getAltitude()
{
    float pressure = ps.readPressureMillibars();
    return ps.pressureToAltitudeMeters(pressure);
}
float InputManager::getAverageTemperature()
{
    return (getTemperatureBar() + getTemperatureICM()) / 2;
}

float InputManager::getTemperatureBar()
{
    return ps.readTemperatureC();
}

float InputManager::getTemperatureICM()
{
    updateICM();
    return icm_struct.temp;
}

Orientation InputManager::getOrientation()
{
    return sensorFusion();
}

Orientation InputManager::sensorFusion()
{
    updateICM();
    orientation.altitude = getAltitude();

    unsigned long currentTime = micros();

    if (previousFusionTime == 0)
    {
        previousFusionTime = currentTime;
        return orientation;
    }

    float dt = (currentTime - previousFusionTime) / 1000000.0f;
    previousFusionTime = currentTime;

    if (dt <= 0.0f || dt > 0.1f)
    {
        return orientation;
    }

    float ax = icm_struct.ax;
    float ay = icm_struct.ay;
    float az = icm_struct.az;

    float gx = icm_struct.gx - gyroXOffset;
    float gy = icm_struct.gy - gyroYOffset;
    float gz = icm_struct.gz - gyroZOffset;

    float accelRoll = atan2f(ay, az) * 180.0f / PI;

    float accelPitch = atan2f(
                           -ax,
                           sqrtf(ay * ay + az * az)) *
                       180.0f / PI;

    float gyroRollRate = gx * 180.0f / PI;
    float gyroPitchRate = gy * 180.0f / PI;
    float gyroYawRate = gz * 180.0f / PI;

    const float alpha = 0.98f;

    orientation.roll =
        alpha * (orientation.roll + gyroRollRate * dt) +
        (1.0f - alpha) * accelRoll;

    orientation.pitch =
        alpha * (orientation.pitch + gyroPitchRate * dt) +
        (1.0f - alpha) * accelPitch;

    orientation.yaw += gyroYawRate * dt;

    if (orientation.yaw > 180.0f)
    {
        orientation.yaw -= 360.0f;
    }
    else if (orientation.yaw < -180.0f)
    {
        orientation.yaw += 360.0f;
    }

    return orientation;
}

void InputManager::setMotor(float speed, Servo motor)
{
    motor.writeMicroseconds(constrain(speed, 1000, 2000));
}

void InputManager::setMotors(Motors motors)
{

    m1.writeMicroseconds(constrain(motors.m1, 1000, 2000));
    m2.writeMicroseconds(constrain(motors.m2, 1000, 2000));
    m3.writeMicroseconds(constrain(motors.m3, 1000, 2000));
    m4.writeMicroseconds(constrain(motors.m4, 1000, 2000));
}

void InputManager::updateICM()
{
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);

    ICM newicm = {gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, accel.acceleration.x,
                  accel.acceleration.y, accel.acceleration.z,
                  mag.magnetic.x, mag.magnetic.y, mag.magnetic.z,
                  temp.temperature};
    icm_struct = newicm;
}

Orientation InputManager::getGoal()
{
}
