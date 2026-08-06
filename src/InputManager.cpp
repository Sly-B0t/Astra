
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
ControllerPtr InputManager::controller = nullptr;

namespace
{
void scanI2CBus()
{
    Serial.println("I2C scan: starting.");
    int found = 0;
    for (uint8_t address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0)
        {
            Serial.printf("I2C scan: found device at 0x%02X\n", address);
            found++;
        }
        delay(2);
    }
    Serial.printf("I2C scan: complete, found %d device(s).\n", found);
}
}

InputManager::InputManager()
{
    Serial.println("InputManager: begin.");
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t *addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    BP32.setup(&InputManager::onConnectedController, &InputManager::onDisconnectedController);
    BP32.forgetBluetoothKeys();
    Serial.println("InputManager: Bluepad32 ready, pairing keys cleared.");
    scanI2CBus();

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

void InputManager::onConnectedController(ControllerPtr ctl)
{

    if (controller == nullptr)
    {
        Serial.printf("CALLBACK: Controller is connected, index=%d\n", ctl->index());
        // Additionally, you can get certain gamepad properties like:
        // Model, VID, PID, BTAddr, flags, etc.
        ControllerProperties properties = ctl->getProperties();
        Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                      properties.product_id);
        controller = ctl;
    }
}

void InputManager::onDisconnectedController(ControllerPtr ctl)
{
    if (controller == ctl)
    {
        Serial.printf("CALLBACK: Controller disconnected from index=%d\n", ctl->index());
        controller = nullptr;
    }
}

void InputManager::dumpGamepad(ControllerPtr ctl)
{
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),       // Controller Index
        ctl->dpad(),        // D-pad
        ctl->buttons(),     // bitmask of pressed buttons
        ctl->axisX(),       // (-511 - 512) left X Axis
        ctl->axisY(),       // (-511 - 512) left Y axis
        ctl->axisRX(),      // (-511 - 512) right X axis
        ctl->axisRY(),      // (-511 - 512) right Y axis
        ctl->brake(),       // (0 - 1023): brake button
        ctl->throttle(),    // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(), // bitmask of pressed "misc" buttons
        ctl->gyroX(),       // Gyro X
        ctl->gyroY(),       // Gyro Y
        ctl->gyroZ(),       // Gyro Z
        ctl->accelX(),      // Accelerometer X
        ctl->accelY(),      // Accelerometer Y
        ctl->accelZ()       // Accelerometer Z
    );
}

void InputManager::processGamepad(ControllerPtr ctl)
{
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    if (ctl->a())
    {
        static int colorIdx = 0;
        // Some gamepads like DS4 and DualSense support changing the color LED.
        // It is possible to change it by calling:
        switch (colorIdx % 3)
        {
        case 0:
            // Red
            ctl->setColorLED(255, 0, 0);
            break;
        case 1:
            // Green
            ctl->setColorLED(0, 255, 0);
            break;
        case 2:
            // Blue
            ctl->setColorLED(0, 0, 255);
            break;
        }
        colorIdx++;
    }

    if (ctl->b())
    {
        // Turn on the 4 LED. Each bit represents one LED.
        static int led = 0;
        led++;
        // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
        // support changing the "Player LEDs": those 4 LEDs that usually indicate
        // the "gamepad seat".
        // It is possible to change them by calling:
        ctl->setPlayerLEDs(led & 0x0f);
    }

    if (ctl->x())
    {
        // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S, Stadia support rumble.
        // It is possible to set it by calling:
        // Some controllers have two motors: "strong motor", "weak motor".
        // It is possible to control them independently.
        ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */,
                            0x40 /* strongMagnitude */);
    }

    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    // dumpGamepad(ctl);
}

void InputManager::updateControllers()
{
    BP32.update();
}

bool InputManager::isControllerConnected() const
{
    return controller != nullptr && controller->isConnected();
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

Control InputManager::getController()
{
    if (!isControllerConnected())
    {
        return {0, 0, 0, 0, 0.0f, 0.0f, 0.0f};
    }

    Control c = {controller->a(), controller->b(), controller->r1(), controller->l1(),
                 controller->throttle() / 1023.0f, controller->brake() / 1023.0f,
                 controller->axisRY() / 512.0f};
    return c;
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
    static Orientation goal = {};

    if (!isControllerConnected())
    {
        goal.altitude = 0.0f;
        return goal;
    }

    Control c = getController();
    float climb = constrain(c.r2 - c.l2, -1.0f, 1.0f);
    float pitchCommand = constrain(-c.ry, -1.0f, 1.0f);
    float yawCommand = 0.0f;

    if (c.r1)
    {
        yawCommand += 1.0f;
    }
    if (c.l1)
    {
        yawCommand -= 1.0f;
    }

    goal.pitch = pitchCommand * 25.0f;
    goal.roll = 0.0f;
    goal.yaw += yawCommand * 2.0f;
    goal.altitude = constrain(climb, 0.0f, 1.0f);

    if (goal.yaw > 180.0f)
    {
        goal.yaw -= 360.0f;
    }
    else if (goal.yaw < -180.0f)
    {
        goal.yaw += 360.0f;
    }

    return goal;
}
