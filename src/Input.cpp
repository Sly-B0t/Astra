#include <Bluepad32.h>
#include <Arduino.h>
#include "Input.h"
#include "Helpers.h"
#include <MPU9250.h>

Inputs::Inputs()
{
}

void Inputs::begin()
{
    Wire.begin();
    delay(2000);
    mpu.setup(0x68);

    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());

    const uint8_t *addr = BP32.localBdAddress();

    Serial.printf("BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController);

    // Keep while testing. Comment out later if pairing becomes annoying.
    BP32.forgetBluetoothKeys();
}

InputStruct Inputs::GetInput()
{
    if (controller && controller->isConnected())
    {
        float r2 = controller->throttle() / 1023.0f;
        float l2 = controller->brake() / 1023.0f;

        float throttle = r2 - l2;
        throttle = constrain(throttle, 0.0f, 1.0f);

        float roll = controller->axisX() / 512.0f;
        float pitch = -controller->axisY() / 512.0f;

        roll = applyDeadzone(roll);
        pitch = applyDeadzone(pitch);

        float yaw = 0.0f;

        if (controller->r1())
        {
            yaw += 1.0f;
        }

        if (controller->l1())
        {
            yaw -= 1.0f;
        }

        InputGoal.throttle = throttle;
        InputGoal.roll = roll * 90;
        InputGoal.pitch = pitch * 90;
        InputGoal.yaw += yaw * 15;
    }
    return InputGoal;
}

InputStruct Inputs::GetIMU()
{
    if (mpu.update())
    {
        InputNow.roll = mpu.getRoll();
        InputNow.yaw = mpu.getYaw();
        InputNow.pitch = mpu.getPitch();
    }
    return InputNow;
}

float applyDeadzone(float x)
{
    if (fabsf(x) < DEADZONE)
    {
        return 0.0f;
    }

    return x;
}

void onConnectedController(ControllerPtr ctl)
{
    if (controller == nullptr)
    {
        Serial.println("Controller connected.");

        ControllerProperties properties = ctl->getProperties();

        Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n",
                      ctl->getModelName().c_str(),
                      properties.vendor_id,
                      properties.product_id);

        controller = ctl;
    }
}

void onDisconnectedController(ControllerPtr ctl)
{
    if (controller == ctl)
    {
        Serial.println("Controller disconnected. Disarming.");

        controller = nullptr;
        armed = false;

        writeAllMotors(ESC_MIN_US);
    }
}