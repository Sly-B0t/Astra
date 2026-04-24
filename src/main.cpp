#include <Bluepad32.h>
#include <Arduino.h>
ControllerPtr controller;
struct InputStruct
{
    float throttle; //  between -1 -> 1 controlleed by R2 and L2
    float yaw;      // controlled by the Y of the left stick -1->1
    float pitch;    // controlled by the X of the left stick -1->1
    float roll;     // controlled by R1 and L1 so it has discrete values, -1,0,1,
};

float rollspeed = 1;
InputStruct newPilotInput; // THIS GETS SET BY THE CONTROLLER TASK
InputStruct PilotInput;    // THIS ONLY GETS CHANGED BY THE TASK THAT USES IT SO THE DATA NEVER CHANGES MID TASK

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl)
{

    if (controller == nullptr)
    {
        Serial.printf("CALLBACK: Controller is connected, index=%d\n");
        // Additionally, you can get certain gamepad properties like:
        // Model, VID, PID, BTAddr, flags, etc.
        ControllerProperties properties = ctl->getProperties();
        Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                      properties.product_id);
        controller = ctl;
    }
}

void onDisconnectedController(ControllerPtr ctl)
{
    if (controller == ctl)
    {
        Serial.printf("CALLBACK: Controller disconnected from index=%d\n");
        controller = nullptr;
    }
}

void dumpGamepad(ControllerPtr ctl)
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

void processGamepad(ControllerPtr ctl)
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
    dumpGamepad(ctl);
}

// gets controller input and adds it to a queue
void controllerInput(void *pvParameters)
{
    while (true)
    {
        BP32.update();

        if (controller && controller->isConnected())
        {
            newPilotInput = (InputStruct){
                (controller->throttle() - controller->brake()) / 1023.0f,
                controller->axisY() / 512.0f,
                controller->axisX() / 512.0f,
                (float)(controller->r1() - controller->l1())};
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
// gets IMU input calculates PID and sets motor PWM
void control(void *pvParameters)
{
    while (true)
    {
    }
}
// debug, maybe log data or make a web page
void telemetry(void *pvParameters)
{
    while (true)
    {
    }
}

void setup()
{
    Serial.begin(115200);

    xTaskCreate(
        controllerInput,    // function
        "Controller Input", // task name
        4096,               // stack size
        NULL,               // parameter
        2,                  // priority
        NULL                // task handle
    );

    xTaskCreate(
        control,
        "Control",
        4096,
        NULL,
        1,
        NULL);
    xTaskCreate(
        telemetry,
        "Telemetry",
        4096,
        NULL,
        3,
        NULL);

    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t *addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
}

// Arduino loop function. Runs in CPU 1.
void loop()
{
}