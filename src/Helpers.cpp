#include <Arduino.h>
#include "Helpers.h"
float batteryPercentFromVoltage(float voltage)
{
    float percent = (voltage - BATTERY_EMPTY_VOLTAGE) /
                    (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE) * 100.0f;

    return constrain(percent, 0.0f, 100.0f);
}
