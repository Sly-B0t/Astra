#include <Arduino.h>

#define DEADZONE 0.10f

#define ESC_MIN_US 1000
#define ESC_MAX_US 2000

#define MAX_ANGLE_RAD (PI / 6.0f) // 30 degrees
#define MAX_PID_CORRECTION 250    // max PID correction in microseconds

#define ENABLE_BATTERY_CUTOFF 0
#define BATTERY_CUTOFF_VOLTAGE 9.0f

#define BATTERY_FULL_VOLTAGE 12.6f
#define BATTERY_EMPTY_VOLTAGE 9.0f

#define SDA_PIN 21
#define SCL_PIN 22

#define MOTOR1PIN 25
#define MOTOR2PIN 26
#define MOTOR3PIN 27
#define MOTOR4PIN 33

// CHANGE THIS TO GET CORRECT MOTOR ORIENTATION
#define MOTORFR MOTOR1PIN
#define MOTORFL MOTOR2PIN
#define MOTORBR MOTOR3PIN
#define MOTORBL MOTOR4PIN

#define ROLL_CORRECTION_STRENGTH 300
#define PITCH_CORRECTION_STRENGTH 300
#define YAW_CORRECTION_STRENGTH 300

#define BATTERYADCINPUT 32

#define PROPORTIONAL 1
#define INTEGRAL 1
#define DERIVITAVE 1

struct Motors
{
    float m1;
    float m2;
    float m3;
    float m4;
};

struct GoalStruct
{
    float throttle; // 0 -> 1
    float yaw;      // radians
    float pitch;    // radians
    float roll;     // radians
};

struct Attitude
{
    float pitch; // radians
    float roll;  // radians
    float yaw;   // radians
};

struct InputStruct
{
    float throttle; // 0 -> 1
    float yaw;      // -1 -> 1
    float pitch;    // -1 -> 1
    float roll;     // -1 -> 1
};

void writeAllMotors(int us);
float batteryPercentFromVoltage(float voltage);
void calibrateIMULevel();
void scanI2C();