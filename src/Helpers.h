struct ICM
{
    float gx, gy, gz;
    float ay, ax, az;
    float mx, my, mz;
    float temp;
};

struct Orientation
{
    float yaw, roll, pitch;
    float altitude;
};

struct Control
{
    int a, b, r1, l1; // X, Circle, R1, and L1
    float r2, l2;     // R2 and L2
    float ry;         // Right analog stick Y only
    // r1 and l1 for yaw
    // r2 and l2 for altitude
    // ry for angling down or up
    // x and circle for arming and disarming
};

// IO Pins
#define SERIALCLOCKPIN 9
#define SERIALDATAPIN 8

#define MOTOR1PIN 11
#define MOTOR2PIN 12
#define MOTOR3PIN 13
#define MOTOR4PIN 14
#define BATTERYADCINPUT 10
#define BAR_ADDR 0x5D
#define MPU_ADDR 0x68
#define LED_18 18

#define LPS_ERROR 1
#define IMU_ERROR 2
#define I2C_ERROR 3

/*


*/
void callError(int code)
{
    while (1)
    {
    }
}

struct Motors
{
    float m1, m2, m3, m4;
};