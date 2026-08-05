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
    // TODO
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
