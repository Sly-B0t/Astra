#include <Helpers.h>

class PID
{
private:
    Orientation goalOrientation;
    Orientation actualOrientation;

public:
    PID();
    ~PID();
    Motors getMotorThrust();
    void setOrientation(Orientation goal, Orientation Actual);
};
