#include "main_interface.h"
#include "lib/gyro.h"

void main_init()
{
    // Init motors
    Init();
    // Init gyro sensor
    SetupGyroSensor();
    // Init IR sensor
    infraredInit();
}

struct Status main_step(float positions[18])
{
    struct Status robot_status;

    // Do Step
    for (int a = 0; a < 18; a = a + 1)
    {
        changeMotorDestinationState(a, positions[a], false); // bool: clamping
    }
    goForDestinationPositions(false); // bool: without Timing

    // Get status from sensor values
    // US Sensor
    robot_status.distance = usGetDistanceFloat();
    // IR Sensor
    robot_status.direction = infraredGetDirection();
    // Gyro Sensor
    robot_status.gyro_data = GetGyroSensorData();

    return robot_status;
}

void main_step_without_sensors(float positions[18])
{
    // Do Step
    for (int a = 0; a < 18; a = a + 1)
    {
        changeMotorDestinationState(a, positions[a], false); // bool: clamping
    }
    goForDestinationPositions(false); // bool: without Timing
}