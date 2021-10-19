#ifndef main_interface_h
#define main_interface_h

#include "lib/gyro.h"
#include "lib/status.h"
#include "lib/motorcontrol.h"
#include "lib/ultrasonic.h"
#include "lib/infrared.h"

void main_init();
struct Status main_step(float positions[18]);
void main_step_without_sensors(float positions[18]);

#endif
