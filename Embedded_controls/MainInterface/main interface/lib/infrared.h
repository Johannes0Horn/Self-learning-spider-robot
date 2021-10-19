#ifndef infrared
#define infrared

// Pins A0-A3
#define	Ir_right_pin PC3
#define	Ir_left_pin PC1
#define	Ir_front_pin PC2
#define	Ir_rear_pin PC0

#include "adc.h"

void infraredInit();
int infraredGetDirection();

#endif
