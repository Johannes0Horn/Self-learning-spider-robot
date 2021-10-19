# Lynxmotion SSC-32U Motorcontrol Documentation

## Basic principle of work
This AVR Code can be used to control a Lynxmotion SSC 32U Servo Controller Board with 18 motors.
The basic principle of work: 
- There are 18 Motors (#1-18). Each motor can be in position 500-2500.
- The microcontroller sends a single serial string to the servoontroller to control all 18 servomotors. 
For example: "#5 P1600 T100" means motor 5 moves to Position 1600 within 100ms.

## How to use
Init:
- Make sure call "Init()" once first to calibrate and set motors to start position.

Each Time to move one or more motors:
- NOT WORKING FOR ARDUINO NANO: 1.) Make sure all servos have completed their previous moves by calling "allActionsCompleted()"
- Set desired motor postions using changeMotorDestinationState(int motornumber(1-18),float destinationPosition(in degrees)).
- Send Command to servo controller by calling "goForDestinationPositions()".

###Important settings in `motorcontrol.h`:
* `F_CPU`: Clock speed
* `FOSC`: Oscilator frequency
* `BAUD`: BAUD rate
* `commandFreq`: How often will a command be send to the controller in milliseconds?

###Additional Features
- 3 Preprogrammed positions: lay, crawl, stand. Can be used by calling setMotorsToStartPosition(), setMotorsToCrawlPosition() and setMotorsToStandPosition().
- unripe walkcycle:
--call "setMotorsToCrawlPosition()"  and "setStartWalkStates()" before walking. Then simply call doOneWalkCycle().

### Example Usage
```c++
#include "motorcontrol.h"
int main(void)
{
	Init();
	/*PARAMS: motornumber, destinationPosition, clamp?*/
	changeMotorDestinationState(1,  90, false);
	changeMotorDestinationState(2, 100, false);
	changeMotorDestinationState(3, 110, false);

	goForDestinationPositions(false); // meaning of bool: if without Timing


	while (1)
	{
		
	}
	return 0;
}
```

## Dependencies
* avr/io.h
* math.h
* stdbool.h
* stdint.h
* stdio.h
* stdlib.h
* string.h
* util/delay.h