/*
This Code can be used to control a Lynxmotion SSC­32U Servo Controller Board.
How to use:

Init:
1.) Make sure call "Init()" once first to calibrate and set motors to start position.

For each motormovement:
//NOT WORKING : 1.) Make sure all servos have completed their previous moves by calling "allActionsCompleted()"
2.) Set desired motor postions using changeMotorDestinationState(int motornumber(1-18),float destinationPosition(in degrees)).
3.) Send Command to servo controller by calling "goForDestinationPositions()".

important settings:
1.) Step start positions(500-2500) of motors via:
startbottommotors
startmiddlemotors
starttopmotors
2.) How often will a command be send to the controller in milliseconds? via:
commandFreq

 */

#ifndef motorcontrol
#define motorcontrol


#define FOSC 16000000
#define F_CPU 16000000 // Clock Speed, important to define before <delay.h> is included.

#define BAUD 9600
#define MYUBRR FOSC / 16 / BAUD - 1

#include <avr/io.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include "usart.h"

//maxSpeed is about 600ms / 2000 steps we use 700ms / 2000 steps to be sure
#define maxSingleStepSpeedPer2000 700
//command frequency in ms
#define commandFreq 100


double round(double d);


float getDegreeFromStep(int step);


int getStepFromDegree(float degree);


//motornumber from 1-18
//destinationPosition from 0 to 180 °
void changeMotorDestinationState(int motornumber, float destinationPosition, bool clamp);

void updateMotorStates();

int getAbsoluteMotorPosition(int motornumber, int stepPosition, bool mirror_front);

void setBottomMotorDestinationStates(float destinationPositionInDegrees);

void setMiddleMotorDestinationStates(float destinationPositionInDegrees);

void setTopMotorDestinationStates(float destinationPositionInDegrees);

float getMotorDestinaionInDegree(int motor);

float getMotorStateInDegree(int motor);

void setCornerTopMotorDestinationStates(float destinationPositionInDegrees);

void setLayPositionDestinations();

void setCrawlPositionDestinations();

void setStandPositionDestinations();

/// Initialisation

int getPinFromMotorNumber(int motorNumber);

void appendToMotorCommandBuffer(char appendix[]);

void setMotorCommandBufferToDestinationStates(bool motorInit);

void goForDestinationPositions(bool init); // meaning of bool "init": if without Timing

int allActionsCompleted();

void setMotorsToStartPosition();

void setMotorsToCrawlPosition();

void setMotorsToStandPosition();

void calibrateMotors();

//call     "setMotorsToCrawlPosition()"  and "setStartWalkStates()" before walking
void doOneWalkCycle();

void setStartWalkStates();

void Init();



#endif
