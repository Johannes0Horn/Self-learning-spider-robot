/*
This Code can be used to control a Lynxmotion SSC­32U Servo Controller Board.
How to use:
General:
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

#include "motorcontrol.h"

int maxbottomMotors = 2400;
int minbottomMotors = 700;

int maxmiddleMotoros = 2100;
int minmiddleMotoros = 600;

int maxtopMotoros = 1600;
int mintopMotoros = 1400;

const int numberOfMotors = 18;

//lay position
int lay_bottommotors = 2300;
int lay_middlemotors = 1500;
int lay_topmotors = 1500;

//crawling position
int crawl_bottommotors = 1300;
int crawl_middlemotors = 1200;
int crawl_topmotors = 1500;
int crawl_topCornerOffset = 500;

//standing position
int stand_bottommotors = 1500;
int stand_middlemotors = 1500;
int stand_topmotors = 1500;

//walkCicle Position
int lifted_middlemotors = 900;
int lifted_bottommotors = 1500;
int walk_moveOffset = 200;
int startWalkStates[18];

char motorCommandBuffer[300];
char usartReceiveStringBuffer[255];

char commandFreq_str[12];
//max steps per singel command per motor
int maxStepsPerCommand;
//contains absolute positions of all 18 motors in degrees: 0° = min , 180° = max
float motorStates[18];
//contains absolute destination positions of all 18 motors in degrees: 0° = min , 180° = max
float motorDestinationStates[18];

double round(double d)
{
    return floor(d + 0.5);
}

float getDegreeFromStep(int step)
{
    //0° = min , 180° = max
    //500 = min, 2500= max
    float slope = 1.0 * (180 - 0) / (2500 - 500);
    float degree = 0 + slope * (step - 500);
    return degree;
}

int getStepFromDegree(float degree)
{
    //0° = min , 180° = max
    //500 = min, 2500= max
    float slope = 1.0 * (2500 - 500) / (180 - 0);
    int step = 500 + round(slope * (degree - 0));
    return step;
}

//motornumber from 0-17
//destinationPosition from 0 to 180 °
void changeMotorDestinationState(int motornumber, float destinationPosition, bool clamp)
{ //clamp destinationPosition depending on max Motor Speed
    if (clamp && abs(getStepFromDegree(motorStates[motornumber]) - getStepFromDegree(destinationPosition)) > maxStepsPerCommand)
    {
        if (destinationPosition > motorStates[motornumber])
        {
            destinationPosition = motorStates[motornumber] + getDegreeFromStep(maxStepsPerCommand);
        }
        else
        {
            destinationPosition = motorStates[motornumber] - getDegreeFromStep(maxStepsPerCommand);
        }
    }
    motorDestinationStates[motornumber] = destinationPosition;
}

void updateMotorStates()
{
    for (int i = 0; i < numberOfMotors; i++)
    {
        motorStates[i] = motorDestinationStates[i];
    }
}

int getAbsoluteMotorPosition(int motornumber, int stepPosition, bool mirror_front)
{

    if (mirror_front)
    {
        if (motornumber <= 10 && motornumber != 1)
        {
            return stepPosition;
        }
        else
        {
            //assume 1500 is mid centered position
            return 3000 - stepPosition;
        }
    }
    else
    {
        if (motornumber <= 8)
        {
            return stepPosition;
        }

        else
        {
            //assume 1500 is mid centered position
            return 3000 - stepPosition;
        }
    }
}

void setBottomMotorDestinationStates(float destinationPositionInDegrees)
{
    int i;
    for (i = 1; i <= numberOfMotors; i++)
    {
        //get legPart from motornumber
        int legpart = (i - 1) % 3;
        if (legpart == 2)
        {
            changeMotorDestinationState(i, getDegreeFromStep(getAbsoluteMotorPosition(i, getStepFromDegree(destinationPositionInDegrees), false)), false);
        }
    }
}

void setMiddleMotorDestinationStates(float destinationPositionInDegrees)
{
    int i;
    for (i = 1; i <= numberOfMotors; i++)
    {
        //get legPart from motornumber
        int legpart = (i - 1) % 3;
        if (legpart == 1)
        {
            changeMotorDestinationState(i, getDegreeFromStep(getAbsoluteMotorPosition(i, getStepFromDegree(destinationPositionInDegrees), false)), false);
        }
    }
}

void setTopMotorDestinationStates(float destinationPositionInDegrees)
{
    int i;
    for (i = 0; i < numberOfMotors; i++)
    {
        //get legPart from motornumber
        int legpart = (i - 1) % 3;
        if (legpart == 0)
        {
            changeMotorDestinationState(i, getDegreeFromStep(getAbsoluteMotorPosition(i, getStepFromDegree(destinationPositionInDegrees), false)), false);
        }
    }
}

float getMotorDestinaionInDegree(int motor)
{
    return motorDestinationStates[motor - 1];
}

float getMotorStateInDegree(int motor)
{
    return motorStates[motor - 1];
}

void setCornerTopMotorDestinationStates(float destinationPositionInDegrees)
{
    changeMotorDestinationState(1, getDegreeFromStep(getAbsoluteMotorPosition(1, getStepFromDegree(destinationPositionInDegrees), true)), false);
    changeMotorDestinationState(7, getDegreeFromStep(getAbsoluteMotorPosition(7, getStepFromDegree(destinationPositionInDegrees), true)), false);
    changeMotorDestinationState(10, getDegreeFromStep(getAbsoluteMotorPosition(10, getStepFromDegree(destinationPositionInDegrees), true)), false);
    changeMotorDestinationState(16, getDegreeFromStep(getAbsoluteMotorPosition(16, getStepFromDegree(destinationPositionInDegrees), true)), false);
}

void setLayPositionDestinations()
{
    int a;
    for (a = 0; a < 18; a = a + 1)
    {
        changeMotorDestinationState(a, 90, false);
    }

    //setTopMotorDestinationStates(getDegreeFromStep(lay_topmotors));
    //setMiddleMotorDestinationStates(getDegreeFromStep(lay_middlemotors));
    //setBottomMotorDestinationStates(getDegreeFromStep(lay_bottommotors));
}
void setCrawlPositionDestinations()
{
    setTopMotorDestinationStates(getDegreeFromStep(crawl_topmotors));
    setMiddleMotorDestinationStates(getDegreeFromStep(crawl_middlemotors));
    setBottomMotorDestinationStates(getDegreeFromStep(crawl_bottommotors));
    setCornerTopMotorDestinationStates(getDegreeFromStep(crawl_topmotors + crawl_topCornerOffset));
}

void setStandPositionDestinations()
{
    setTopMotorDestinationStates(getDegreeFromStep(stand_topmotors));
    setMiddleMotorDestinationStates(getDegreeFromStep(stand_middlemotors));
    setBottomMotorDestinationStates(getDegreeFromStep(stand_bottommotors));
}

/// Initialisation

int getPinFromMotorNumber(int motorNumber)
{
    int pinNumber;
    switch (motorNumber)
    {
    case 0:
        pinNumber = 24;
        break;
    case 1:
        pinNumber = 25;
        break;
    case 2:
        pinNumber = 26;
        break;
    case 3:
        pinNumber = 20;
        break;
    case 4:
        pinNumber = 21;
        break;
    case 5:
        pinNumber = 22;
        break;
    case 6:
        pinNumber = 16;
        break;
    case 7:
        pinNumber = 17;
        break;
    case 8:
        pinNumber = 18;
        break;
    case 9:
        pinNumber = 8;
        break;
    case 10:
        pinNumber = 9;
        break;
    case 11:
        pinNumber = 10;
        break;
    case 12:
        pinNumber = 4;
        break;
    case 13:
        pinNumber = 5;
        break;
    case 14:
        pinNumber = 6;
        break;
    case 15:
        pinNumber = 0;
        break;
    case 16:
        pinNumber = 1;
        break;
    case 17:
        pinNumber = 2;
        break;
    default:
        pinNumber = 0;
        break;
    }
    return pinNumber;
}
int getPinFromMotorNumberMirrored(int motorNumber)
{
    int pinNumber;
    switch (motorNumber)
    {
    case 1:
        pinNumber = 0;
        break;
    case 2:
        pinNumber = 1;
        break;
    case 3:
        pinNumber = 2;
        break;
    case 4:
        pinNumber = 4;
        break;
    case 5:
        pinNumber = 5;
        break;
    case 6:
        pinNumber = 6;
        break;
    case 7:
        pinNumber = 8;
        break;
    case 8:
        pinNumber = 9;
        break;
    case 9:
        pinNumber = 10;
        break;
    case 10:
        pinNumber = 16;
        break;
    case 11:
        pinNumber = 17;
        break;
    case 12:
        pinNumber = 18;
        break;
    case 13:
        pinNumber = 20;
        break;
    case 14:
        pinNumber = 21;
        break;
    case 15:
        pinNumber = 22;
        break;
    case 16:
        pinNumber = 24;
        break;
    case 17:
        pinNumber = 25;
        break;
    case 18:
        pinNumber = 26;
        break;
    default:
        pinNumber = 0;
        break;
    }
    return pinNumber;
}

void appendToMotorCommandBuffer(char appendix[])
{
    if (strlen(appendix) + strlen(motorCommandBuffer) <= 255)
    {
        strncat(motorCommandBuffer, appendix, 255);
    }
}

void setMotorCommandBufferToDestinationStates(bool motorInit)
{ //reset motorCommandBuffer to empty string
    strncpy(motorCommandBuffer, "", 255);

    int i;
    for (i = 0; i < numberOfMotors; i++)
    {  



        //get destination position
        int destinationPosition = getStepFromDegree(motorDestinationStates[i]);
         //mirror where necessary
        destinationPosition = getAbsoluteMotorPosition(i, destinationPosition, false);
        //convert i(motornumber) to string
        char motorNumber_str[12];
        sprintf(motorNumber_str, "%d", getPinFromMotorNumber(i));
        //convert destinationPosition to string
        char destinationPosition_str[12];
        sprintf(destinationPosition_str, "%d", destinationPosition);
        //create currentcommand and append to toal command
        //char currentCommandbuffer[255];
        //if motor init, only use Channel and Positions, no Time, because the motorcontroller doesnt know the init motorpositions
        appendToMotorCommandBuffer("#");
        appendToMotorCommandBuffer(motorNumber_str);
        appendToMotorCommandBuffer("P");
        appendToMotorCommandBuffer(destinationPosition_str);

        if (!motorInit)
        {
            appendToMotorCommandBuffer("T");
            appendToMotorCommandBuffer(commandFreq_str);
        }
    }
}

void goForDestinationPositions(bool init) // meaning of bool "init": if without Timing
{
    //prepare command
    setMotorCommandBufferToDestinationStates(init);
    USART_println(motorCommandBuffer);
    //empty  buffer
    motorCommandBuffer[0] = '\0';
    updateMotorStates();
}

int allActionsCompleted()
{
    //toto println vs print \r speed test

    USART_print("Q\r");

    char indicationChar = USART_getc();

    char completed = '.';
    char inProgress = '+';
    if (indicationChar == completed)
    {
        return 1;
    }
    else if (indicationChar == inProgress)
    {
        return 0;
    }
    else
    {
        return 2;
    }
}

void setMotorsToStartPosition()
{
    //set Destination States to Start positions
    setLayPositionDestinations();
    //send command destination states
    goForDestinationPositions(true);
}
void setMotorsToCrawlPosition()
{
    setCrawlPositionDestinations();
    goForDestinationPositions(true);
}
void setMotorsToStandPosition()
{
    setStandPositionDestinations();
    goForDestinationPositions(true);
}
void calibrateMotors()
{ //empty  buffer
    memset(&motorCommandBuffer[0], 0, sizeof(motorCommandBuffer));
    //motorCommandBuffer[0] = '\0';
    //When calibrating also send Position commands, so motors wont behave randomly while calibrating*/
    setLayPositionDestinations();
    setMotorCommandBufferToDestinationStates(true);
    USART_println(motorCommandBuffer);
    //empty  buffer
    motorCommandBuffer[0] = '\0';

    //top leg motors
    char top_leg_motor_calibrations[55] = "#0PO100 #4PO-80 #8PO0 #16PO-50 #20PO-50 #24PO0";
    //middle leg motors
    char middle_leg_motor_calibrations[55] = "#1PO0 #5PO-50 #9PO-100 #17PO-50 #21PO0 #25PO0";
    //bottom leg motors
    char bottom_leg_motor_calibrations[55] = "#2PO20 #6PO-30 #10PO-100 #18PO100 #22PO-30 #26PO20";

    appendToMotorCommandBuffer(top_leg_motor_calibrations);
    appendToMotorCommandBuffer(middle_leg_motor_calibrations);
    appendToMotorCommandBuffer(bottom_leg_motor_calibrations);
    USART_println(motorCommandBuffer);
    setLayPositionDestinations();
    setMotorCommandBufferToDestinationStates(true);
    USART_println(motorCommandBuffer);
    //empty  buffer
    motorCommandBuffer[0] = '\0';
    updateMotorStates();
}

//call     "setMotorsToCrawlPosition()"  and "setStartWalkStates()" before walking
void doOneWalkCycle()
{
    //using tripod gait
    int tripod_1_topmotors[3] = {4, 10, 16};
    int tripod_1_middlemotors[3] = {5, 11, 17};
    int tripod_1_bottommotors[3] = {6, 12, 18};

    int tripod_2_topmotors[3] = {1, 7, 13};
    int tripod_2_middlemotors[3] = {2, 8, 14};
    int tripod_2_bottommotors[3] = {3, 9, 15};

    _delay_ms(500);

    //frame 0: lift tripod 1, drop tripod 2 using middlemotors and bottommotors
    for (int i = 0; i < 3; i++)
    {
        changeMotorDestinationState(tripod_1_middlemotors[i], getDegreeFromStep(getAbsoluteMotorPosition(tripod_1_middlemotors[i], lifted_middlemotors, false)), false);
        changeMotorDestinationState(tripod_1_bottommotors[i], getDegreeFromStep(getAbsoluteMotorPosition(tripod_1_bottommotors[i], lifted_bottommotors, false)), false);

        changeMotorDestinationState(tripod_2_middlemotors[i], getDegreeFromStep(getAbsoluteMotorPosition(tripod_2_middlemotors[i], crawl_middlemotors, false)), false);
        changeMotorDestinationState(tripod_2_bottommotors[i], getDegreeFromStep(getAbsoluteMotorPosition(tripod_2_bottommotors[i], crawl_bottommotors, false)), false);
    }
    goForDestinationPositions(true);
    _delay_ms(1000);

    //frame 1: move touching legs(tripod 2) back and lifted legs (tripod 1) forward using topmotors
    //touching legs back
    changeMotorDestinationState(tripod_2_topmotors[0], getDegreeFromStep(startWalkStates[tripod_2_topmotors[0] - 1] + walk_moveOffset), false);
    changeMotorDestinationState(tripod_2_topmotors[1], getDegreeFromStep(startWalkStates[tripod_2_topmotors[1] - 1] - walk_moveOffset), false);
    changeMotorDestinationState(tripod_2_topmotors[2], getDegreeFromStep(startWalkStates[tripod_2_topmotors[2] - 1] - walk_moveOffset), false);

    //lifted legs  forward
    changeMotorDestinationState(tripod_1_topmotors[0], getDegreeFromStep(startWalkStates[tripod_1_topmotors[0] - 1] - walk_moveOffset), false);
    changeMotorDestinationState(tripod_1_topmotors[1], getDegreeFromStep(startWalkStates[tripod_1_topmotors[1] - 1] + walk_moveOffset), false);
    changeMotorDestinationState(tripod_1_topmotors[2], getDegreeFromStep(startWalkStates[tripod_1_topmotors[2] - 1] + walk_moveOffset), false);

    goForDestinationPositions(true);
    _delay_ms(500);

    //frame 2: lift tripod 2, drop tripod 1 using middlemotors and bottommotors
    for (int i = 0; i < 3; i++)
    {
        changeMotorDestinationState(tripod_1_middlemotors[i], getDegreeFromStep(getAbsoluteMotorPosition(tripod_1_middlemotors[i], crawl_middlemotors, false)), false);
        changeMotorDestinationState(tripod_1_bottommotors[i], getDegreeFromStep(getAbsoluteMotorPosition(tripod_1_bottommotors[i], crawl_bottommotors, false)), false);

        changeMotorDestinationState(tripod_2_middlemotors[i], getDegreeFromStep(getAbsoluteMotorPosition(tripod_2_middlemotors[i], lifted_middlemotors, false)), false);
        changeMotorDestinationState(tripod_2_bottommotors[i], getDegreeFromStep(getAbsoluteMotorPosition(tripod_2_bottommotors[i], lifted_bottommotors, false)), false);
    }
    goForDestinationPositions(true);
    _delay_ms(500);
    //frame 3: move touching legs(tripod 1) back and lifted legs (tripod 2) forward using topmotors
    //touching legs back
    changeMotorDestinationState(tripod_2_topmotors[0], getDegreeFromStep(startWalkStates[tripod_2_topmotors[0] - 1] - walk_moveOffset), false);
    changeMotorDestinationState(tripod_2_topmotors[1], getDegreeFromStep(startWalkStates[tripod_2_topmotors[1] - 1] + walk_moveOffset), false);
    changeMotorDestinationState(tripod_2_topmotors[2], getDegreeFromStep(startWalkStates[tripod_2_topmotors[2] - 1] + walk_moveOffset), false);

    //lifted legs  forward
    changeMotorDestinationState(tripod_1_topmotors[0], getDegreeFromStep(startWalkStates[tripod_1_topmotors[0] - 1] + walk_moveOffset), false);
    changeMotorDestinationState(tripod_1_topmotors[1], getDegreeFromStep(startWalkStates[tripod_1_topmotors[1] - 1] - walk_moveOffset), false);
    changeMotorDestinationState(tripod_1_topmotors[2], getDegreeFromStep(startWalkStates[tripod_1_topmotors[2] - 1] - walk_moveOffset), false);

    goForDestinationPositions(true);
    _delay_ms(1000);
}

void setStartWalkStates()
{
    //save Start states in steps
    for (int i = 0; i < 18; i++)
    {
        startWalkStates[i] = getStepFromDegree(motorStates[i]);
    }
}

void Init()
{
    /*sets necessary varaibles, calibrates motors and sets motors to start position*/

    USART_init(MYUBRR);
    //max steps per singel command per motor
    maxStepsPerCommand = commandFreq / maxSingleStepSpeedPer2000; //~285 ~= 25° (for commandFreq=100 and maxSingleStepSpeedPer2000 = 700)
    //convert commandFreq to string so it can be used in a command
    sprintf(commandFreq_str, "%d", commandFreq);
    calibrateMotors();
    _delay_ms(1000);
    setMotorsToStartPosition();
    _delay_ms(2000);
}