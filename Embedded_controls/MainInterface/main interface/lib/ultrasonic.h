#ifndef ultrasonic_h
#define ultrasonic_h

#define Trigger_pin	PD7	// Trigger pin bit
#define	Echo_pin PB0 // Echo pin bit
#define Echo_PIN PINB // Echo pin register
#define Trigger_DDR DDRD // Trigger data direction register
#define Echo_DDR DDRB // Echo data direction register
#define Trigger_PORT PORTD // Trigger port
#define Echo_PORT PORTB // Echo port
#define F_CPU 16000000 // Clock speed
#define FOSC 16000000 // Oscilator frequency
#define BAUD 9600 // BAUD rate

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

double usGetDistanceFloat();
double usGetAverageDistanceFloat(uint8_t numberOfDistances);

#endif